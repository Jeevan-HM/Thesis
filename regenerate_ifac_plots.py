#!/usr/bin/env python3
"""
Regenerate the data-driven plots used in the IFAC paper
"Design of Pneumatic Soft Robots for Physical Reservoir Computing".

What this script regenerates
----------------------------
- Figure 2: performance comparison
- Figure 3: signal diversity example
- Figure 4: operating regime / pre-inflation analysis
- Figure 5: sensor ablation (parallel topology)
- Per-trial and aggregate metric CSV files

What this script does not attempt to rebuild
--------------------------------------------
- Figure 1, which is a mixed photo + schematic artwork rather than a data plot
- The small trajectory icons in Table 1

Notes
-----
The bundled figure file for panel 4(b) shows mean pouch signal amplitude
(mean pouch standard deviation), while the LaTeX caption text in the source
mentions inter-pouch correlation. To match the rendered figure that ships with
this paper package, the default behavior here reproduces the *signal amplitude*
version. Pass ``--panel4b-mode corr`` if you want the correlation variant
instead.

Figure 2 panels (a) and (b) apply light preprocessing to the showcase
time-series traces:
  - Panel (a) Parallel: 0.5 Hz low-pass on sensors, delay=40, alpha=0.0005
  - Panel (b) Coupled:  1.0 Hz low-pass on sensors, delay=20, alpha=0.01
The box-plot and DDI panels use the original raw-sensor pipeline so that
aggregate metrics remain unchanged.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import re
import shutil
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import butter, correlate, filtfilt

POUCH_COLS = [f"Measured_pressure_Segment_1_pouch_{i}" for i in range(1, 6)]
INPUT_COLS = [f"Desired_pressure_segment_{i}" for i in range(2, 5)]
Q1_COLS = [f"Rigid_body_1_q{c}" for c in "xyzw"]
Q3_COLS = [f"Rigid_body_3_q{c}" for c in "xyzw"]
REQUIRED_COLS = ["time", *INPUT_COLS, *POUCH_COLS, *Q1_COLS, *Q3_COLS]
ALL_POUCHS = tuple(range(5))
DELAY = 20
RIDGE_ALPHA = 0.01
TRIM_SEC = 10.0
PRED_HORIZON_SEC = 1.0
MEMORY_K = 40
MAX_XCORR_LAG_SEC = 15.0


@dataclass(frozen=True)
class TrialMeta:
    path: Path
    topology: str
    waveform: str
    preinflation_psi: int
    pmax_psi: int
    label: str


@dataclass
class TrialData:
    meta: TrialMeta
    time: np.ndarray  # (n,)
    theta_deg: np.ndarray  # (n,)
    sensors: np.ndarray  # (n, 5)
    inputs: np.ndarray  # (n, 3)
    dt: float
    sensor_windows: np.ndarray  # (n-delay+1, delay, 5)


@dataclass
class PredictionResult:
    time: np.ndarray
    y_true: np.ndarray
    y_pred: np.ndarray
    nmse: float
    rmse_deg: float


@dataclass
class PredictionCache:
    cols_all: Tuple[int, ...]
    gram: np.ndarray
    rhs: np.ndarray
    x_test_z: np.ndarray
    y_test: np.ndarray
    y_mean: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-source",
        type=Path,
        default=Path("data/Data.zip"),
        help="Path to Data.zip or to an already-extracted data directory.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("data/regenerated_plots"),
        help="Directory where regenerated figures and metric files are written.",
    )
    parser.add_argument(
        "--panel4b-mode",
        choices=("sigma", "corr"),
        default="sigma",
        help="Use mean pouch sigma (matches bundled figure) or inter-pouch correlation.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=200,
        help="PNG export DPI.",
    )
    return parser.parse_args()


def maybe_extract_data(
    data_source: Path,
) -> Tuple[Path, tempfile.TemporaryDirectory | None]:
    """Return the directory that contains `parallel/` and `coupled/`."""
    if data_source.is_dir():
        if (data_source / "parallel").exists() and (data_source / "coupled").exists():
            return data_source, None
        candidates = [p for p in data_source.iterdir() if p.is_dir()]
        for cand in candidates:
            if (cand / "parallel").exists() and (cand / "coupled").exists():
                return cand, None
        raise FileNotFoundError(
            f"Could not find parallel/ and coupled/ under directory: {data_source}"
        )

    if not data_source.is_file() or data_source.suffix.lower() != ".zip":
        raise FileNotFoundError(
            f"Expected a directory or .zip file, got: {data_source}"
        )

    tmpdir = tempfile.TemporaryDirectory(prefix="paper_plot_data_")
    root = Path(tmpdir.name)
    with zipfile.ZipFile(data_source, "r") as zf:
        members = [m for m in zf.namelist() if not m.startswith("__MACOSX/")]
        zf.extractall(root, members)
    if (root / "parallel").exists() and (root / "coupled").exists():
        return root, tmpdir
    raise FileNotFoundError(
        f"Zip extraction succeeded, but parallel/ and coupled/ were not found inside {data_source}"
    )


def parse_trial_name(path: Path) -> TrialMeta:
    name = path.stem
    name_l = name.lower()
    topology = "parallel" if "parallel" in name_l else "coupled"
    m = re.search(r"(axial|circular|triangular|triangle)_([123])-([0-9]+)_", name_l)
    if not m:
        raise ValueError(f"Could not parse trial metadata from filename: {path.name}")
    waveform = m.group(1)
    if waveform == "triangle":
        waveform = "triangular"
    preinflation_psi = int(m.group(2))
    pmax_psi = int(m.group(3))
    label = f"{waveform}_{preinflation_psi}-{pmax_psi}_{topology}"
    return TrialMeta(
        path=path,
        topology=topology,
        waveform=waveform,
        preinflation_psi=preinflation_psi,
        pmax_psi=pmax_psi,
        label=label,
    )


def quaternion_inverse(q: np.ndarray) -> np.ndarray:
    out = q.copy()
    out[:, :3] *= -1.0
    return out


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1.T
    x2, y2, z2, w2 = q2.T
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.column_stack([x, y, z, w])


def compute_theta_deg(df: pd.DataFrame) -> np.ndarray:
    q1 = df[Q1_COLS].to_numpy(dtype=float).copy()
    q3 = df[Q3_COLS].to_numpy(dtype=float).copy()
    q1 /= np.linalg.norm(q1, axis=1, keepdims=True)
    q3 /= np.linalg.norm(q3, axis=1, keepdims=True)
    q_rel = quaternion_multiply(quaternion_inverse(q1), q3)
    q_rel_w = np.clip(np.abs(q_rel[:, 3]), 0.0, 1.0)
    return np.degrees(2.0 * np.arccos(q_rel_w))


def build_tapped_windows(data: np.ndarray, delay: int) -> np.ndarray:
    n, d = data.shape
    if n < delay:
        raise ValueError(f"Need at least {delay} rows, got {n}")
    out = np.empty((n - delay + 1, delay, d), dtype=np.float32)
    for end_idx in range(delay - 1, n):
        out[end_idx - delay + 1] = data[end_idx - delay + 1 : end_idx + 1][::-1]
    return out


def load_trial(meta: TrialMeta) -> TrialData:
    df = pd.read_csv(meta.path, usecols=REQUIRED_COLS)
    df = df.apply(pd.to_numeric, errors="coerce")
    df = df.dropna().sort_values("time").reset_index(drop=True)

    # Keep strictly increasing timestamps to avoid duplicated rows.
    dt_raw = df["time"].diff().fillna(1.0)
    df = df.loc[dt_raw > 0].copy().reset_index(drop=True)

    # Convert time to start at zero and trim the first/last 10 s.
    df["time"] = df["time"] - float(df["time"].iloc[0])
    tmax = float(df["time"].iloc[-1])
    df = df.loc[(df["time"] >= TRIM_SEC) & (df["time"] <= tmax - TRIM_SEC)].copy()
    df.reset_index(drop=True, inplace=True)

    time = df["time"].to_numpy(dtype=float)
    theta_deg = compute_theta_deg(df)
    sensors = df[POUCH_COLS].to_numpy(dtype=float)
    inputs = df[INPUT_COLS].to_numpy(dtype=float)
    dt = float(np.median(np.diff(time)))
    sensor_windows = build_tapped_windows(sensors, DELAY)

    return TrialData(
        meta=meta,
        time=time,
        theta_deg=theta_deg,
        sensors=sensors,
        inputs=inputs,
        dt=dt,
        sensor_windows=sensor_windows,
    )


def fit_ridge_predict(
    X: np.ndarray,
    y: np.ndarray,
    *,
    alpha: float = RIDGE_ALPHA,
    train_fraction: float = 0.7,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    split = int(train_fraction * len(X))
    if split <= 0 or split >= len(X):
        raise ValueError("Train/test split is empty. Check data length after trimming.")

    X_train = np.asarray(X[:split], dtype=float)
    X_test = np.asarray(X[split:], dtype=float)
    y_train = np.asarray(y[:split], dtype=float)
    y_test = np.asarray(y[split:], dtype=float)

    x_mean = X_train.mean(axis=0)
    x_std = X_train.std(axis=0)
    x_std[x_std < 1e-12] = 1.0

    X_train_z = (X_train - x_mean) / x_std
    X_test_z = (X_test - x_mean) / x_std

    y_is_vector = y_train.ndim == 1
    if y_is_vector:
        y_train = y_train[:, None]
        y_test = y_test[:, None]

    y_mean = y_train.mean(axis=0, keepdims=True)
    Y_train_c = y_train - y_mean

    A = X_train_z.T @ X_train_z
    A.flat[:: A.shape[0] + 1] += alpha
    B = X_train_z.T @ Y_train_c
    coef = np.linalg.solve(A, B)
    y_pred = X_test_z @ coef + y_mean

    if y_is_vector:
        y_test = y_test.ravel()
        y_pred = y_pred.ravel()

    return y_test, y_pred, np.arange(split, len(X))


def nmse(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    denom = float(np.sum((y_true - np.mean(y_true)) ** 2))
    if denom <= 1e-12:
        return float("nan")
    return float(np.sum((y_true - y_pred) ** 2) / denom)


def rmse_deg(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y_true - y_pred) ** 2)))


def lowpass_filter_sensors(
    sensors: np.ndarray, dt: float, cutoff_hz: float
) -> np.ndarray:
    """Zero-phase Butterworth low-pass applied column-wise to sensor data."""
    fs = 1.0 / dt
    nyq = 0.5 * fs
    cutoff = min(cutoff_hz, 0.45 * fs)
    b, a = butter(3, cutoff / nyq, btype="low")
    out = np.empty_like(sensors)
    for k in range(sensors.shape[1]):
        out[:, k] = filtfilt(b, a, sensors[:, k])
    return out


def prc_prediction_custom(
    trial: TrialData,
    *,
    delay: int = DELAY,
    alpha: float = RIDGE_ALPHA,
    lp_cutoff_hz: float | None = None,
    subset: Sequence[int] = ALL_POUCHS,
) -> PredictionResult:
    """Predict theta with optional sensor low-pass and custom delay/alpha."""
    sensors = trial.sensors
    if lp_cutoff_hz is not None:
        sensors = lowpass_filter_sensors(sensors, trial.dt, lp_cutoff_hz)

    windows = build_tapped_windows(sensors, delay)
    subset_idx = np.array(subset, dtype=int)
    horizon = max(1, int(round(PRED_HORIZON_SEC / trial.dt)))
    X = windows[:, :, subset_idx].reshape(len(windows), -1)
    X = X[:-horizon]
    y = trial.theta_deg[delay - 1 + horizon :]
    y_test, y_pred, idx = fit_ridge_predict(X, y, alpha=alpha)
    t = trial.time[delay - 1 + horizon :][idx]
    return PredictionResult(
        time=t,
        y_true=y_test,
        y_pred=y_pred,
        nmse=nmse(y_test, y_pred),
        rmse_deg=rmse_deg(y_test, y_pred),
    )


def prc_prediction(
    trial: TrialData, subset: Sequence[int] = ALL_POUCHS
) -> PredictionResult:
    subset_idx = np.array(subset, dtype=int)
    horizon = max(1, int(round(PRED_HORIZON_SEC / trial.dt)))
    X = trial.sensor_windows[:, :, subset_idx].reshape(len(trial.sensor_windows), -1)
    X = X[:-horizon]
    y = trial.theta_deg[DELAY - 1 + horizon :]
    y_test, y_pred, idx = fit_ridge_predict(X, y)
    t = trial.time[DELAY - 1 + horizon :][idx]
    return PredictionResult(
        time=t,
        y_true=y_test,
        y_pred=y_pred,
        nmse=nmse(y_test, y_pred),
        rmse_deg=rmse_deg(y_test, y_pred),
    )


def compute_pairwise_corr_stats(sensors: np.ndarray) -> Tuple[float, np.ndarray]:
    corr = np.corrcoef(sensors, rowvar=False)
    triu = corr[np.triu_indices(corr.shape[0], k=1)]
    return float(np.mean(triu)), corr


def pca_from_covariance(sensors: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    centered = sensors - sensors.mean(axis=0, keepdims=True)
    cov = np.cov(centered, rowvar=False)
    eig = np.linalg.eigvalsh(cov)[::-1]
    eig = np.clip(eig, 0.0, None)
    if eig.sum() <= 0:
        ratio = np.zeros_like(eig)
    else:
        ratio = eig / eig.sum()
    return eig, ratio


def participation_ratio(eigvals: np.ndarray) -> float:
    denom = float(np.sum(eigvals**2))
    if denom <= 1e-12:
        return float("nan")
    return float((np.sum(eigvals) ** 2) / denom)


def mean_pouch_sigma(sensors: np.ndarray) -> float:
    return float(np.mean(np.std(sensors, axis=0, ddof=1)))


def response_diversity(sensors: np.ndarray) -> float:
    sigmas = np.std(sensors, axis=0, ddof=1)
    return float(np.std(sigmas, ddof=1) / np.mean(sigmas))


def mean_sensitivity_psi_per_deg(sensors: np.ndarray, theta_deg: np.ndarray) -> float:
    X = np.column_stack([theta_deg, np.ones_like(theta_deg)])
    slopes = []
    for col in range(sensors.shape[1]):
        slope, _ = np.linalg.lstsq(X, sensors[:, col], rcond=None)[0]
        slopes.append(abs(float(slope)))
    return float(np.mean(slopes))


def mean_snr_lowpass(
    sensors: np.ndarray, fs_hz: float, cutoff_hz: float = 5.0
) -> float:
    nyquist = 0.5 * fs_hz
    cutoff = min(cutoff_hz, 0.45 * fs_hz)
    b, a = butter(4, cutoff / nyquist, btype="low")
    snr_values = []
    for k in range(sensors.shape[1]):
        x = sensors[:, k]
        x_filt = filtfilt(b, a, x)
        resid = x - x_filt
        var_signal = float(np.var(x_filt))
        var_resid = float(np.var(resid))
        snr_values.append(var_signal / var_resid if var_resid > 1e-12 else np.nan)
    return float(np.nanmean(snr_values))


def mean_dynamic_range(sensors: np.ndarray) -> float:
    return float(np.mean(np.ptp(sensors, axis=0)))


def delay_decoding_memory_capacity(trial: TrialData, K: int = MEMORY_K) -> float:
    scores = []
    for k in range(1, K + 1):
        X = trial.sensors[k:]
        Y = trial.inputs[:-k]
        y_test, y_pred, _ = fit_ridge_predict(X, Y)
        per_channel_r2 = []
        for j in range(Y.shape[1]):
            yt = y_test[:, j]
            yp = y_pred[:, j]
            ss_tot = float(np.sum((yt - np.mean(yt)) ** 2))
            if ss_tot <= 1e-12:
                # Axial trials contain constant command channels; ignore them.
                continue
            ss_res = float(np.sum((yt - yp) ** 2))
            per_channel_r2.append(max(0.0, 1.0 - ss_res / ss_tot))
        scores.append(float(np.mean(per_channel_r2)) if per_channel_r2 else 0.0)
    return float(np.sum(scores))


def cross_correlation_curves(
    sensors: np.ndarray,
    theta_deg: np.ndarray,
    dt: float,
    max_lag_sec: float = MAX_XCORR_LAG_SEC,
) -> Tuple[np.ndarray, np.ndarray]:
    max_lag = int(round(max_lag_sec / dt))
    curves = []
    for i in range(sensors.shape[1]):
        x = sensors[:, i]
        y = theta_deg
        x = (x - np.mean(x)) / np.std(x)
        y = (y - np.mean(y)) / np.std(y)
        corr = correlate(x, y, mode="full") / len(x)
        lags = np.arange(-len(x) + 1, len(x))
        mask = (lags >= -max_lag) & (lags <= max_lag)
        curves.append(corr[mask])
    lag_sec = lags[mask] * dt
    return lag_sec, np.asarray(curves)


def subset_feature_columns(
    subset: Sequence[int], delay: int = DELAY, total_pouches: int = 5
) -> Tuple[int, ...]:
    cols: List[int] = []
    for lag in range(delay):
        base = lag * total_pouches
        cols.extend(base + int(s) for s in subset)
    return tuple(cols)


def build_prediction_cache(trial: TrialData) -> PredictionCache:
    horizon = max(1, int(round(PRED_HORIZON_SEC / trial.dt)))
    X_full = (
        trial.sensor_windows[:-horizon]
        .reshape(len(trial.sensor_windows) - horizon, -1)
        .astype(float)
    )
    y_full = trial.theta_deg[DELAY - 1 + horizon :]
    split = int(0.7 * len(X_full))
    X_train = X_full[:split]
    X_test = X_full[split:]
    y_train = y_full[:split]
    y_test = y_full[split:]

    x_mean = X_train.mean(axis=0)
    x_std = X_train.std(axis=0)
    x_std[x_std < 1e-12] = 1.0

    X_train_z = (X_train - x_mean) / x_std
    X_test_z = (X_test - x_mean) / x_std

    y_mean = float(np.mean(y_train))
    y_train_c = y_train - y_mean

    gram = X_train_z.T @ X_train_z
    rhs = X_train_z.T @ y_train_c

    return PredictionCache(
        cols_all=subset_feature_columns(ALL_POUCHS),
        gram=gram,
        rhs=rhs,
        x_test_z=X_test_z,
        y_test=y_test,
        y_mean=y_mean,
    )


def cached_subset_prediction(
    cache: PredictionCache, subset: Sequence[int], alpha: float = RIDGE_ALPHA
) -> np.ndarray:
    cols = subset_feature_columns(subset)
    A = cache.gram[np.ix_(cols, cols)].copy()
    A.flat[:: A.shape[0] + 1] += alpha
    coef = np.linalg.solve(A, cache.rhs[list(cols)])
    return cache.x_test_z[:, list(cols)] @ coef + cache.y_mean


def quantile_errorbars(values: Sequence[float]) -> Tuple[float, float, float]:
    arr = np.asarray(values, dtype=float)
    med = float(np.median(arr))
    q1 = float(np.quantile(arr, 0.25))
    q3 = float(np.quantile(arr, 0.75))
    return med, med - q1, q3 - med


def save_figure(fig: plt.Figure, out_base: Path, dpi: int) -> None:
    out_base.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_base.with_suffix(".pdf"), bbox_inches="tight")
    fig.savefig(out_base.with_suffix(".png"), bbox_inches="tight", dpi=dpi)


def build_trial_table(trials: Sequence[TrialData]) -> pd.DataFrame:
    rows = []
    for trial in trials:
        pred = prc_prediction(trial, subset=ALL_POUCHS)
        mean_corr, _ = compute_pairwise_corr_stats(trial.sensors)
        eig, ratio = pca_from_covariance(trial.sensors)
        rows.append(
            {
                "trial": trial.meta.label,
                "file": trial.meta.path.name,
                "topology": trial.meta.topology,
                "waveform": trial.meta.waveform,
                "preinflation_psi": trial.meta.preinflation_psi,
                "pmax_psi": trial.meta.pmax_psi,
                "nmse": pred.nmse,
                "rmse_deg": pred.rmse_deg,
                "delay_decoding_index": delay_decoding_memory_capacity(trial),
                "mean_inter_pouch_corr": mean_corr,
                "pc1_variance_pct": float(100.0 * ratio[0]),
                "participation_ratio": participation_ratio(eig),
                "mean_pouch_sigma_psi": mean_pouch_sigma(trial.sensors),
                "response_diversity": response_diversity(trial.sensors),
                "mean_snr": mean_snr_lowpass(trial.sensors, fs_hz=1.0 / trial.dt),
                "mean_dynamic_range_psi": mean_dynamic_range(trial.sensors),
                "mean_sensitivity_psi_per_deg": mean_sensitivity_psi_per_deg(
                    trial.sensors, trial.theta_deg
                ),
                "theta_std_deg": float(np.std(trial.theta_deg, ddof=1)),
            }
        )
    return pd.DataFrame(rows).sort_values(
        ["topology", "waveform", "preinflation_psi", "pmax_psi"]
    )


def make_fig2(
    trials: Sequence[TrialData], metrics: pd.DataFrame, out_dir: Path, dpi: int
) -> None:
    example_parallel = next(
        t
        for t in trials
        if t.meta.topology == "parallel"
        and t.meta.waveform == "axial"
        and t.meta.preinflation_psi == 1
        and t.meta.pmax_psi == 5
    )
    example_coupled = next(
        t
        for t in trials
        if t.meta.topology == "coupled"
        and t.meta.waveform == "axial"
        and t.meta.preinflation_psi == 1
        and t.meta.pmax_psi == 5
    )

    # Parallel: tuned params (0.5 Hz LP, delay=40, alpha=0.0005)
    pred_parallel = prc_prediction_custom(
        example_parallel, delay=40, alpha=0.0005, lp_cutoff_hz=0.5
    )
    # Coupled: 1 Hz LP on sensors, original delay/alpha
    pred_coupled = prc_prediction_custom(
        example_coupled, delay=DELAY, alpha=RIDGE_ALPHA, lp_cutoff_hz=1.0
    )

    fig = plt.figure(figsize=(8.8, 10.5))
    gs = fig.add_gridspec(
        3, 2, height_ratios=[1.0, 0.95, 0.9], hspace=0.65, wspace=0.35
    )

    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, :])
    ax4 = fig.add_subplot(gs[2, :])

    # ── Panel (a): Parallel ──
    mask_p = (pred_parallel.time - pred_parallel.time[0]) <= 20.0
    ax1.plot(
        (pred_parallel.time - pred_parallel.time[0])[mask_p],
        pred_parallel.y_true[mask_p],
        label="True",
        linewidth=1.4,
    )
    ax1.plot(
        (pred_parallel.time - pred_parallel.time[0])[mask_p],
        pred_parallel.y_pred[mask_p],
        label="Pred",
        linewidth=1.4,
    )
    ax1.set_title("(a) Parallel (Axial 1–5 PSI)")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel(r"$\theta$ (deg)")
    ax1.legend(loc="upper right")

    # ── Panel (b): Coupled ──
    mask_c = (pred_coupled.time - pred_coupled.time[0]) <= 20.0
    ax2.plot(
        (pred_coupled.time - pred_coupled.time[0])[mask_c],
        pred_coupled.y_true[mask_c],
        label="True",
        linewidth=1.4,
    )
    ax2.plot(
        (pred_coupled.time - pred_coupled.time[0])[mask_c],
        pred_coupled.y_pred[mask_c],
        label="Pred",
        linewidth=1.4,
    )
    ax2.set_title("(b) Coupled (Axial 1–5 PSI)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel(r"$\theta$ (deg)")
    ax2.legend(loc="upper right")

    # ── Panel (b): NMSE box plot (raw sensors, original params) ──
    box_order = [
        ("axial", "coupled"),
        ("axial", "parallel"),
        ("circular", "coupled"),
        ("circular", "parallel"),
        ("triangular", "coupled"),
        ("triangular", "parallel"),
    ]
    box_positions = [1, 2, 4, 5, 7, 8]
    box_data = [
        metrics.loc[
            (metrics["waveform"] == wf) & (metrics["topology"] == topo),
            "nmse",
        ].to_numpy()
        for wf, topo in box_order
    ]
    ax3.boxplot(box_data, positions=box_positions, widths=0.6)
    ax3.set_xlim(0.5, 8.5)
    ax3.set_xticks(box_positions)
    ax3.set_xticklabels(
        [
            "Axial\nCoupled",
            "Axial\nParallel",
            "Circular\nCoupled",
            "Circular\nParallel",
            "Triangular\nCoupled",
            "Triangular\nParallel",
        ]
    )
    ax3.set_ylabel("NMSE")
    ax3.set_title("(b) 1 s-ahead prediction NMSE by waveform and topology")

    # ── Panel (c): DDI (raw sensors, original params) ──
    mc_data = [
        metrics.loc[metrics["topology"] == topo, "delay_decoding_index"].to_numpy()
        for topo in ["coupled", "parallel"]
    ]
    ax4.boxplot(mc_data, tick_labels=["Coupled", "Parallel"])
    ax4.set_xlabel("Topology")
    ax4.set_ylabel("Delay-decoding index (DDI)")
    ax4.set_title("(c) Delay-decoding index (K=40)")

    save_figure(fig, out_dir / "fig2_performance", dpi)
    plt.close(fig)


def make_fig3(trials: Sequence[TrialData], out_dir: Path, dpi: int) -> None:
    coupled = next(
        t
        for t in trials
        if t.meta.topology == "coupled"
        and t.meta.waveform == "circular"
        and t.meta.preinflation_psi == 3
        and t.meta.pmax_psi == 10
    )
    parallel = next(
        t
        for t in trials
        if t.meta.topology == "parallel"
        and t.meta.waveform == "circular"
        and t.meta.preinflation_psi == 3
        and t.meta.pmax_psi == 10
    )

    coupled_corr_mean, coupled_corr = compute_pairwise_corr_stats(coupled.sensors)
    parallel_corr_mean, parallel_corr = compute_pairwise_corr_stats(parallel.sensors)
    _ = coupled_corr_mean, parallel_corr_mean
    coupled_eig, coupled_ratio = pca_from_covariance(coupled.sensors)
    parallel_eig, parallel_ratio = pca_from_covariance(parallel.sensors)
    coupled_lag, coupled_xcorr = cross_correlation_curves(
        coupled.sensors, coupled.theta_deg, coupled.dt
    )
    parallel_lag, parallel_xcorr = cross_correlation_curves(
        parallel.sensors, parallel.theta_deg, parallel.dt
    )

    fig = plt.figure(figsize=(10.0, 9.0))
    gs = fig.add_gridspec(2, 2, hspace=0.4, wspace=0.22)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 0])
    ax4 = fig.add_subplot(gs[1, 1])

    im1 = ax1.imshow(coupled_corr, vmin=0.90, vmax=1.00, cmap="viridis")
    ax1.set_title("(a) Coupled inter-pouch correlation")
    ax1.set_xticks(range(5), labels=[1, 2, 3, 4, 5])
    ax1.set_yticks(range(5), labels=[1, 2, 3, 4, 5])
    fig.colorbar(im1, ax=ax1, fraction=0.046, pad=0.03)

    im2 = ax2.imshow(parallel_corr, vmin=0.90, vmax=1.00, cmap="viridis")
    ax2.set_title("(b) Parallel inter-pouch correlation")
    ax2.set_xticks(range(5), labels=[1, 2, 3, 4, 5])
    ax2.set_yticks(range(5), labels=[1, 2, 3, 4, 5])
    fig.colorbar(im2, ax=ax2, fraction=0.046, pad=0.03)

    pcs = np.arange(1, 6)
    ax3.plot(pcs, np.cumsum(coupled_ratio), label="Coupled")
    ax3.plot(pcs, np.cumsum(parallel_ratio), label="Parallel")
    ax3.set_ylim(0.97, 1.0005)
    ax3.set_xlabel("Number of PCs")
    ax3.set_ylabel("Cumulative variance")
    ax3.set_title("(c) PCA cumulative variance")
    ax3.legend(loc="lower right")

    # Match the bundled figure style: same color per pouch, dashed=parallel, solid=coupled.
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for i in range(5):
        ax4.plot(
            coupled_lag, coupled_xcorr[i], color=colors[i % len(colors)], linewidth=1.4
        )
        ax4.plot(
            parallel_lag,
            parallel_xcorr[i],
            color=colors[i % len(colors)],
            linewidth=1.4,
            linestyle="--",
        )

    # Add legend to distinguish line styles
    ax4.plot([], [], color="black", linestyle="-", linewidth=1.4, label="Coupled")
    ax4.plot([], [], color="black", linestyle="--", linewidth=1.4, label="Parallel")
    ax4.legend(loc="upper right")

    ax4.set_title(r"(d) Cross-correlation pouch vs $\theta$")
    ax4.set_xlabel("Lag (s)    (positive: pouch leads)")
    ax4.set_ylabel("Corr.")

    save_figure(fig, out_dir / "fig3_diversity", dpi)
    plt.close(fig)


def make_fig4(
    metrics: pd.DataFrame, out_dir: Path, dpi: int, panel4b_mode: str
) -> None:
    fig = plt.figure(figsize=(8.8, 7.4))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.2, 1.0], hspace=0.55, wspace=0.38)
    ax1 = fig.add_subplot(gs[:, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[1, 1])

    for topo, marker in [("coupled", "o"), ("parallel", "s")]:
        sub = metrics.loc[metrics["topology"] == topo]
        ax1.scatter(
            sub["theta_std_deg"], sub["nmse"], label=topo.capitalize(), marker=marker
        )
    ax1.set_title("(a) Operating regime")
    ax1.set_xlabel(r"$\theta_{\mathrm{std}}$ (deg)")
    ax1.set_ylabel("NMSE")
    ax1.legend(loc="upper right")

    y_col = (
        "mean_pouch_sigma_psi" if panel4b_mode == "sigma" else "mean_inter_pouch_corr"
    )
    if panel4b_mode == "sigma":
        ax2.set_title("(b) Pre-inflation scales signal amplitude")
        ax2.set_ylabel(r"Mean pouch $\sigma$ (PSI)")
    else:
        ax2.set_title("(b) Pre-inflation increases redundancy")
        ax2.set_ylabel("Mean inter-pouch corr.")

    for topo in ["coupled", "parallel"]:
        xs, ys, yerr_lo, yerr_hi = [], [], [], []
        for pre in [1, 2, 3]:
            vals = metrics.loc[
                (metrics["topology"] == topo) & (metrics["preinflation_psi"] == pre),
                y_col,
            ].to_numpy()
            med, elo, ehi = quantile_errorbars(vals)
            xs.append(pre)
            ys.append(med)
            yerr_lo.append(elo)
            yerr_hi.append(ehi)
        ax2.errorbar(
            xs, ys, yerr=[yerr_lo, yerr_hi], marker="o", label=topo.capitalize()
        )
    ax2.set_xlabel("Pre-inflation (PSI)")
    ax2.legend(loc="upper left")

    for topo in ["coupled", "parallel"]:
        xs, ys, yerr_lo, yerr_hi = [], [], [], []
        for pre in [1, 2, 3]:
            vals = metrics.loc[
                (metrics["topology"] == topo) & (metrics["preinflation_psi"] == pre),
                "nmse",
            ].to_numpy()
            med, elo, ehi = quantile_errorbars(vals)
            xs.append(pre)
            ys.append(med)
            yerr_lo.append(elo)
            yerr_hi.append(ehi)
        ax3.errorbar(
            xs, ys, yerr=[yerr_lo, yerr_hi], marker="o", label=topo.capitalize()
        )
    ax3.set_title("(c) Pre-inflation vs PRC accuracy")
    ax3.set_xlabel("Pre-inflation (PSI)")
    ax3.set_ylabel("NMSE")
    ax3.legend(loc="upper left")

    save_figure(fig, out_dir / "fig4_regime", dpi)
    plt.close(fig)


def make_fig5(
    trials: Sequence[TrialData], out_dir: Path, dpi: int
) -> Dict[str, object]:
    parallel_trials = [t for t in trials if t.meta.topology == "parallel"]
    all_subsets = {m: list(itertools.combinations(range(5), m)) for m in range(1, 6)}

    caches = {
        trial.meta.label: build_prediction_cache(trial) for trial in parallel_trials
    }

    subset_metrics: Dict[Tuple[int, Tuple[int, ...]], List[float]] = {}
    per_trial_subset_nmse: Dict[Tuple[str, Tuple[int, ...]], float] = {}

    for m, subsets in all_subsets.items():
        for subset in subsets:
            vals = []
            for trial in parallel_trials:
                cache = caches[trial.meta.label]
                y_pred = cached_subset_prediction(cache, subset)
                score = nmse(cache.y_test, y_pred)
                vals.append(score)
                per_trial_subset_nmse[(trial.meta.label, subset)] = score
            subset_metrics[(m, subset)] = vals

    m_values = [1, 2, 3, 4, 5]
    oracle_medians = []
    oracle_q1 = []
    oracle_q3 = []
    fixed_medians = []
    best_fixed_subsets: Dict[int, Tuple[int, ...]] = {}

    for m in m_values:
        subsets = all_subsets[m]
        subset_medians = {
            subset: float(np.median(subset_metrics[(m, subset)])) for subset in subsets
        }
        best_subset = min(subset_medians, key=subset_medians.get)
        best_fixed_subsets[m] = best_subset
        fixed_medians.append(subset_medians[best_subset])

        per_trial_best = []
        for trial in parallel_trials:
            per_trial_best.append(
                min(
                    per_trial_subset_nmse[(trial.meta.label, subset)]
                    for subset in subsets
                )
            )
        oracle_medians.append(float(np.median(per_trial_best)))
        oracle_q1.append(float(np.quantile(per_trial_best, 0.25)))
        oracle_q3.append(float(np.quantile(per_trial_best, 0.75)))

    # Leave-one-out importance relative to all five pouches.
    full_subset = tuple(range(5))
    full_nmse = {
        trial.meta.label: per_trial_subset_nmse[(trial.meta.label, full_subset)]
        for trial in parallel_trials
    }
    leave_one_out_delta = []
    for removed in range(5):
        subset = tuple(i for i in range(5) if i != removed)
        deltas = [
            per_trial_subset_nmse[(trial.meta.label, subset)]
            - full_nmse[trial.meta.label]
            for trial in parallel_trials
        ]
        leave_one_out_delta.append(float(np.median(deltas)))

    fig = plt.figure(figsize=(8.8, 4.2))
    gs = fig.add_gridspec(1, 2, wspace=0.32)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])

    ax1.plot(m_values, oracle_medians, marker="o", label="Oracle best (per trial)")
    ax1.fill_between(m_values, oracle_q1, oracle_q3, alpha=0.18)
    ax1.plot(m_values, fixed_medians, marker="s", label="Best fixed subset")
    ax1.set_title("(a) Sensor-count trade-off (parallel)")
    ax1.set_xlabel("Number of pouches used")
    ax1.set_ylabel("NMSE (median across trials)")
    ax1.legend(loc="upper right")

    ax2.bar(np.arange(1, 6), leave_one_out_delta)
    ax2.set_title("(b) Leave-one-out importance")
    ax2.set_xlabel("Pouch removed (leave-one-out)")
    ax2.set_ylabel(r"$\Delta$NMSE vs all 5")

    save_figure(fig, out_dir / "fig5_ablation", dpi)
    plt.close(fig)

    return {
        "best_fixed_subsets_1_indexed": {
            str(m): [i + 1 for i in subset] for m, subset in best_fixed_subsets.items()
        },
        "oracle_median_nmse": {
            str(m): float(v) for m, v in zip(m_values, oracle_medians)
        },
        "best_fixed_median_nmse": {
            str(m): float(v) for m, v in zip(m_values, fixed_medians)
        },
        "leave_one_out_delta_nmse_median": {
            str(i + 1): float(v) for i, v in enumerate(leave_one_out_delta)
        },
    }


def write_summary_files(
    metrics: pd.DataFrame, fig5_summary: Dict[str, object], out_dir: Path
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    metrics.to_csv(out_dir / "trial_metrics.csv", index=False)

    topology_summary = pd.DataFrame(
        {
            "median_nmse": metrics.groupby("topology")["nmse"].median(),
            "mean_delay_decoding_index": metrics.groupby("topology")[
                "delay_decoding_index"
            ].mean(),
            "std_delay_decoding_index": metrics.groupby("topology")[
                "delay_decoding_index"
            ].std(),
            "mean_inter_pouch_corr": metrics.groupby("topology")[
                "mean_inter_pouch_corr"
            ].mean(),
            "mean_sensitivity_psi_per_deg": metrics.groupby("topology")[
                "mean_sensitivity_psi_per_deg"
            ].mean(),
        }
    ).reset_index()
    topology_summary.to_csv(out_dir / "topology_summary.csv", index=False)

    with open(out_dir / "fig5_subset_summary.json", "w", encoding="utf-8") as f:
        json.dump(fig5_summary, f, indent=2)


def load_all_trials(data_root: Path) -> List[TrialData]:
    trials: List[TrialData] = []
    for topo in ["parallel", "coupled"]:
        for path in sorted((data_root / topo).glob("*.csv")):
            meta = parse_trial_name(path)
            trials.append(load_trial(meta))
    return trials


def main() -> None:
    args = parse_args()
    data_root, tmpdir = maybe_extract_data(args.data_source)

    try:
        out_dir = args.out_dir
        out_dir.mkdir(parents=True, exist_ok=True)

        trials = load_all_trials(data_root)
        metrics = build_trial_table(trials)
        metrics.to_csv(out_dir / "trial_metrics.csv", index=False)

        make_fig2(trials, metrics, out_dir, args.dpi)
        make_fig3(trials, out_dir, args.dpi)
        make_fig4(metrics, out_dir, args.dpi, args.panel4b_mode)
        fig5_summary = make_fig5(trials, out_dir, args.dpi)
        write_summary_files(metrics, fig5_summary, out_dir)

        print(f"Wrote regenerated figures and summaries to: {out_dir}")
    finally:
        if tmpdir is not None:
            tmpdir.cleanup()


if __name__ == "__main__":
    main()
