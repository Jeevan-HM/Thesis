"""
open_loop_controller_plot.py
────────────────────────────
9-panel PRC Inverse Controller validation figure.
Axial 1-10, Circular 1-10, Triangular 1-10 (parallel topology).

Key fix vs naive version:
  The OptiTrack runs at ~48 Hz but the logger runs at 100 Hz, so every
  other row of mocap data is a frozen duplicate — creating a staircase
  artifact.  This script uses mocap_time_rel_s to extract unique mocap
  frames, applies SLERP quaternion interpolation for bending angle and
  linear interpolation for tip position, giving smooth ground-truth
  targets at the full 100 Hz logger rate.

Usage:
    python open_loop_controller_plot.py

Outputs saved to ./temp/
    prc_inverse_axial_3-10.png
    prc_inverse_circular_3-10.png
    prc_inverse_triangular_3-10.png

Requirements:
    pip install numpy pandas scipy matplotlib scikit-learn
"""

import os

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation, Slerp
from sklearn.linear_model import Ridge

os.makedirs("temp", exist_ok=True)

# ── column names ───────────────────────────────────────────────────────────
POUCH = [f"Measured_pressure_Segment_1_pouch_{i}" for i in range(1, 6)]
INPUT = [f"Desired_pressure_segment_{i}" for i in range(2, 5)]
RB1Q = ["Rigid_body_1_qx", "Rigid_body_1_qy", "Rigid_body_1_qz", "Rigid_body_1_qw"]
RB3Q = ["Rigid_body_3_qx", "Rigid_body_3_qy", "Rigid_body_3_qz", "Rigid_body_3_qw"]
RB3P = ["Rigid_body_3_x", "Rigid_body_3_y", "Rigid_body_3_z"]

# ── hyper-parameters (Config D — deployed) ─────────────────────────────────
FS = 100
TRIM_SEC = 10
TDL = 30
TH_HIST = 10
LP_HZ = 1.0
ALPHA = 1.0
TRAIN_FRAC = 0.70
PLOT_SEC = 50

# ── data files ─────────────────────────────────────────────────────────────
DATA = {
    ("axial", 3, 10): "data/_data_extract/parallel/axial_3-10_parallel.csv",
    ("circular", 3, 10): "data/_data_extract/parallel/circular_3-10_parallel.csv",
    ("triangular", 3, 10): "data/_data_extract/parallel/triangular_3-10_parallel.csv",
}


# ══════════════════════════════════════════════════════════════════════════
# signal helpers
# ══════════════════════════════════════════════════════════════════════════


def lp(X, hz=LP_HZ, fs=FS, order=4):
    """Zero-phase low-pass filter, column-wise."""
    b, a = butter(order, hz / (fs / 2), btype="low")
    fn = lambda x: filtfilt(b, a, x)
    return (
        fn(X)
        if X.ndim == 1
        else np.column_stack([fn(X[:, j]) for j in range(X.shape[1])])
    )


def safe_nmse(y, yh):
    d = ((y - y.mean()) ** 2).mean()
    return np.nan if d < 1e-12 else ((y - yh) ** 2).mean() / d


def tdl_matrix(X, n):
    """(T, D) → (T-n+1, D*n), lag-0 first."""
    T, D = X.shape
    return np.hstack([X[n - 1 - k : T - k if k else T] for k in range(n)])


# ══════════════════════════════════════════════════════════════════════════
# mocap interpolation  — THE KEY FIX
# ══════════════════════════════════════════════════════════════════════════


def interpolate_mocap(df, t_logger):
    """
    The OptiTrack (~48 Hz) and logger (100 Hz) are unsynchronised.
    Each row has a mocap_time_rel_s stamp; duplicate values = frozen frames.

    This function:
      1. Extracts unique mocap frames using mocap_time_rel_s.
      2. SLERP-interpolates quaternions for RB1 and RB3.
      3. Linearly interpolates RB3 tip position.
    Returns tip_mm (N, 3), theta_deg (N,), valid_mask (T,)
    for rows of t_logger that fall within the mocap time range.
    """
    t_moc = df["mocap_time_rel_s"].values

    # unique mocap frames (first occurrence of each timestamp)
    _, uid = np.unique(t_moc, return_index=True)
    tu = t_moc[uid]

    # SLERP for quaternions
    r1u = Rotation.from_quat(df[RB1Q].values[uid])
    r3u = Rotation.from_quat(df[RB3Q].values[uid])

    # valid logger timestamps within mocap range
    valid = (t_logger >= tu[0]) & (t_logger <= tu[-1])
    tq = t_logger[valid]

    r1i = Slerp(tu, r1u)(tq)
    r3i = Slerp(tu, r3u)(tq)
    theta = np.degrees((r1i.inv() * r3i).magnitude())

    # linear interp for tip position
    tip_u = df[RB3P].values[uid]
    tip_mm = np.column_stack([np.interp(tq, tu, tip_u[:, j]) for j in range(3)]) * 1000

    return tip_mm, theta, valid


# ══════════════════════════════════════════════════════════════════════════
# feature builders
# ══════════════════════════════════════════════════════════════════════════


def inverse_features(S_lp, theta):
    """Returns X (N, F), offset into original arrays."""
    dS = np.gradient(S_lp, axis=0)
    off_tdl = TDL - 1

    X_S = tdl_matrix(S_lp, TDL)
    X_dS = tdl_matrix(dS, TDL)
    S0 = S_lp[off_tdl:]
    poly = np.hstack(
        [
            (S0[:, i] * S0[:, j]).reshape(-1, 1)
            for i in range(5)
            for j in range(i + 1, 5)
        ]
    )
    th = theta[off_tdl:]
    sc = np.column_stack(
        [
            np.sin(np.radians(th)),
            np.cos(np.radians(th)),
            np.sin(np.radians(2 * th)),
            np.cos(np.radians(2 * th)),
        ]
    )
    X_th = tdl_matrix(th.reshape(-1, 1), TH_HIST)
    ex = TH_HIST - 1

    X = np.hstack([X_S[ex:], X_dS[ex:], poly[ex:], sc[ex:], X_th])
    return X, off_tdl + ex


def forward_features(S_lp, u, offset):
    """Returns X_fwd (N, F) aligned to the given offset."""
    dS = np.gradient(S_lp, axis=0)
    ex = offset - (TDL - 1)
    X_S = tdl_matrix(S_lp, TDL)[ex:]
    X_dS = tdl_matrix(dS, TDL)[ex:]
    u_a = u[offset:]
    N = min(len(X_S), len(X_dS), len(u_a))
    return np.hstack([X_S[:N], X_dS[:N], u_a[:N]])


# ══════════════════════════════════════════════════════════════════════════
# pipeline
# ══════════════════════════════════════════════════════════════════════════


def run(csv_path, waveform, p0, pmax):
    print(f"\n── {waveform.capitalize()}  P₀={p0}  Pmax={pmax} ──")

    df = pd.read_csv(csv_path).sort_values("time").reset_index(drop=True)
    df[POUCH + INPUT] = df[POUCH + INPUT].ffill().bfill()
    df = df.dropna(subset=RB1Q + RB3Q + RB3P + ["mocap_time_rel_s"]).reset_index(
        drop=True
    )

    t = df["time"].values
    df = df[(t >= t[0] + TRIM_SEC) & (t <= t[-1] - TRIM_SEC)].reset_index(drop=True)
    t = df["time"].values

    # ── smooth mocap via SLERP + linear interp ─────────────────────────────
    tip_mm, theta, valid = interpolate_mocap(df, t)

    # trim all arrays to valid region
    S_lp = lp(df[POUCH].values[valid])
    u_true = df[INPUT].values[valid]
    t = t[valid]
    T = len(t)
    print(f"  Samples after interp: {T}")

    # ── inverse model ──────────────────────────────────────────────────────
    Xi, off = inverse_features(S_lp, theta)
    N = Xi.shape[0]
    u_tgt = u_true[off : off + N]
    t_inv = t[off : off + N]

    sp = int(N * TRAIN_FRAC)
    inv_m = Ridge(alpha=ALPHA).fit(Xi[:sp], u_tgt[:sp])
    u_pred = inv_m.predict(Xi)

    inv_nm = [safe_nmse(u_tgt[sp:, j], u_pred[sp:, j]) for j in range(3)]
    print(
        f"  Inverse NMSE  seg2={inv_nm[0]:.3f}  seg3={inv_nm[1]:.3f}  seg4={inv_nm[2]:.3f}"
    )

    # pad u_pred to full length for forward_features
    u_pred_full = np.vstack([u_true[:off], u_pred])

    # ── forward model ──────────────────────────────────────────────────────
    Xf_tr = forward_features(S_lp, u_true, off)
    Xf_pr = forward_features(S_lp, u_pred_full, off)
    Nf = min(Xf_tr.shape[0], Xf_pr.shape[0], T - off)

    Xf_tr = Xf_tr[:Nf]
    Xf_pr = Xf_pr[:Nf]
    tip_f = tip_mm[off : off + Nf]
    t_f = t[off : off + Nf]

    spf = int(Nf * TRAIN_FRAC)
    fwd_m = Ridge(alpha=ALPHA).fit(Xf_tr[:spf], tip_f[:spf])

    tip_true = fwd_m.predict(Xf_tr)
    tip_pred = fwd_m.predict(Xf_pr)

    rmse_tr = np.sqrt(np.mean((tip_f[spf:] - tip_true[spf:]) ** 2))
    rmse_pr = np.sqrt(np.mean((tip_f[spf:] - tip_pred[spf:]) ** 2))
    print(f"  3D RMSE  true u → {rmse_tr:.2f} mm   pred û → {rmse_pr:.2f} mm")

    # ── slice last PLOT_SEC seconds of test window ─────────────────────────
    n_pl = int(PLOT_SEC * FS)

    def tail(a):
        return a[spf:][-n_pl:]

    n_inv = min(n_pl, N - sp)
    u_t_t = u_tgt[sp:][-n_inv:]
    u_p_t = u_pred[sp:][-n_inv:]
    t_it = t_inv[sp:][-n_inv:] - t_inv[sp:][-n_inv:][0]

    return dict(
        waveform=waveform,
        p0=p0,
        pmax=pmax,
        gt_full=tip_f[spf:],
        true_full=tip_true[spf:],
        pred_full=tip_pred[spf:],
        gt=tail(tip_f),
        true=tail(tip_true),
        pred=tail(tip_pred),
        t=tail(t_f) - tail(t_f)[0],
        u_tgt=u_t_t,
        u_pred=u_p_t,
        t_inv=t_it,
        rmse_true=rmse_tr,
        rmse_pred=rmse_pr,
        inv_nmse=inv_nm,
    )


# ══════════════════════════════════════════════════════════════════════════
# figure  — same layout as your existing plots
# ══════════════════════════════════════════════════════════════════════════

BLACK = "#000000"
BLUE = "#1f77b4"
RED_D = "#d62728"
SEG_C = ["#1f77b4", "#ff7f0e", "#2ca02c"]


def make_figure(r):
    wv = r["waveform"].capitalize()
    fig = plt.figure(figsize=(14, 10), facecolor="white")
    fig.suptitle(
        f"PRC Inverse Controller (sin/cos encoding) — {wv}\n"
        f"P₀={r['p0']} PSI, pmax={r['pmax']} PSI  |  "
        f"3D RMSE: true u → {r['rmse_true']:.1f} mm, pred û → {r['rmse_pred']:.1f} mm",
        fontsize=11,
        y=0.998,
    )

    lkw = dict(fontsize=8, loc="upper right", framealpha=0.8)
    gt = r["gt_full"]
    tr = r["true_full"]
    pr = r["pred_full"]

    # row 1: 3D + XY + XZ
    ax3 = fig.add_subplot(3, 3, 1, projection="3d")
    ax3.plot(gt[:, 0], gt[:, 2], gt[:, 1], color=BLACK, lw=1.6, label="Ground truth")
    ax3.plot(
        tr[:, 0], tr[:, 2], tr[:, 1], color=BLUE, lw=0.8, alpha=0.7, label="Fwd(true u)"
    )
    ax3.plot(
        pr[:, 0],
        pr[:, 2],
        pr[:, 1],
        color=RED_D,
        lw=0.8,
        alpha=0.7,
        ls="--",
        label="Fwd(pred û)",
    )
    ax3.set_xlabel("X (mm)", fontsize=7, labelpad=1)
    ax3.set_ylabel("Z (mm)", fontsize=7, labelpad=1)
    ax3.set_zlabel("Y (mm)", fontsize=7, labelpad=1)
    ax3.tick_params(labelsize=6)
    ax3.set_title("(a) 3D tip trajectory", fontsize=9)
    ax3.legend(**lkw)

    for idx, (xi, yi, lbl) in enumerate(
        [(0, 1, "(b) XY projection"), (0, 2, "(c) XZ projection")]
    ):
        ax = fig.add_subplot(3, 3, 2 + idx)
        ax.plot(gt[:, xi], gt[:, yi], color=BLACK, lw=1.6, label="Ground truth")
        ax.plot(
            tr[:, xi], tr[:, yi], color=BLUE, lw=0.8, alpha=0.7, label="Fwd(true u)"
        )
        ax.plot(
            pr[:, xi],
            pr[:, yi],
            color=RED_D,
            lw=0.8,
            alpha=0.7,
            ls="--",
            label="Fwd(pred û)",
        )
        ax.set_xlabel("X (mm)", fontsize=9)
        ax.set_ylabel("Y (mm)" if yi == 1 else "Z (mm)", fontsize=9)
        ax.set_title(lbl, fontsize=9)
        ax.legend(**lkw)

    # row 2: tip X Y Z time series
    for col, (title, yl, dim) in enumerate(
        [
            ("(d) Tip X(t)", "X (mm)", 0),
            ("(e) Tip Y(t)", "Y (mm)", 1),
            ("(f) Tip Z(t)", "Z (mm)", 2),
        ]
    ):
        ax = fig.add_subplot(3, 3, 4 + col)
        ax.plot(r["t"], r["gt"][:, dim], color=BLACK, lw=1.4, label="Ground truth")
        ax.plot(
            r["t"],
            r["true"][:, dim],
            color=BLUE,
            lw=0.9,
            alpha=0.8,
            label="Fwd(true u)",
        )
        ax.plot(
            r["t"],
            r["pred"][:, dim],
            color=RED_D,
            lw=0.9,
            alpha=0.8,
            ls="--",
            label="Fwd(pred û)",
        )
        ax.set_xlabel("Time (s)", fontsize=9)
        ax.set_ylabel(yl, fontsize=9)
        ax.set_title(title, fontsize=9)
        ax.legend(**lkw)

    # row 3: segment command pressures
    for col, (title, sc) in enumerate(
        zip(["(g) Segment 2", "(h) Segment 3", "(i) Segment 4"], SEG_C)
    ):
        ax = fig.add_subplot(3, 3, 7 + col)
        ax.plot(r["t_inv"], r["u_tgt"][:, col], color=BLACK, lw=1.4, label="True u(t)")
        ax.plot(
            r["t_inv"],
            r["u_pred"][:, col],
            color=sc,
            lw=1.0,
            ls="--",
            label=f"Pred û  (NMSE={r['inv_nmse'][col]:.3f})",
        )
        ax.set_xlabel("Time (s)", fontsize=9)
        ax.set_ylabel("PSI", fontsize=9)
        ax.set_title(title, fontsize=9)
        ax.legend(fontsize=7.5, loc="upper right", framealpha=0.8)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out = f"temp/prc_inverse_{wv.lower()}_{r['p0']}-{r['pmax']}.png"
    plt.savefig(out, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"  Saved → {out}")


if __name__ == "__main__":
    for (waveform, p0, pmax), path in DATA.items():
        make_figure(run(path, waveform, p0, pmax))
    print("\nDone.")
