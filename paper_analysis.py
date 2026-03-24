import argparse
import glob
import os
import re
import warnings

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.backends.backend_pdf import PdfPages
from scipy.signal import butter, filtfilt
from sklearn.decomposition import PCA
from sklearn.linear_model import Ridge
from sklearn.metrics import r2_score
from sklearn.preprocessing import StandardScaler

warnings.filterwarnings('ignore')

def build_tapped_delay(X, n_tau=20):
    N, d = X.shape
    M = N - n_tau + 1
    if M <= 0:
        raise ValueError("Not enough samples")
    out = np.zeros((M, d * n_tau))
    for k in range(n_tau):
        out[:, k * d : (k + 1) * d] = X[n_tau - 1 - k : n_tau - 1 - k + M]
    return out

def nmse(y_true, y_pred):
    denom = np.sum((y_true - np.mean(y_true)) ** 2)
    if denom < 1e-12:
        return np.nan
    return float(np.sum((y_true - y_pred) ** 2) / denom)

def ddi_score(S, U, max_delay=40):
    # DDI up to K=40
    # train on S(t) to predict U(t-k) for k=1..max_delay
    N = len(S)
    frac = 0.7
    ntr = int(frac * N)
    
    total_ddi = 0.0
    for k in range(1, max_delay + 1):
        if N - k <= 0:
            break
        S_k = S[k:]
        U_k = U[:-k]
        
        S_tr, S_te = S_k[:ntr], S_k[ntr:]
        U_tr, U_te = U_k[:ntr], U_k[ntr:]
        
        scaler = StandardScaler().fit(S_tr)
        S_tr_s = scaler.transform(S_tr)
        S_te_s = scaler.transform(S_te)
        
        mdl = Ridge(alpha=0.01)
        mdl.fit(S_tr_s, U_tr)
        U_pred = mdl.predict(S_te_s)
        
        # mean R^2 across U dimensions
        r2s = []
        for i in range(U.shape[1]):
            v = np.var(U_te[:, i])
            if v > 1e-6:
                r2 = r2_score(U_te[:, i], U_pred[:, i])
                r2s.append(r2)
        if r2s:
            mean_r2 = np.mean(r2s)
            total_ddi += max(0, mean_r2)
    return total_ddi

def lowpass_filter(data, fs=100.0, cutoff=5.0):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(2, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data, axis=0)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_root", type=str, default="data/_data_extract")
    parser.add_argument("--out_dir", type=str, default="figures")
    args = parser.parse_args()
    
    os.makedirs(args.out_dir, exist_ok=True)
    
    csv_files = glob.glob(os.path.join(args.data_root, "**", "*.csv"), recursive=True)
    
    results = []
    
    for p in csv_files:
        name = os.path.basename(p).lower()
        if 'axial' in name: traj = 'axial'
        elif 'circular' in name: traj = 'circular'
        elif 'triang' in name: traj = 'triangular'
        else: traj = 'unknown'
        
        if 'coupled' in name: config = 'coupled'
        elif 'parallel' in name: config = 'parallel'
        else: config = 'unknown'
        
        # parse pre-inflation from name (1-5 means 1 PSI pre, 5 PSI max usually)
        m = re.search(r"(\d+)-(\d+)", name)
        pre_inf = float(m.group(1)) if m else np.nan
        max_p = float(m.group(2)) if m else np.nan
        
        meta = {
            "file": os.path.basename(p),
            "trajectory": traj,
            "config": config,
            "pre_inflation": pre_inf,
            "max_pressure": max_p
        }
        
        try:
            df = pd.read_csv(p)
            
            pouch_cols = [c for c in df.columns if "Measured_pressure_Segment_1_pouch" in c][:5]
            if len(pouch_cols) < 5: continue
            
            u_cols = [c for c in df.columns if "Desired_pressure_segment" in c and c != "Desired_pressure_segment_1"]
            if len(u_cols) < 3: continue
            
            if "theta" not in df.columns:
                qx, qy, qz, qw = df["Rigid_body_1_qx"], df["Rigid_body_1_qy"], df["Rigid_body_1_qz"], df["Rigid_body_1_qw"]
                df["theta"] = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0)) * 180 / np.pi
                
            df = df.dropna(subset=pouch_cols + u_cols + ["theta"])
            
            # discard first and last 10s (approx 1000 samples @ 100Hz)
            if len(df) < 2500: continue
            df = df.iloc[1000:-1000].reset_index(drop=True)
            
            S = df[pouch_cols].to_numpy(float)
            U = df[u_cols].to_numpy(float)
            theta = df["theta"].to_numpy(float)
            
            # Diversity metrics
            corr_matrix = np.corrcoef(S.T)
            # upper triangle indices
            i_upper = np.triu_indices(5, k=1)
            mean_r = np.mean(corr_matrix[i_upper])
            unshared_var = 1 - mean_r**2
            
            pca = PCA()
            pca.fit(S)
            pc1_var = pca.explained_variance_ratio_[0] * 100
            lambdas = pca.explained_variance_
            participation_ratio = (np.sum(lambdas)**2) / np.sum(lambdas**2)
            
            S_stds = np.std(S, axis=0)
            response_div = np.std(S_stds) / np.mean(S_stds) if np.mean(S_stds) > 0 else 0
            
            dyn_range = np.mean(np.max(S, axis=0) - np.min(S, axis=0))
            
            S_filt = lowpass_filter(S)
            noise_var = np.var(S - S_filt, axis=0)
            signal_var = np.var(S_filt, axis=0)
            snrs = 10 * np.log10(signal_var / np.clip(noise_var, 1e-12, None))
            mean_snr = np.mean(snrs)
            
            theta_std = np.std(theta)
            
            # Sensitivity (linear fit from theta to each pouch)
            sens = []
            for i in range(5):
                if theta_std > 1e-6:
                    cov = np.cov(theta, S[:, i])[0, 1]
                    sens.append(abs(cov / (theta_std**2)))
            mean_sens = np.mean(sens) if sens else 0
            
            # Prediction Task (1s ahead = 100 steps)
            delta = 100
            n_tau = 20
            
            # tapped delay for prediction
            # we want to predict theta(t+100) using S(t), S(t-1)... S(t-19)
            if len(S) > n_tau + delta:
                X = build_tapped_delay(S, n_tau=n_tau)
                # align targets: X[0] corresponds to S[19]. We want theta[19 + 100]
                y = theta[n_tau - 1 + delta:]
                X = X[:len(y)]  # truncate X
                
                N = len(y)
                ntr = int(0.7 * N)
                
                # S-only prediction
                scaler = StandardScaler().fit(X[:ntr])
                mdl = Ridge(alpha=0.01)
                mdl.fit(scaler.transform(X[:ntr]), y[:ntr])
                yp_S = mdl.predict(scaler.transform(X[ntr:]))
                nmse_val = nmse(y[ntr:], yp_S)
                rmse_val = np.sqrt(np.mean((y[ntr:] - yp_S)**2))
                
                # Context (U) prediction
                X_U = build_tapped_delay(U, n_tau=1) # just current U
                X_U = X_U[:len(y)]
                scaler_U = StandardScaler().fit(X_U[:ntr])
                mdl_U = Ridge(alpha=0.01)
                mdl_U.fit(scaler_U.transform(X_U[:ntr]), y[:ntr])
                yp_U = mdl_U.predict(scaler_U.transform(X_U[ntr:]))
                nmse_U = nmse(y[ntr:], yp_U)
                
                # Context + S prediction
                X_US = np.hstack((X, X_U))
                scaler_US = StandardScaler().fit(X_US[:ntr])
                mdl_US = Ridge(alpha=0.01)
                mdl_US.fit(scaler_US.transform(X_US[:ntr]), y[:ntr])
                yp_US = mdl_US.predict(scaler_US.transform(X_US[ntr:]))
                nmse_US = nmse(y[ntr:], yp_US)
                
            else:
                nmse_val, rmse_val, nmse_U, nmse_US = np.nan, np.nan, np.nan, np.nan
                
            # DDI
            ddi_val = ddi_score(S, U, max_delay=40)
            
            # Pouch importance (leave one out) - parallel only
            importances = []
            if config == 'parallel':
                for i in range(5):
                    cols_idx = [j for j in range(5) if j != i]
                    X_abl = build_tapped_delay(S[:, cols_idx], n_tau=n_tau)
                    X_abl = X_abl[:len(y)]
                    scr = StandardScaler().fit(X_abl[:ntr])
                    mdl = Ridge(alpha=0.01)
                    mdl.fit(scr.transform(X_abl[:ntr]), y[:ntr])
                    yp_abl = mdl.predict(scr.transform(X_abl[ntr:]))
                    nmse_abl = nmse(y[ntr:], yp_abl)
                    importances.append(nmse_abl - nmse_val) # increase in error
            
            res_dict = {
                **meta,
                "nmse": nmse_val,
                "rmse": rmse_val,
                "nmse_U": nmse_U,
                "nmse_US": nmse_US,
                "ddi": ddi_val,
                "mean_r": mean_r,
                "unshared_var": unshared_var,
                "pc1_var": pc1_var,
                "participation_ratio": participation_ratio,
                "response_div": response_div,
                "dyn_range": dyn_range,
                "snr": mean_snr,
                "sens": mean_sens,
                "theta_std": theta_std
            }
            if config == 'parallel':
                for i in range(5):
                    res_dict[f"P{i+1}_imp"] = importances[i]
            results.append(res_dict)
            
        except Exception as e:
            print(f"Failed on {p}: {e}")
            
    df_res = pd.DataFrame(results)
    df_res.to_csv(os.path.join(args.out_dir, "analysis_results.csv"), index=False)
    
    # Paper-style plots
    with PdfPages(os.path.join(args.out_dir, "paper_plots.pdf")) as pdf:
        # Fig 2b: NMSE Boxplot
        fig, ax = plt.subplots()
        df_res.boxplot(column='nmse', by=['trajectory', 'config'], ax=ax, rot=45)
        plt.suptitle("")
        plt.title("NMSE by Trajectory and Topology")
        plt.ylabel("NMSE")
        plt.ylim(0, 2)
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
        
        # Fig 2c: DDI
        fig, ax = plt.subplots()
        df_res.boxplot(column='ddi', by='config', ax=ax)
        plt.suptitle("")
        plt.title("Delay-Decoding Index (DDI)")
        plt.ylabel("DDI")
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
        
        # Fig 3a & 3b: Mean r and unshared variance
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        df_res.boxplot(column='mean_r', by='config', ax=axes[0])
        axes[0].set_title("Mean Inter-pouch r")
        df_res.boxplot(column='unshared_var', by='config', ax=axes[1])
        axes[1].set_title("Unshared Var (1-r^2)")
        plt.suptitle("")
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
        
        # Fig 3c: unshared var vs nmse
        fig, ax = plt.subplots()
        for cfg in ['coupled', 'parallel']:
            sub = df_res[df_res['config'] == cfg]
            ax.scatter(sub['unshared_var'], sub['nmse'], label=cfg)
        ax.set_xlabel("Unshared Var (1-r^2)")
        ax.set_ylabel("NMSE")
        ax.set_title("Redundancy vs Prediction Error")
        ax.legend()
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
        
        # Fig 4: Pre-inflation
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        for cfg in ['coupled', 'parallel']:
            sub = df_res[df_res['config'] == cfg]
            axes[0].scatter(sub['pre_inflation'], sub['dyn_range'], label=cfg)
            axes[1].scatter(sub['pre_inflation'], sub['nmse'], label=cfg)
        axes[0].set_title("Pre-inflation vs Dynamic Range")
        axes[0].set_xlabel("Pre-inflation (PSI)")
        axes[0].set_ylabel("Dyn Range (PSI)")
        axes[1].set_title("Pre-inflation vs NMSE")
        axes[1].set_xlabel("Pre-inflation (PSI)")
        axes[1].set_ylabel("NMSE")
        axes[0].legend()
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)

    print(f"Analysis complete. Results in {args.out_dir}")

if __name__ == "__main__":
    main()
