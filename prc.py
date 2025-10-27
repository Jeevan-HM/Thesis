import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_absolute_error, r2_score

# CSV_PATH = "Test_1.csv"  # change if needed
CSV_PATH = "experiments/cleaned_data/cleaned_data/Test_1.csv"

# 1) Load (no header) and keep first 16 columns
df = pd.read_csv(CSV_PATH, header=None)
df = df.iloc[:, :16].copy()
df.columns = ["time", "y1", "y2", "y3"] + [f"s{i}" for i in range(1, 13)]

# 2) Coerce numeric, drop bad rows, sort by time
for c in df.columns:
    df[c] = pd.to_numeric(df[c], errors="coerce")
df = df.dropna().sort_values("time").reset_index(drop=True)

# 3) Trim first/last 10 s by relative time
t0 = df["time"].iloc[0]
df["t_rel"] = df["time"] - t0
tmax = df["t_rel"].iloc[-1]
if tmax <= 20:
    raise ValueError(f"Duration {tmax:.3f}s is too short to drop 10s at start/end.")

df = df[(df["t_rel"] >= 10.0) & (df["t_rel"] <= (tmax - 10.0))].reset_index(drop=True)
if len(df) < 4:
    raise ValueError("Not enough samples after trimming.")

# 4) Split 50/50 by time order
mid = len(df) // 2
train = df.iloc[:mid].copy()
test = df.iloc[mid:].copy()
if len(train) == 0 or len(test) == 0:
    raise ValueError("Train/test split empty — check input length.")

# 5) Prepare features/targets
start_sensor = 3
X_train = train[[f"s{i}" for i in range(start_sensor, 13)]].values
Y_train = train[["y1", "y2", "y3"]].values
X_test = test[[f"s{i}" for i in range(start_sensor, 13)]].values
Y_test = test[["y1", "y2", "y3"]].values

# 6) Fit linear readout and predict
model = LinearRegression()
model.fit(X_train, Y_train)
Yhat_test = model.predict(X_test)

# 7) Quick metrics
r2_each = r2_score(Y_test, Yhat_test, multioutput="raw_values")
mae_each = mean_absolute_error(Y_test, Yhat_test, multioutput="raw_values")
print("R2 per target  [y1, y2, y3]:", np.round(r2_each, 4))
print("MAE per target [y1, y2, y3]:", np.round(mae_each, 4))

# 8) Plot test emulation (3 panels)
t_plot = test["time"].values - test["time"].iloc[0]
titles = ["y1 emulation (test)", "y2 emulation (test)", "y3 emulation (test)"]

fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
for i in range(3):
    axes[i].plot(t_plot, Y_test[:, i], label=f"y{i + 1} true", linewidth=1.6)
    axes[i].plot(t_plot, Yhat_test[:, i], "--", label=f"y{i + 1} pred", linewidth=1.3)
    axes[i].set_ylabel("amp")
    axes[i].set_title(titles[i])
    axes[i].grid(True, alpha=0.3)
    axes[i].legend(loc="upper right", frameon=False)

axes[-1].set_xlabel("time (s)")
fig.tight_layout()
plt.show()

# 9) Plot sensor time-series (test split) in 3 subplots:
#    (1) s1–s2, (2) s3–s7, (3) s8–s12
groups = [
    list(range(1, 3)),  # s1, s2
    list(range(3, 8)),  # s3..s7
    list(range(8, 13)),  # s8..s12
]

fig2, axes2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
group_titles = ["Sensors s1–s2 (test)", "Sensors s3–s7 (test)", "Sensors s8–s12 (test)"]

for ax, g, title in zip(axes2, groups, group_titles):
    for k in g:
        ax.plot(t_plot, test[f"s{k}"].values, label=f"s{k}", linewidth=1.0)
    ax.set_ylabel("sensor")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", ncol=3, frameon=False)

axes2[-1].set_xlabel("time (s)")
fig2.tight_layout()
plt.show()
