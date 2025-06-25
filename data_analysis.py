import os
import re
from datetime import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Optional: for interactive tooltips
try:
    import mplcursors
    HAS_MPLCURSORS = True
except ImportError:
    HAS_MPLCURSORS = False

# ---- CONFIGURATION ----
PLOT_CONFIG = [
    {
        "title": "Desired Pressures",
        "ylabel": "Desired Pressure",
        "columns": [5,6],  # column indices (0-based)
        "labels": None,      # Use None to use column headers
        "colors": ["tab:blue", "tab:orange", "tab:green"],
        "plot_type": "line",
    },
    {
        "title": "Mocap Body Yaw Orientation (Body 3)",
        "ylabel": "Yaw (rad)",
        "columns": ["yaw_body3"],  # special: calculated column
        "labels": ["Yaw"],
        "colors": ["tab:red"],
        "plot_type": "line",
    },
    {
        "title": "Mocap Body Pitch Orientation (Body 3)",
        "ylabel": "Pitch (rad)",
        "columns": ["pitch_body3"], # special: calculated column
        "labels": ["Pitch"],
        "colors": ["tab:green"],
        "plot_type": "line",
    },
    {
        "title": "Sensing Column (Segment 1)",
        "ylabel": "Sensor Pressure",
        "columns": list(range(6, 11)),
        "labels": None,
        "colors": [
            "tab:blue", "tab:orange", "tab:green",
            "tab:red", "tab:purple"
        ],
        "plot_type": "line",
    },
    {
        "title": "Sensing Column (Segment 3)",
        "ylabel": "Sensor Pressure",
        "columns": list(range(11, 16 )),
        "labels": None,
        "colors": [
            "tab:blue", "tab:orange", "tab:green",
            "tab:red", "tab:purple"
        ],
        "plot_type": "line",
    },
]
TIME_COLUMN = 0  # index of time column

# ---- END CONFIGURATION ----

def get_experiment():
    experiments_base_dir = '/home/g1/Developer/RISE_Lab/colcon_ws/experiments'
    folder_names = [
        name for name in os.listdir(experiments_base_dir)
        if os.path.isdir(os.path.join(experiments_base_dir, name))
        and not name.startswith('.')
    ]
    date_folder_pattern = re.compile(r'^[A-Za-z]+-\d{2}$')
    date_folders = [name for name in folder_names if date_folder_pattern.match(name)]
    if not date_folders:
        raise RuntimeError("No valid date-named folders found in experiments.")
    def folder_to_date(folder):
        try:
            return datetime.strptime(folder, "%B-%d")
        except ValueError:
            return None
    dated_folders = [(folder, folder_to_date(folder)) for folder in date_folders]
    dated_folders = [item for item in dated_folders if item[1] is not None]
    if not dated_folders:
        raise RuntimeError("No valid date-named folders found in experiments.")
    latest_folder = max(dated_folders, key=lambda x: x[1])[0]
    latest_folder_path = os.path.join(experiments_base_dir, latest_folder)
    test_files = [
        f for f in os.listdir(latest_folder_path)
        if re.match(r'Test_\d+\.csv$', f)
    ]
    if not test_files:
        raise RuntimeError(
            f"Latest date folder ({latest_folder}) contains no valid 'Test_*.csv' files."
        )
    test_nums = []
    for fname in test_files:
        m = re.match(r'Test_(\d+)\.csv$', fname)
        if m:
            test_nums.append((int(m.group(1)), fname))
    if not test_nums:
        raise RuntimeError(
            f"Latest date folder ({latest_folder}) contains no valid 'Test_*.csv' files."
        )
    latest_test_file = max(test_nums, key=lambda x: x[0])[1]
    filename = os.path.join(latest_folder_path, latest_test_file)
    print(f"Latest experiment file: {filename}")
    return filename

def quaternion_to_yaw(qx, qy, qz, qw):
    """Calculates yaw (Z-axis rotation) from a quaternion."""
    return -np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

def quaternion_to_pitch(qx, qy, qz, qw):
    """Calculates pitch (Y-axis rotation) from a quaternion."""
    # Ensure the argument to asin is within the valid range [-1, 1]
    arg = 2 * (qw * qy - qz * qx)
    arg = np.clip(arg, -1.0, 1.0)
    return np.arcsin(arg)

def main():
    filename = get_experiment()
    print(f"\nAnalyzing:\n{filename}\n")

    data = pd.read_csv(filename)
    if data.empty:
        raise ValueError("Data could not be read. Check file path or format.")
        
    time = data.iloc[:, TIME_COLUMN].values

    # ---- NEW: Filter data to start from 10 seconds ----
    start_time_sec = 10
    if time[-1] >= start_time_sec:
        print(f"Slicing data to start from {start_time_sec} seconds.")
        # Find the index of the first time value >= 10
        start_index = np.argmax(time >= start_time_sec)
        
        # Slice the time array and the main DataFrame
        time = time[start_index:]
        data = data.iloc[start_index:].reset_index(drop=True)
    else:
        print(f"Warning: Total experiment duration is less than {start_time_sec}s. Plotting from beginning.")
    # ---- END NEW ----

    # Precompute any derived columns
    derived = {}
    try:
        # Based on the headers, mocap3 quaternion data is in columns 33-36
        # mocap3_qx, mocap3_qy, mocap3_qz, mocap3_qw
        quat_body3 = data.iloc[:, 33:37].astype(float).values
        
        # Calculate Yaw for Body 3
        derived["yaw_body3"] = quaternion_to_yaw(
            quat_body3[:, 0], quat_body3[:, 1], quat_body3[:, 2], quat_body3[:, 3]
        )
        
        # Calculate Pitch for Body 3
        derived["pitch_body3"] = quaternion_to_pitch(
            quat_body3[:, 0], quat_body3[:, 1], quat_body3[:, 2], quat_body3[:, 3]
        )
    except Exception as e:
        print(f"Could not calculate orientation: {e}")
        derived["yaw_body3"] = None
        derived["pitch_body3"] = None


    headers = list(data.columns)

    # --- Improved Matplotlib Style ---
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(
        len(PLOT_CONFIG), 1, figsize=(14, 4.5 * len(PLOT_CONFIG)), sharex=True
    )
    if len(PLOT_CONFIG) == 1:
        axes = [axes]
    fig.suptitle(
        f"Experiment Analysis: {os.path.basename(filename)}",
        fontsize=22,
        fontweight='bold',
        y=1.02
    )

    for ax, plot_cfg in zip(axes, PLOT_CONFIG):
        cols = plot_cfg["columns"]
        labels = plot_cfg.get("labels")
        colors = plot_cfg["colors"]
        plot_type = plot_cfg.get("plot_type", "line")

        for i, col in enumerate(cols):
            y_data = None
            if isinstance(col, str):
                y_data = derived.get(col, None)
                label = labels[i] if labels and i < len(labels) else col
            else:
                if col >= data.shape[1]:
                    print(f"Warning: Column index {col} out of range.")
                    continue
                y_data = data.iloc[:, col].values
                # Use header as label if labels is None
                label = (
                    headers[col]
                    if labels is None
                    else (labels[i] if i < len(labels) else f"Col {col}")
                )

            if y_data is not None:
                if plot_type == "line":
                    ax.plot(
                        time,
                        y_data,
                        label=label,
                        color=colors[i] if i < len(colors) else None,
                        linewidth=2.5,
                    )

        ax.set_ylabel(plot_cfg["ylabel"], fontsize=16)
        ax.set_title(plot_cfg["title"], fontsize=17, pad=12)
        ax.legend(fontsize=13, loc='upper right', frameon=True)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.tick_params(axis='both', which='major', labelsize=13)

    axes[-1].set_xlabel("Time (s)", fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.98], h_pad=2.0)

    # Optional: Interactive tooltips
    if HAS_MPLCURSORS:
        for ax in axes:
            mplcursors.cursor(ax.lines, hover=True)

    plt.show()

if __name__ == "__main__":
    main()
