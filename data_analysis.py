import os
import re
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Optional: for interactive tooltips
try:
    import mplcursors

    HAS_MPLCURSORS = True
except ImportError:
    HAS_MPLCURSORS = False

# =================================================================================
# ---- DATA_MAPPING & SETTINGS (EDITABLE) ----
# =================================================================================
# NOTE: All column indices are based on the CSV file structure.

# -- General Settings --
EXPERIMENTS_BASE_DIR = "/home/g1/Developer/RISE_Lab/experiments"
START_TIME_OFFSET_SEC = 10  # Time in seconds to skip at the beginning of the plot

# -- Column Indices --
# Based on CSV structure: time,pd_4,pd_7,pd_8,pm_4_1,pm_4_2,pm_4_3,pm_4_4,pm_7_1,pm_7_2,pm_7_3,pm_7_4,pm_8_1,pm_8_2,pm_8_3,pm_8_4,mocap1_x,mocap1_y,mocap1_z,mocap1_qx,mocap1_qy,mocap1_qz,mocap1_qw,mocap2_x,mocap2_y,mocap2_z,mocap2_qx,mocap2_qy,mocap2_qz,mocap2_qw,mocap3_x,mocap3_y,mocap3_z,mocap3_qx,mocap3_qy,mocap3_qz,mocap3_qw
TIME_COL = 0
DESIRED_PRESSURE_COLS = [1, 2, 3]  # pd_4, pd_7, pd_8
MEASURED_PRESSURE_ARD4_COLS = [4, 5, 6, 7]  # pm_4_1, pm_4_2, pm_4_3, pm_4_4
MEASURED_PRESSURE_ARD7_COLS = [8, 9, 10, 11]  # pm_7_1, pm_7_2, pm_7_3, pm_7_4
MEASURED_PRESSURE_ARD8_COLS = [12, 13, 14, 15]  # pm_8_1, pm_8_2, pm_8_3, pm_8_4

# Mocap Body 3 position (x, y, z) - columns 31, 32, 33 (0-based indexing)
MOCAP_POS_X_COL = 30  # mocap3_x (column 31 in 1-based)
MOCAP_POS_Y_COL = 31  # mocap3_y (column 32 in 1-based)
MOCAP_POS_Z_COL = 32  # mocap3_z (column 33 in 1-based)

# Mocap Body 3 quaternion (qx, qy, qz, qw) - columns 34, 35, 36, 37 (0-based indexing)
MOCAP_QUAT_SLICE = slice(33, 37)  # mocap3_qx, mocap3_qy, mocap3_qz, mocap3_qw

# -- Derived Column Names (for internal use) --
YAW_BODY_NAME = "yaw_body"
PITCH_BODY_NAME = "pitch_body"
ROLL_BODY_NAME = "roll_body"
# =================================================================================


# ---- PLOT CONFIGURATION ----
# This section uses the variables defined above to configure the plots.

MOCAP_PLOT_CONFIG = [
    {
        "title": "Mocap Position (Body 3 - Trajectory)",
        "xlabel": "Time (s)",
        "ylabel": "Position (m)",
        "columns": [MOCAP_POS_X_COL, MOCAP_POS_Y_COL, MOCAP_POS_Z_COL],
        "labels": ["X Position", "Y Position", "Z Position"],
        "colors": ["tab:blue", "tab:orange", "tab:green"],
    },
    {
        "title": "Mocap Yaw Orientation (Body 3 - Trajectory)",
        "xlabel": "Time (s)",
        "ylabel": "Yaw (rad)",
        "columns": [YAW_BODY_NAME],
        "labels": ["Yaw"],
        "colors": ["tab:red"],
    },
    {
        "title": "Mocap Pitch Orientation (Body 3 - Trajectory)",
        "xlabel": "Time (s)",
        "ylabel": "Pitch (rad)",
        "columns": [PITCH_BODY_NAME],
        "labels": ["Pitch"],
        "colors": ["tab:purple"],
    },
]

SENSOR_CONTROL_CONFIG = [
    {
        "title": "Desired Pressures (Arduino 4, 7, 8)",
        "xlabel": "Time (s)",
        "ylabel": "Desired Pressure (PSI)",
        "columns": DESIRED_PRESSURE_COLS,
        "labels": ["Arduino 4", "Arduino 7", "Arduino 8"],
        "colors": ["tab:blue", "tab:orange", "tab:green"],
    },
    {
        "title": "Measured Pressures (Arduino 4)",
        "xlabel": "Time (s)",
        "ylabel": "Sensor Pressure (PSI)",
        "columns": MEASURED_PRESSURE_ARD4_COLS,
        "labels": ["Sensor 1", "Sensor 2", "Sensor 3", "Sensor 4"],
        "colors": ["tab:blue", "tab:orange", "tab:green", "tab:red"],
    },
    {
        "title": "Measured Pressures (Arduino 7)",
        "xlabel": "Time (s)",
        "ylabel": "Sensor Pressure (PSI)",
        "columns": MEASURED_PRESSURE_ARD7_COLS,
        "labels": ["Sensor 1", "Sensor 2", "Sensor 3", "Sensor 4"],
        "colors": ["tab:blue", "tab:orange", "tab:green", "tab:red"],
    },
    {
        "title": "Measured Pressures (Arduino 8)",
        "xlabel": "Time (s)",
        "ylabel": "Sensor Pressure (PSI)",
        "columns": MEASURED_PRESSURE_ARD8_COLS,
        "labels": ["Sensor 1", "Sensor 2", "Sensor 3", "Sensor 4"],
        "colors": ["tab:blue", "tab:orange", "tab:green", "tab:red"],
    },
]


def get_experiment():
    """
    Finds the latest experiment file based on a specific directory structure.
    It looks for folders named like 'Month-Day' and finds the most recent one,
    then finds the highest numbered 'Test_*.csv' file within that folder.
    """
    try:
        # NOTE: You may need to adjust this path to your experiments folder.
        experiments_base_dir = "/home/g1/Developer/RISE_Lab/experiments"
        if not os.path.exists(experiments_base_dir):
            raise FileNotFoundError
        folder_names = [
            name
            for name in os.listdir(experiments_base_dir)
            if os.path.isdir(os.path.join(experiments_base_dir, name))
            and not name.startswith(".")
        ]
        # Regex to match folder names like "June-25"
        date_folder_pattern = re.compile(r"^[A-Za-z]+-\d{1,2}$")
        date_folders = [
            name for name in folder_names if date_folder_pattern.match(name)
        ]
        if not date_folders:
            raise RuntimeError("No valid date-named folders found in experiments.")

        def folder_to_date(folder):
            """Converts a folder name like 'June-25' to a datetime object."""
            try:
                # Assume current year. This simplifies things but has edge cases.
                return datetime.strptime(folder, "%B-%d").replace(
                    year=datetime.now().year
                )
            except ValueError:
                return None

        dated_folders = [(folder, folder_to_date(folder)) for folder in date_folders]
        dated_folders = [item for item in dated_folders if item[1] is not None]
        if not dated_folders:
            raise RuntimeError("No valid date-named folders found in experiments.")

        # Sort folders by date to find the most recent one.
        # This handles the year-end transition correctly.
        current_date = datetime.now()
        dated_folders.sort(
            key=lambda x: (
                x[1].year if x[1].month <= current_date.month else x[1].year - 1,
                x[1].month,
                x[1].day,
            ),
            reverse=True,
        )
        latest_folder = dated_folders[0][0]

        latest_folder_path = os.path.join(experiments_base_dir, latest_folder)
        # Find all files matching the 'Test_X_Y.csv' pattern.
        test_files = [
            f
            for f in os.listdir(latest_folder_path)
            if re.match(r"Test_\d+_\d+\.csv$", f)
        ]
        if not test_files:
            raise RuntimeError(
                f"Latest date folder ({latest_folder}) contains no valid 'Test_X_Y.csv' files."
            )
        # Extract the second number from the filename to find the latest test.
        test_nums = []
        for fname in test_files:
            m = re.match(r"Test_\d+_(\d+)\.csv$", fname)
            if m:
                test_nums.append((int(m.group(1)), fname))
        if not test_nums:
            raise RuntimeError(
                f"Latest date folder ({latest_folder}) contains no valid 'Test_X_Y.csv' files."
            )
        latest_test_file = max(test_nums, key=lambda x: x[0])[1]
        filename = os.path.join(latest_folder_path, latest_test_file)
        print(f"Latest experiment file: {filename}")
        return filename
    except FileNotFoundError:
        print("Error: The specified experiment directory was not found.")
        print("Please check the 'experiments_base_dir' path in the script.")
        exit()
    except RuntimeError as e:
        print(f"Error finding experiment file: {e}")
        exit()


def quaternion_to_roll(qx, qy, qz, qw):
    """Calculates roll (X-axis rotation)."""
    return np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))


def quaternion_to_pitch(qx, qy, qz, qw):
    """Calculates pitch (Y-axis rotation)."""
    arg = np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0)
    return np.arcsin(arg)


def quaternion_to_yaw(qx, qy, qz, qw):
    """Calculates yaw (Z-axis rotation)."""
    return -np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))


def create_plot_window(
    fig_num, plot_configs, data, derived, time, headers, window_title
):
    """Helper to create a figure window and populate it with 2D plots."""
    num_plots = len(plot_configs)
    fig, axes = plt.subplots(
        num_plots, 1, figsize=(14, 5 * num_plots), num=fig_num, squeeze=False
    )
    axes = axes.flatten()
    fig.suptitle(window_title, fontsize=22, fontweight="bold", y=0.99)

    for ax, plot_cfg in zip(axes, plot_configs):
        for i, col in enumerate(plot_cfg["columns"]):
            y_data, label = (None, None)
            if isinstance(col, str):
                y_data = derived.get(col)
                label = plot_cfg.get("labels", [col])[i]
            elif col < data.shape[1]:
                y_data = data.iloc[:, col].values
                label = headers[col]
            else:
                print(f"Warning: Column index {col} out of range. Skipping.")
                continue

            if y_data is not None:
                ax.plot(
                    time,
                    y_data,
                    label=label,
                    color=plot_cfg["colors"][i % len(plot_cfg["colors"])],
                    linewidth=2.5,
                )

        ax.set_xlabel(plot_cfg.get("xlabel", "Time (s)"), fontsize=16)
        ax.set_ylabel(plot_cfg.get("ylabel", "Position (m)"), fontsize=16)
        ax.set_title(plot_cfg["title"], fontsize=17, pad=12)
        ax.legend(fontsize=13, loc="upper right", frameon=True)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.tick_params(axis="both", which="major", labelsize=13)

    fig.tight_layout(rect=[0, 0, 1, 0.96], h_pad=2.5)
    if HAS_MPLCURSORS:
        for ax in axes:
            mplcursors.cursor(ax.lines, hover=True)


def create_3d_mocap_plot(fig_num, data, window_title):
    """Creates a 3D plot for the mocap trajectory."""
    fig = plt.figure(num=fig_num, figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    x = data.iloc[:, MOCAP_POS_X_COL].values
    y = data.iloc[:, MOCAP_POS_Y_COL].values
    z = data.iloc[:, MOCAP_POS_Z_COL].values

    ax.plot(x, y, z, label="Trajectory")
    ax.scatter(
        x[0], y[0], z[0], c="g", s=100, marker="o", label="Start", depthshade=False
    )
    ax.scatter(
        x[-1], y[-1], z[-1], c="r", s=100, marker="s", label="End", depthshade=False
    )

    ax.set_xlabel("X Position (m)", fontweight="bold", fontsize=12)
    ax.set_ylabel("Y Position (m)", fontweight="bold", fontsize=12)
    ax.set_zlabel("Z Position (m)", fontweight="bold", fontsize=12)
    ax.set_title(window_title, fontsize=20, fontweight="bold", pad=20)
    ax.legend()
    ax.grid(True)

    max_range = (
        np.array([x.max() - x.min(), y.max() - y.min(), z.max() - z.min()]).max() / 2.0
    )
    if max_range == 0:
        max_range = 1
    mid_x, mid_y, mid_z = (
        (x.max() + x.min()) / 2,
        (y.max() + y.min()) / 2,
        (z.max() + z.min()) / 2,
    )
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


def main():
    """Main function to run the data analysis and plotting."""
    filename = get_experiment()
    # filename = "/home/g1/Developer/RISE_Lab/colcon_ws/experiments/June-23/Test_5.csv"
    # filename = '/home/g1/Developer/RISE_Lab/colcon_ws/experiments/July-10/cleaned_data/Test_1.csv'
    if not filename:
        return

    print(f"\nAnalyzing:\n{filename}\n")
    data = pd.read_csv(filename)
    if data.empty:
        raise ValueError("Data could not be read.")

    time = data.iloc[:, TIME_COL].values
    if time[-1] >= START_TIME_OFFSET_SEC:
        print(f"Slicing data to start from {START_TIME_OFFSET_SEC} seconds.")
        start_index = np.argmax(time >= START_TIME_OFFSET_SEC)
        time = time[start_index:]
        data = data.iloc[start_index:].reset_index(drop=True)

    derived = {}
    try:
        quat_body = data.iloc[:, MOCAP_QUAT_SLICE].astype(float).values
        qx, qy, qz, qw = quat_body.T
        derived[YAW_BODY_NAME] = quaternion_to_yaw(qx, qy, qz, qw)
        derived[PITCH_BODY_NAME] = quaternion_to_pitch(qx, qy, qz, qw)
        derived[ROLL_BODY_NAME] = quaternion_to_roll(qx, qy, qz, qw)
    except Exception as e:
        print(f"Could not calculate orientation from quaternions: {e}")

    headers = list(data.columns)
    base_title = os.path.basename(filename)
    plt.style.use("seaborn-v0_8-whitegrid")

    create_plot_window(
        1,
        MOCAP_PLOT_CONFIG,
        data,
        derived,
        time,
        headers,
        f"Mocap Data (Time Series - Body 3): {base_title}",
    )
    create_plot_window(
        2,
        SENSOR_CONTROL_CONFIG,
        data,
        derived,
        time,
        headers,
        f"Sensor & Control Data: {base_title}",
    )
    create_3d_mocap_plot(3, data, f"Mocap 3D Trajectory (Body 3): {base_title}")

    plt.show()


if __name__ == "__main__":
    main()
