#!/usr/bin/env python3
import os
import re
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.ticker import MultipleLocator

# Optional: for interactive tooltips
try:
    import mplcursors

    HAS_MPLCURSORS = True
except ImportError:
    HAS_MPLCURSORS = False

# =================================================================================
# ---- DATA_MAPPING & SETTINGS (EDITABLE) ----
# =================================================================================
# NOTE: Column mapping now uses header names (strings) instead of fixed indices.

# -- General Settings --
EXPERIMENTS_BASE_DIR = "/home/g1/Developer/Thesis/experiments"
# EXPERIMENTS_BASE_DIR = "/Users/g1/Developer/Thesis/experiments"
START_TIME_OFFSET_SEC = 10  # Time in seconds to skip at the beginning

# time	pd_4	pd_7	pd_8	pm_1	pm_2	pm_3	pm_4	pm_5	pm_6	pm_7	pm_8	pm_9	pm_10	pm_11	pm_12	pm_13	pm_14	pm_15	pm_16	mocap_1	mocap_2	mocap_3	mocap_4	mocap_5	mocap_6	mocap_7	mocap_8	mocap_9	mocap_10	mocap_11	mocap_12	mocap_13	mocap_14	mocap_15	mocap_16	mocap_17	mocap_18	mocap_19	mocap_20	mocap_21


# -- Column Names --
TIME_COL = "time"


# DESIRED_PRESSURE_COLS = [
#     "Desired_pressure_segment_1",
#     "Desired_pressure_segment_2",
#     "Desired_pressure_segment_3",
#     "Desired_pressure_segment_4",
# ]

DESIRED_PRESSURE_COLS = ["pd_3", "pd_6", "pd_7", "pd_8"]

# Segment-based grouping of measured pressure sensors
# MEASURED_PRESSURE_SEGMENT1_COLS = ["pm_1", "pm_2", "pm_3", "pm_4", "pm_5"]
# MEASURED_PRESSURE_SEGMENT1_COLS = ["pm_1", "pm_2", "pm_3", "pm_4", "pm_5"]
# MEASURED_PRESSURE_SEGMENT2_COLS = ["pm_6", "pm_7", "pm_8", "pm_9", "pm_10"]
# MEASURED_PRESSURE_SEGMENT3_COLS = ["pm_11"]
# MEASURED_PRESSURE_SEGMENT4_COLS = ["pm_12"]

# MEASURED_PRESSURE_SEGMENT1_COLS = [
#     "Measured_pressure_Segment_1_pouch_1",
#     "Measured_pressure_Segment_1_pouch_2",
#     "Measured_pressure_Segment_1_pouch_3",
#     "Measured_pressure_Segment_1_pouch_4",
#     "Measured_pressure_Segment_1_pouch_5",
# ]
# MEASURED_PRESSURE_SEGMENT2_COLS = [
#     "Measured_pressure_Segment_2_pouch_1",
#     "Measured_pressure_Segment_2_pouch_2",
#     "Measured_pressure_Segment_2_pouch_3",
#     "Measured_pressure_Segment_2_pouch_4",
#     "Measured_pressure_Segment_2_pouch_5",
# ]
# MEASURED_PRESSURE_SEGMENT3_COLS = ["Measured_pressure_Segment_3"]
# MEASURED_PRESSURE_SEGMENT4_COLS = ["Measured_pressure_Segment_4"]

MEASURED_PRESSURE_SEGMENT1_COLS = ["pm_3_1", "pm_3_2", "pm_3_3", "pm_3_4", "pm_7_1"]
MEASURED_PRESSURE_SEGMENT2_COLS = ["pm_7_2", "pm_7_3", "pm_7_4", "pm_8_1", "pm_8_2"]
MEASURED_PRESSURE_SEGMENT3_COLS = ["pm_8_4"]
MEASURED_PRESSURE_SEGMENT4_COLS = ["pm_8_3"]
MEASURED_PRESSURE_SEGMENT4_COLS = ["pm_8_3"]

# Mocap Body 3 position (x, y, z)
# MOCAP_POS_COLS = ["mocap_15", "mocap_16", "mocap_17"]

MOCAP_POS_COLS = ["mocap_3_x", "mocap_3_y", "mocap_3_z"]

# MOCAP_POS_COLS = ["mocap_rigid_body_x", "mocap_rigid_body_y", "mocap_rigid_body_z"]


# Mocap Body 3 quaternion (qx, qy, qz, qw)

# MOCAP_QUAT_COLS = ["mocap_18", "mocap_19", "mocap_20", "mocap_21"]

MOCAP_QUAT_COLS = ["mocap_3_qx", "mocap_3_qy", "mocap_3_qz", "mocap_3_qw"]

# MOCAP_QUAT_COLS = [
#     "mocap_rigid_body_qx",
#     "mocap_rigid_body_qy",
#     "mocap_rigid_body_qz",
#     "mocap_rigid_body_qw",
# ]
# # MOCAP_QUAT_COLS = ["mocap_3_qx", "mocap_3_qy", "mocap_3_qz", "mocap_3_qw"]

# -- Derived Column Names (for internal use) --
YAW_BODY_NAME = "yaw_body"
PITCH_BODY_NAME = "pitch_body"
ROLL_BODY_NAME = "roll_body"
# =================================================================================


# ---- PLOT CONFIGURATION ----
MOCAP_PLOT_CONFIG = [
    {
        "title": "Mocap Position (Body 3 - Trajectory)",
        "xlabel": "Time (s)",
        "ylabel": "Position ",
        "columns": MOCAP_POS_COLS,
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

SENSOR_CONTROL_CONFIG_1 = [
    {
        "title": "Desired Pressures",
        "xlabel": "Time (s)",
        "ylabel": "Desired Pressure (PSI)",
        "columns": DESIRED_PRESSURE_COLS,
        "labels": ["pd_segment_1", "pd_segment_2", "pd_segment_3", "pd_segment_4"],
        "colors": ["tab:red", "tab:orange", "tab:blue", "tab:green"],
    },
    {
        "title": "Measured Pressures (Segment 3)",
        "xlabel": "Time (s)",
        "ylabel": "Sensor Pressure (PSI)",
        "columns": MEASURED_PRESSURE_SEGMENT3_COLS,
        "labels": ["pm_segment_3"],
        "colors": ["tab:blue", "tab:cyan"],
    },
    {
        "title": "Measured Pressures (Segment 4)",
        "xlabel": "Time (s)",
        "ylabel": "Sensor Pressure (PSI)",
        "columns": MEASURED_PRESSURE_SEGMENT4_COLS,
        "labels": ["pm_segment_4"],
        "colors": ["tab:purple"],
    },
]

SENSOR_CONTROL_CONFIG_2 = [
    {
        "title": "Measured Pressures (Segment 1)",
        "xlabel": "Time (s)",
        "ylabel": "Sensor Pressure (PSI)",
        "columns": MEASURED_PRESSURE_SEGMENT1_COLS,
        "labels": [
            "Segment_1_pouch_1",
            "Segment_1_pouch_2",
            "Segment_1_pouch_3",
            "Segment_1_pouch_4",
            "Segment_1_pouch_5",
        ],
        "colors": ["tab:red", "tab:pink", "crimson", "tab:brown", "salmon"],
    },
    {
        "title": "Measured Pressures (Segment 2)",
        "xlabel": "Time (s)",
        "ylabel": "Sensor Pressure (PSI)",
        "columns": MEASURED_PRESSURE_SEGMENT2_COLS,
        "labels": [
            "Segment_2_pouch_1",
            "Segment_2_pouch_1",
            "Segment_2_pouch_2",
            "Segment_2_pouch_3",
            "Segment_2_pouch_4",
            "Segment_2_pouch_5",
        ],
        "colors": ["tab:orange", "tab:olive", "gold", "darkorange", "peru"],
    },
]


def get_experiment():
    """Finds the latest experiment file based on directory structure."""
    try:
        if not os.path.exists(EXPERIMENTS_BASE_DIR):
            raise FileNotFoundError
        folder_names = [
            name
            for name in os.listdir(EXPERIMENTS_BASE_DIR)
            if os.path.isdir(os.path.join(EXPERIMENTS_BASE_DIR, name))
            and not name.startswith(".")
        ]
        date_folder_pattern = re.compile(r"^[A-Za-z]+-\d{1,2}$")
        date_folders = [
            name for name in folder_names if date_folder_pattern.match(name)
        ]
        if not date_folders:
            raise RuntimeError("No valid date-named folders found.")

        def folder_to_date(folder):
            try:
                return datetime.strptime(folder, "%B-%d").replace(
                    year=datetime.now().year
                )
            except ValueError:
                return None

        dated_folders = [(folder, folder_to_date(folder)) for folder in date_folders]
        dated_folders = [item for item in dated_folders if item[1] is not None]
        if not dated_folders:
            raise RuntimeError("No valid date-named folders found.")

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

        latest_folder_path = os.path.join(EXPERIMENTS_BASE_DIR, latest_folder)
        test_files = [
            f
            for f in os.listdir(latest_folder_path)
            if re.match(r"(Test_\d+_\d+|Experiment_\d+)\.csv$", f)
        ]
        if not test_files:
            raise RuntimeError(f"No valid test files found in '{latest_folder}'.")

        test_nums = []
        for fname in test_files:
            m = re.match(r"Test_\d+_(\d+)\.csv$|Experiment_(\d+)\.csv$", fname)
            if m:
                num = m.group(1) or m.group(2)
                test_nums.append((int(num), fname))
        if not test_nums:
            raise RuntimeError(
                f"No validly named test files found in '{latest_folder}'."
            )

        latest_test_file = max(test_nums, key=lambda x: x[0])[1]
        filename = os.path.join(latest_folder_path, latest_test_file)
        print(f"Latest experiment file: {filename}")
        return filename
    except FileNotFoundError:
        print(f"Error: Directory not found. Check 'EXPERIMENTS_BASE_DIR'.")
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
    fig_num, plot_configs, data, time, window_title, x_tick_interval=None
):
    """Helper to create a figure window and populate it with 2D plots."""
    num_plots = len(plot_configs)
    fig, axes = plt.subplots(
        num_plots, 1, figsize=(16, 6 * num_plots), num=fig_num, squeeze=False
    )
    axes = axes.flatten()
    fig.suptitle(window_title, fontsize=22, fontweight="bold", y=0.995)

    for ax, plot_cfg in zip(axes, plot_configs):
        # Filter for columns that actually exist in the DataFrame
        columns_to_plot = [col for col in plot_cfg["columns"] if col in data]
        if not columns_to_plot:
            print(
                f"Warning: None of {plot_cfg['columns']} found for plot '{plot_cfg['title']}'. Skipping."
            )
            continue

        for i, col_name in enumerate(columns_to_plot):
            ax.plot(
                time,
                data[col_name].values,
                label=plot_cfg["labels"][i % len(plot_cfg["labels"])],
                color=plot_cfg["colors"][i % len(plot_cfg["colors"])],
                linewidth=2.5,
            )

        ax.set_xlabel(plot_cfg.get("xlabel", "Time (s)"), fontsize=16)
        ax.set_ylabel(plot_cfg.get("ylabel", "Value"), fontsize=16)
        ax.set_title(plot_cfg["title"], fontsize=17, pad=12)
        ax.legend(fontsize=13, loc="upper right", frameon=True)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.tick_params(axis="both", which="major", labelsize=13)

        # Set major ticks interval on the x-axis if specified
        if x_tick_interval:
            ax.xaxis.set_major_locator(MultipleLocator(x_tick_interval))

    fig.tight_layout(rect=[0, 0, 1, 0.99], h_pad=4.0, pad=3.0)
    if HAS_MPLCURSORS:
        for ax in axes:
            mplcursors.cursor(ax.lines, hover=True)


def create_3d_mocap_plot(fig_num, data, window_title):
    """Creates a 3D plot for the mocap trajectory."""
    # Check if all required columns exist
    if not all(col in data.columns for col in MOCAP_POS_COLS):
        print("Warning: Missing one or more mocap position columns. Skipping 3D plot.")
        return

    fig = plt.figure(num=fig_num, figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    x, y, z = data[MOCAP_POS_COLS].values.T

    ax.plot(x, y, z, label="Trajectory")
    ax.scatter(
        x[0], y[0], z[0], c="g", s=100, marker="o", label="Start", depthshade=False
    )
    ax.scatter(
        x[-1], y[-1], z[-1], c="r", s=100, marker="s", label="End", depthshade=False
    )

    ax.set_xlabel("X Position ", fontweight="bold", fontsize=12)
    ax.set_ylabel("Y Position ", fontweight="bold", fontsize=12)
    ax.set_zlabel("Z Position ", fontweight="bold", fontsize=12)
    ax.set_title(window_title, fontsize=20, fontweight="bold", pad=20)
    ax.legend()
    ax.grid(True)

    max_range = np.ptp(np.vstack([x, y, z]), axis=1).max() / 2.0 or 1.0
    mid_x, mid_y, mid_z = np.mean(x), np.mean(y), np.mean(z)
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


def main():
    """Main function to run the data analysis and plotting."""
    # filename = get_experiment()

    filename = "experiments/October-25/Experiment_14.csv"
    # filename = "experiments/October-25/cleaned_data/circular_motion_5_psi_peak.csv"
    if not filename:
        return
    print(f"\nAnalyzing:\n{filename}\n")
    try:
        data = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return

    if data.empty:
        print("Error: Data file is empty.")
        return

    # Create a new DataFrame for derived data (like yaw, pitch, roll)
    # This avoids modifying the original data DataFrame
    derived_data = pd.DataFrame()

    # Calculate orientation from quaternions if columns exist
    if all(col in data.columns for col in MOCAP_QUAT_COLS):
        try:
            quat_body = data[MOCAP_QUAT_COLS].astype(float).values
            qx, qy, qz, qw = quat_body.T
            derived_data[YAW_BODY_NAME] = quaternion_to_yaw(qx, qy, qz, qw)
            derived_data[PITCH_BODY_NAME] = quaternion_to_pitch(qx, qy, qz, qw)
            derived_data[ROLL_BODY_NAME] = quaternion_to_roll(qx, qy, qz, qw)
        except Exception as e:
            print(f"Warning: Could not calculate orientation from quaternions: {e}")

    # Combine original and derived data for plotting
    plot_data = pd.concat([data, derived_data], axis=1)

    # Prepare time vector and trim data if needed
    time = plot_data[TIME_COL].values
    if time[-1] >= START_TIME_OFFSET_SEC:
        print(f"Slicing data to start from {START_TIME_OFFSET_SEC} seconds.")
        start_index = np.argmax(time >= START_TIME_OFFSET_SEC)
        time = time[start_index:]
        plot_data = plot_data.iloc[start_index:].reset_index(drop=True)

    base_title = os.path.basename(filename)

    create_plot_window(
        1,
        SENSOR_CONTROL_CONFIG_1,
        plot_data,
        time,
        f"Sensor & Control Data (Desired, Segments 3 & 4): {base_title}",
        # x_tick_interval=3,  # Set x-axis ticks to 1-second intervals
    )

    create_plot_window(
        2,
        SENSOR_CONTROL_CONFIG_2,
        plot_data,
        time,
        f"Sensor & Control Data (Segments 1 & 2): {base_title}",
    )

    create_plot_window(
        3,
        MOCAP_PLOT_CONFIG,
        plot_data,
        time,
        f"Mocap Data (Time Series - Body 3): {base_title}",
    )

    create_3d_mocap_plot(4, plot_data, f"Mocap 3D Trajectory (Body 3): {base_title}")

    plt.show()


if __name__ == "__main__":
    main()
