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

# Plots for the first window (Mocap Data)
MOCAP_PLOT_CONFIG = [
    {
        "title": "Mocap Position (Body 3)",
        "xlabel": "Time (s)",
        "ylabel": "Position (m)",
        "columns": [16, 17, 18],  # mocap3_x, mocap3_y, mocap3_z
        "labels": None,           # Use column headers
        "colors": ["tab:blue", "tab:orange", "tab:green"],
        "plot_type": "line",
    },
    {
        "title": "Mocap Yaw Orientation (Body 3)",
        "xlabel": "Time (s)",
        "ylabel": "Yaw (rad)",
        "columns": ["yaw_body3"],  # special: calculated column
        "labels": ["Yaw"],
        "colors": ["tab:red"],
        "plot_type": "line",
    },
    {
        "title": "Mocap Pitch Orientation (Body 3)",
        "xlabel": "Time (s)",
        "ylabel": "Pitch (rad)",
        "columns": ["pitch_body3"], # special: calculated column
        "labels": ["Pitch"],
        "colors": ["tab:purple"],
        "plot_type": "line",
    },
    # {
    #     "title": "Mocap Roll Orientation (Body 3)",
    #     "xlabel": "Time (s)",
    #     "ylabel": "Roll (rad)",
    #     "columns": ["roll_body3"], # special: calculated column
    #     "labels": ["Roll"],
    #     "colors": ["tab:cyan"],
    #     "plot_type": "line",
    # },
]

# Plots for the second window (Sensor and Control Data)
SENSOR_CONTROL_CONFIG = [
    {
        "title": "Desired Pressures",
        "xlabel": "Time (s)",
        "ylabel": "Desired Pressure (PSI)",
        "columns": [4,5, 6],
        "labels": None,
        "colors": ["tab:blue", "tab:orange", "tab:green"],
        "plot_type": "line",
    },
    {
        "title": "Sensing Column (Segment 1)",
        "xlabel": "Time (s)",
        "ylabel": "Sensor Pressure (PSI)",
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
        "xlabel": "Time (s)",
        "ylabel": "Sensor Pressure (PSI)",
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
    """
    Finds the latest experiment file based on a specific directory structure.
    It looks for folders named like 'Month-Day' and finds the most recent one,
    then finds the highest numbered 'Test_*.csv' file within that folder.
    """
    try:
        # NOTE: You may need to adjust this path to your experiments folder.
        experiments_base_dir = '/home/g1/Developer/RISE_Lab/colcon_ws/experiments'
        if not os.path.exists(experiments_base_dir):
             raise FileNotFoundError
        folder_names = [
            name for name in os.listdir(experiments_base_dir)
            if os.path.isdir(os.path.join(experiments_base_dir, name))
            and not name.startswith('.')
        ]
        # Regex to match folder names like "June-25"
        date_folder_pattern = re.compile(r'^[A-Za-z]+-\d{2}$')
        date_folders = [name for name in folder_names if date_folder_pattern.match(name)]
        if not date_folders:
            raise RuntimeError("No valid date-named folders found in experiments.")
            
        def folder_to_date(folder):
            """Converts a folder name like 'June-25' to a datetime object."""
            try:
                # Assume current year. This simplifies things but has edge cases.
                return datetime.strptime(folder, "%B-%d").replace(year=datetime.now().year)
            except ValueError:
                return None
                
        dated_folders = [(folder, folder_to_date(folder)) for folder in date_folders]
        dated_folders = [item for item in dated_folders if item[1] is not None]
        if not dated_folders:
            raise RuntimeError("No valid date-named folders found in experiments.")
        
        # Sort folders by date to find the most recent one.
        # This handles the year-end transition correctly.
        current_date = datetime.now()
        dated_folders.sort(key=lambda x: (x[1].year if x[1].month <= current_date.month else x[1].year - 1, x[1].month, x[1].day), reverse=True)
        latest_folder = dated_folders[0][0]

        latest_folder_path = os.path.join(experiments_base_dir, latest_folder)
        # Find all files matching the 'Test_*.csv' pattern.
        test_files = [
            f for f in os.listdir(latest_folder_path)
            if re.match(r'Test_\d+\.csv$', f)
        ]
        if not test_files:
            raise RuntimeError(
                f"Latest date folder ({latest_folder}) contains no valid 'Test_*.csv' files."
            )
        # Extract the number from the filename to find the latest test.
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
        filename = "/home/g1/Developer/RISE_Lab/colcon_ws/experiments/June-25/cleaned_data/Test_1.csv"
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
    """Calculates roll (X-axis rotation) from a quaternion."""
    return np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))

def quaternion_to_pitch(qx, qy, qz, qw):
    """Calculates pitch (Y-axis rotation) from a quaternion."""
    # Ensure the argument to arcsin is within the valid range [-1, 1]
    arg = 2 * (qw * qy - qz * qx)
    arg = np.clip(arg, -1.0, 1.0)
    return np.arcsin(arg)

def quaternion_to_yaw(qx, qy, qz, qw):
    """Calculates yaw (Z-axis rotation) from a quaternion."""
    return -np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

def create_plot_window(fig_num, plot_configs, data, derived, time, headers, window_title):
    """Helper function to create a figure window and populate it with 2D plots."""
    num_plots = len(plot_configs)
    # Note: Removed sharex=True to allow individual x-axis labels on each plot.
    fig, axes = plt.subplots(
        num_plots, 1, figsize=(14, 5 * num_plots), num=fig_num
    )
    if num_plots == 1:
        axes = [axes]
        
    fig.suptitle(window_title, fontsize=22, fontweight='bold', y=0.99)

    for ax, plot_cfg in zip(axes, plot_configs):
        cols = plot_cfg["columns"]
        labels = plot_cfg.get("labels")
        colors = plot_cfg["colors"]
        
        for i, col in enumerate(cols):
            y_data = None
            # Check if the column is a special calculated string or a numeric index
            if isinstance(col, str):
                y_data = derived.get(col, None)
                label = labels[i] if labels and i < len(labels) else col
            else:
                if col >= data.shape[1]:
                    print(f"Warning: Column index {col} is out of range for the data. Skipping.")
                    continue
                y_data = data.iloc[:, col].values
                # Determine the label for the line
                label = (
                    headers[col]
                    if labels is None
                    else (labels[i] if i < len(labels) else f"Col {col}")
                )

            # Plot the data if it was successfully retrieved
            if y_data is not None:
                ax.plot(
                    time,
                    y_data,
                    label=label,
                    color=colors[i % len(colors)] if colors else None,
                    linewidth=2.5,
                )

        # --- AXIS LABELING ---
        ax.set_xlabel(plot_cfg.get("xlabel", "Time (s)"), fontsize=16) # Set X-axis label for each plot
        ax.set_ylabel(plot_cfg["ylabel"], fontsize=16) # Set Y-axis label for each plot
        
        ax.set_title(plot_cfg["title"], fontsize=17, pad=12)
        ax.legend(fontsize=13, loc='upper right', frameon=True)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.tick_params(axis='both', which='major', labelsize=13)
        
    fig.tight_layout(rect=[0, 0, 1, 0.96], h_pad=2.5)

    # Add interactive cursors if the library is available
    if HAS_MPLCURSORS:
        for ax in axes:
            mplcursors.cursor(ax.lines, hover=True)

def create_3d_mocap_plot(fig_num, data, window_title):
    """Creates a 3D plot for the mocap trajectory with labeled axes."""
    fig = plt.figure(num=fig_num, figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Extract position data for Body 3
    x = data.iloc[:, 30].values
    y = data.iloc[:, 31].values
    z = data.iloc[:, 32].values

    # Plot the 3D trajectory
    ax.plot(x, y, z, label='Trajectory')

    # Mark the start and end points for clarity
    ax.scatter(x[0], y[0], z[0], c='g', s=100, marker='o', label='Start', depthshade=False)
    ax.scatter(x[-1], y[-1], z[-1], c='r', s=100, marker='s', label='End', depthshade=False)

    # --- AXIS LABELING ---
    ax.set_xlabel('X Position (m)', fontweight='bold', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontweight='bold', fontsize=12)
    ax.set_zlabel('Z Position (m)', fontweight='bold', fontsize=12)
    
    ax.set_title(window_title, fontsize=20, fontweight='bold', pad=20)
    ax.legend()
    ax.grid(True)
    
    # Set aspect ratio to be equal to make the plot less distorted and more intuitive
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    if max_range == 0: max_range = 1 # Avoid division by zero if the object is stationary
    mid_x = (x.max()+x.min()) * 0.5
    mid_y = (y.max()+y.min()) * 0.5
    mid_z = (z.max()+z.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


def main():
    """Main function to run the data analysis and plotting."""
    filename = get_experiment()
    # You can uncomment the line below to test with a specific file
    # filename = "/home/g1/Developer/RISE_Lab/colcon_ws/experiments/June-25/Test_2.csv"
    print(f"\nAnalyzing:\n{filename}\n")

    data = pd.read_csv(filename)
    if data.empty:
        raise ValueError("Data could not be read. Check file path or format.")

    time = data.iloc[:, TIME_COLUMN].values

    # Optionally, slice the data to focus on a specific time range
    start_time_sec = 10
    if time[-1] >= start_time_sec:
        print(f"Slicing data to start from {start_time_sec} seconds.")
        start_index = np.argmax(time >= start_time_sec)
        time = time[start_index:]
        data = data.iloc[start_index:].reset_index(drop=True)
    else:
        print(f"Warning: Total experiment duration is less than {start_time_sec}s. Plotting from beginning.")

    # Precompute any derived columns (e.g., orientation from quaternions)
    derived = {}
    try:
        # Extract quaternion data for Body 3 (columns 33 to 36)
        quat_body3 = data.iloc[:, 33:37].astype(float).values
        qx = quat_body3[:, 0]
        qy = quat_body3[:, 1]
        qz = quat_body3[:, 2]
        qw = quat_body3[:, 3]

        derived["yaw_body3"] = quaternion_to_yaw(qx, qy, qz, qw)
        derived["pitch_body3"] = quaternion_to_pitch(qx, qy, qz, qw)
        derived["roll_body3"] = quaternion_to_roll(qx, qy, qz, qw)

    except Exception as e:
        print(f"Could not calculate orientation from quaternions: {e}")
        derived["yaw_body3"] = None
        derived["pitch_body3"] = None
        derived["roll_body3"] = None

    headers = list(data.columns)
    base_title = os.path.basename(filename)

    # --- Create the plot windows ---
    plt.style.use('seaborn-v0_8-whitegrid')

    # Window 1: Mocap Data (2D Time Series)
    create_plot_window(
        fig_num=1,
        plot_configs=MOCAP_PLOT_CONFIG,
        data=data,
        derived=derived,
        time=time,
        headers=headers,
        window_title=f"Mocap Data Analysis (Time Series): {base_title}"
    )

    # Window 2: Sensor and Control Data (2D Time Series)
    create_plot_window(
        fig_num=2,
        plot_configs=SENSOR_CONTROL_CONFIG,
        data=data,
        derived=derived,
        time=time,
        headers=headers,
        window_title=f"Sensor & Control Data: {base_title}"
    )

    # Window 3: Mocap Position (3D Trajectory)
    create_3d_mocap_plot(
        fig_num=3,
        data=data,
        window_title=f"Mocap 3D Trajectory (Body 3): {base_title}"
    )

    plt.show()

if __name__ == "__main__":
    main()
