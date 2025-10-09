import os

import pandas as pd

# ---- CONFIGURATION ----

OUTPUT_FOLDER_NAME = "cleaned_data"
INPUT_FODLER_NAME = "experiments/cleaned_data/"
# This dictionary defines all the renaming operations.
# 'old_name': 'new_name'
COLUMN_RENAME_MAP = {
    "pd_4": "Desired_pressure_segment_1_2",
    "pd_7": "Desired_pressure_segment_3",
    "pd_8": "Desired_pressure_segment_4",
    "pm_4_1": "Measured_pressure_Segment_4",
    "pm_4_2": "Measured_pressure_segment_3",
    "pm_4_3": "Measured_pressure_Segment_1_pouch_1",
    "pm_4_4": "Measured_pressure_Segment_1_pouch_2",
    "pm_7_1": "Measured_pressure_Segment_1_pouch_3",
    "pm_7_2": "Measured_pressure_Segment_1_pouch_4",
    "pm_7_3": "Measured_pressure_Segment_1_pouch_5",
    "pm_7_4": "Measured_pressure_Segment_2_pouch_1",
    "pm_8_1": "Measured_pressure_Segment_2_pouch_2",
    "pm_8_2": "Measured_pressure_Segment_2_pouch_3",
    "pm_8_3": "Measured_pressure_Segment_2_pouch_4",
    "pm_8_4": "Measured_pressure_Segment_2_pouch_5",
    # Rename mocap3 columns to the new generic rigid body names
    "mocap3_x": "mocap_rigid_body_x",
    "mocap3_y": "mocap_rigid_body_y",
    "mocap3_z": "mocap_rigid_body_z",
    "mocap3_qx": "mocap_rigid_body_qx",
    "mocap3_qy": "mocap_rigid_body_qy",
    "mocap3_qz": "mocap_rigid_body_qz",
    "mocap3_qw": "mocap_rigid_body_qw",
}

# This list contains all columns that should be completely removed.
COLUMNS_TO_DROP = [
    "mocap1_x",
    "mocap1_y",
    "mocap1_z",
    "mocap1_qx",
    "mocap1_qy",
    "mocap1_qz",
    "mocap1_qw",
    "mocap2_x",
    "mocap2_y",
    "mocap2_z",
    "mocap2_qx",
    "mocap2_qy",
    "mocap2_qz",
    "mocap2_qw",
]

# ---- END CONFIGURATION ----


def process_and_clean_csv(input_filepath, output_dir):
    """
    Reads a single CSV, renames and drops columns, and saves it to the output directory.
    """
    try:
        # Get the base name of the file to use for messages and the output file
        base_filename = os.path.basename(input_filepath)
        print(f"--- Processing: {base_filename} ---")

        # Read the original CSV file
        df = pd.read_csv(input_filepath)

        # --- Column Transformation ---

        # 1. Drop the unwanted mocap columns
        # We check which columns actually exist in the dataframe to avoid errors.
        existing_cols_to_drop = [col for col in COLUMNS_TO_DROP if col in df.columns]
        df.drop(columns=existing_cols_to_drop, inplace=True, errors="ignore")
        print(f"Dropped {len(existing_cols_to_drop)} columns.")

        # 2. Rename the remaining columns based on the map
        df.rename(columns=COLUMN_RENAME_MAP, inplace=True)
        print("Renamed columns.")

        # --- Saving the new file ---
        output_filepath = os.path.join(output_dir, base_filename)
        df.to_csv(output_filepath, index=False)
        print(f"Successfully saved cleaned file to: {output_filepath}\n")

    except FileNotFoundError:
        print(f"Error: The file '{input_filepath}' was not found.")
    except Exception as e:
        print(f"An error occurred while processing {base_filename}: {e}")


def main():
    """
    Main function to ask for a folder path and process all CSV files within it.
    """
    # Loop until the user provides a valid directory path
    while True:
        target_folder = INPUT_FODLER_NAME
        if os.path.isdir(target_folder):
            break
        else:
            print("Invalid path. Please enter a valid folder path.")

    # --- Prepare Output Directory ---
    # Create the 'cleaned_data' directory inside the user-specified folder
    output_dir = os.path.join(target_folder, OUTPUT_FOLDER_NAME)
    os.makedirs(output_dir, exist_ok=True)
    print("-" * 50)
    print(f"Cleaned files will be saved in: {output_dir}")
    print("-" * 50)

    # --- Find and Process CSV Files ---
    csv_files = [f for f in os.listdir(target_folder) if f.lower().endswith(".csv")]

    if not csv_files:
        print(f"No CSV files found in '{target_folder}'.")
        return

    print(f"Found {len(csv_files)} CSV file(s) to process.\n")

    # Loop through each found CSV file and process it
    for filename in csv_files:
        full_input_path = os.path.join(target_folder, filename)
        process_and_clean_csv(full_input_path, output_dir)

    print("=" * 50)
    print("All files processed.")
    print("=" * 50)


if __name__ == "__main__":
    main()
