import os
import glob
import pandas as pd
import numpy as np
from sklearn.linear_model import Ridge
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')

def load_data(data_dir):
    """Loads only parallel data files (best case dataset)."""
    # Only load files from the parallel folder as it yields higher-dimensional pressure state
    files = glob.glob(os.path.join(data_dir, 'parallel', '*.csv'), recursive=True)
    if not files:
        return pd.DataFrame()
        
    dfs = []
    for f in files:
        df = pd.read_csv(f)
        # Drop initial and final 10 seconds to remove transients as described in the thesis
        if 'time' in df.columns:
            t_max = df['time'].max()
            df = df[(df['time'] >= 10.0) & (df['time'] <= t_max - 10.0)]
        dfs.append(df)
    return pd.concat(dfs, ignore_index=True)

def prepare_features(df, history_length=5):
    """
    Prepares input features X and target Y.
    To handle the nonlinear 'leakage' (memory) issue, we use a tapped-delay line of the kinematics
    and the previous pressure states from segment 1, mimicking a physical reservoir computing approach.
    """
    df = df.sort_values(by=['time']).reset_index(drop=True)
    
    # Target: Desired pressures for actuation segments 2-4
    target_cols = ['Desired_pressure_segment_2', 'Desired_pressure_segment_3', 'Desired_pressure_segment_4']
    
    # Base features: End-effector kinematics and Segment 1 pressures (acting as the 'memory' reservoir)
    kinematic_cols = ['Rigid_body_3_x', 'Rigid_body_3_y', 'Rigid_body_3_z',
                      'Rigid_body_3_qx', 'Rigid_body_3_qy', 'Rigid_body_3_qz', 'Rigid_body_3_qw']
                      
    pressure_cols = ['Measured_pressure_Segment_1_pouch_1', 'Measured_pressure_Segment_1_pouch_2',
                     'Measured_pressure_Segment_1_pouch_3', 'Measured_pressure_Segment_1_pouch_4',
                     'Measured_pressure_Segment_1_pouch_5']
    
    # Use both kinematics and reservoir state available
    feature_cols = kinematic_cols + pressure_cols
    
    # Clean dropping NaNs
    df = df.dropna(subset=target_cols + feature_cols).reset_index(drop=True)
    
    X_list = []
    Y_list = []
    
    # Constructing tapped delay line features
    feature_data = df[feature_cols].values
    target_data = df[target_cols].values
    
    for i in range(history_length, len(df)):
        # Flatten history window to vector
        history_window = feature_data[i-history_length:i].flatten()
        X_list.append(history_window)
        Y_list.append(target_data[i])
        
    X = np.array(X_list)
    Y = np.array(Y_list)
    return X, Y

def main():
    print("Loading data...")
    # Adjust path assuming script is run from project root
    data_dir = "data/_data_extract"
    df = load_data(data_dir)
    
    if df.empty:
        print("No data found!")
        return
        
    print(f"Data shape: {df.shape}")
    
    print("Preparing features with history to handle leakage/memory...")
    # We use a history context of 20 steps to account for pressure leakage/hysteresis dynamics (n=20 as in paper)
    history_length = 20 
    X, Y = prepare_features(df, history_length=history_length)
    
    X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.1, random_state=42)
    print(f"Train size: {X_train.shape[0]}, Test size: {X_test.shape[0]}")
    
    print("Training controller with a little bit of 'leakage' (Ridge Regression)...")
    # Ridge regression explicitly introduces weight "leakage" (L2 regularization) 
    # alpha=0.01 directly matches the paper's linear readout for PRC prediction
    controller = Ridge(alpha=0.01)
    
    controller.fit(X_train, Y_train)
    
    print("Evaluating controller...")
    predictions = controller.predict(X_test)
    mse = mean_squared_error(Y_test, predictions)
    print(f"Controller Test MSE: {mse:.4f}")
    
    print("Generating correlation plots...")
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    segments = ['Segment 2', 'Segment 3', 'Segment 4']
    
    for i, ax in enumerate(axes):
        ax.scatter(Y_test[:, i], predictions[:, i], alpha=0.1, color='royalblue', s=2)
        
        # Perfect prediction line
        p_min = min(Y_test[:, i].min(), predictions[:, i].min())
        p_max = max(Y_test[:, i].max(), predictions[:, i].max())
        ax.plot([p_min, p_max], [p_min, p_max], 'k--', lw=2, label='Perfect Prediction')
        
        ax.set_title(f'True vs Predicted ({segments[i]})')
        ax.set_xlabel('True Pressure (PSI)')
        ax.set_ylabel('Predicted Pressure (PSI)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
