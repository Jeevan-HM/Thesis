import numpy as np
import joblib
import os
import argparse

class SoftArmController:
    """
    Nonlinear Controller for the Soft Robotic Arm.
    Accounts for pressure leakage and hysteresis by using a historical context (tapped-delay line)
    of the kinematics and the segment 1 reservoir pressures.
    """
    def __init__(self, model_path="models/nonlinear_leakage_controller.pkl"):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model not found at {model_path}. Please train the controller first.")
        self.model = joblib.load(model_path)
        # History length matching the trained model 
        # (7 kinematics + 5 pressures = 12 features per step * 10 steps = 120 features)
        self.history_length = 10
        self.feature_dim = 12 
        
    def predict_pressures(self, history_state):
        """
        Inputs:
            history_state: numpy array of shape (history_length, feature_dim)
                           Features: [Rigid_body_3_x, _y, _z, _qx, _qy, _qz, _qw,
                                      Pouch_1, Pouch_2, Pouch_3, Pouch_4, Pouch_5]
        Outputs:
            predicted_pressures: [Desired_pressure_segment_2, 3, 4]
        """
        if history_state.shape != (self.history_length, self.feature_dim):
            raise ValueError(f"Expected history_state of shape ({self.history_length}, {self.feature_dim}), got {history_state.shape}")
        
        # Flatten the history vector
        flat_input = history_state.flatten().reshape(1, -1)
        
        # Predict
        predicted_pressures = self.model.predict(flat_input)
        
        # Make sure negative pressures or unachievable extreme pressures are clipped if necessary
        return np.clip(predicted_pressures[0], a_min=0.0, a_max=30.0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test inference on trained Soft Arm Controller.")
    parser.add_argument("--model", type=str, default="models/nonlinear_leakage_controller.pkl", help="Path to the model.")
    args = parser.parse_args()
    
    print("Loading Controller...")
    controller = SoftArmController(model_path=args.model)
    
    print("Generating simulated history state for inference...")
    # Generate a dummy state: shape (10, 12)
    dummy_history = np.random.rand(10, 12)
    
    try:
        pressures = controller.predict_pressures(dummy_history)
        print("Successfully generated pressure commands:")
        print(f"Segment 2: {pressures[0]:.2f} PSI")
        print(f"Segment 3: {pressures[1]:.2f} PSI")
        print(f"Segment 4: {pressures[2]:.2f} PSI")
    except Exception as e:
        print("Error during prediction:", e)
