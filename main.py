"""
Simplified Soft Robot Pressure Control System

Configuration: Modify the variables below
Run with: python main.py

Optional: For AI auto-descriptions, add to .env file:
  GEMINI_API_KEY=your-api-key-here
"""

import datetime
import logging
import math
import os
import signal
import socket
import struct
import sys
import threading
import time

try:
    from dotenv import load_dotenv

    load_dotenv()  # Load environment variables from .env file
except ImportError:
    print("Warning: python-dotenv not installed, proceeding without it.")
    pass  # dotenv not required, will use system env vars

try:
    from google import genai

    GEMINI_AVAILABLE = True
    print("Google Generative AI module loaded. Auto-descriptions enabled.")
except ImportError:
    GEMINI_AVAILABLE = False
    print("Warning: google-genai not available. Auto-descriptions disabled.")

try:
    import numpy as np
    import zmq

    MOCAP_AVAILABLE = True
except ImportError:
    MOCAP_AVAILABLE = False
    print("Warning: ZMQ/numpy not available. Mocap disabled.")

try:
    import h5py

    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False
    print("ERROR: h5py not installed. Install with: pip install h5py")
    sys.exit(1)

# ============================================================================
# CONFIGURATION - Edit these values
# ============================================================================

ARDUINO_IDS = [3, 6, 7, 8]
TARGET_PRESSURES = [5.0, 5.0, 5.0, 5.0]
PC_ADDRESS = "10.211.215.251"

# Experiment settings
EXPERIMENT_DURATION = 120.0
END_AFTER_ONE_CYCLE = True

# Graceful exit settings
RAMPDOWN_DURATION = 5.0  # seconds to ramp down all pressures to zero

# Mocap settings
USE_MOCAP = True
MOCAP_PORT = "tcp://127.0.0.1:3885"
MOCAP_DATA_SIZE = 21

# Gemini API settings
USE_GEMINI_AUTO_DESCRIPTION = True
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "")  # Set via environment variable
WAVE_FUNCTION = "axial"
# WAVE_FUNCTION = "circular"


# ============================================================================

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(message)s")
logger = logging.getLogger(__name__)

ARDUINO_PORTS = [10001, 10002, 10003, 10004, 10005, 10006, 10007, 10008]


class ArduinoManager:
    def __init__(self):
        self.server_sockets = []
        self.client_sockets = []

    def connect(self):
        for arduino_id in ARDUINO_IDS:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((PC_ADDRESS, ARDUINO_PORTS[arduino_id - 1]))
            sock.listen(1)
            logger.info(f"Waiting for Arduino {arduino_id}...")
            self.server_sockets.append(sock)

            client, addr = sock.accept()
            self.client_sockets.append(client)
            logger.info(f"Arduino {arduino_id} connected")
        return True

    def send_pressure(self, idx, pressure):
        try:
            self.client_sockets[idx].send(struct.pack("f", pressure))
            data = b""
            while len(data) < 8:
                data += self.client_sockets[idx].recv(8 - len(data))

            sensors = []
            for val in struct.unpack(">4h", data):
                volt = val * (12.288 / 65536.0)
                sensors.append(round((30.0 * (volt - 0.5)) / 4.0, 4))
            return sensors
        except:
            return [0.0] * 4

    def cleanup(self):
        for sock in self.client_sockets + self.server_sockets:
            try:
                sock.close()
            except:
                pass


class MocapManager:
    def __init__(self):
        self.data = [0.0] * MOCAP_DATA_SIZE
        self.running = False
        self.socket = None
        self.thread = None

    def connect(self):
        if not MOCAP_AVAILABLE:
            return False
        try:
            ctx = zmq.Context()
            self.socket = ctx.socket(zmq.SUB)
            self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
            self.socket.setsockopt(zmq.CONFLATE, True)
            self.socket.connect(MOCAP_PORT)
            logger.info("Mocap connected")
            return True
        except:
            return False

    def start(self):
        if not self.socket:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        last_data = None
        while self.running:
            try:
                msg = self.socket.recv(zmq.NOBLOCK)
                arr = np.fromstring(msg.decode("utf-8"), dtype=float, sep=",")
                if len(arr) >= MOCAP_DATA_SIZE:
                    self.data = arr[:MOCAP_DATA_SIZE].tolist()
                    last_data = self.data.copy()
            except:
                if last_data:
                    self.data = last_data.copy()
            time.sleep(0.01)

    def get_data(self):
        return self.data.copy()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)


class DataLogger:
    def __init__(self):
        self.hdf5_path = None
        self.exp_group_name = None
        self.start_time = None
        self.data_buffer = []
        self.lock = threading.Lock()

    def generate_ai_description(self) -> str:
        """Generate a brief experiment description using Gemini Flash"""
        if (
            not GEMINI_AVAILABLE
            or not USE_GEMINI_AUTO_DESCRIPTION
            or not GEMINI_API_KEY
        ):
            return "No description provided"

        try:
            # No need to import again, it's done at the top level
            client = genai.Client()
            model = "gemini-2.5-flash"
            # Create a concise prompt with experiment details
            prompt = f"""
                        Generate a 1-sentence experiment description (max 15 words) for:
                        Type: {WAVE_FUNCTION}
                        Duration: {int(EXPERIMENT_DURATION)}s
                        Arduinos: {ARDUINO_IDS}
                        Pressures: {TARGET_PRESSURES} PSI
                        MoCap: {"Yes" if USE_MOCAP and MOCAP_AVAILABLE else "No"}
                    """

            response = client.models.generate_content(model=model, contents=prompt)

            description = response.text.strip()
            logger.info(f"AI description: {description}")
            return description

        except Exception as e:
            logger.warning(f"Failed to generate AI description: {e}")
            return "Auto-description failed"

    def start(self):
        now = datetime.datetime.now()
        folder = "experiments"
        os.makedirs(folder, exist_ok=True)

        # Create monthly HDF5 file
        month_file = now.strftime("%Y_%B.h5")
        self.hdf5_path = os.path.join(folder, month_file)

        # Determine experiment number
        exp_num = 1
        if os.path.exists(self.hdf5_path):
            with h5py.File(self.hdf5_path, "r") as f:
                existing = [k for k in f.keys() if k.startswith("exp_")]
                if existing:
                    # Extract numbers from exp names
                    numbers = []
                    for exp_name in existing:
                        try:
                            # Split by underscore and get the number after exp_
                            parts = exp_name.split("_")
                            if len(parts) >= 2:
                                num = int(parts[1])
                                numbers.append(num)
                        except (ValueError, IndexError):
                            continue
                    if numbers:
                        exp_num = max(numbers) + 1

        # Create experiment name with human-readable date/time
        # Format: exp_001_axial_Oct30_14h23m
        date_str = now.strftime("%b%d_%Hh%Mm")
        self.exp_group_name = f"exp_{exp_num:03d}_{WAVE_FUNCTION}_{date_str}"
        self.start_time = time.time()

        logger.info(
            f"Logging to: {os.path.abspath(self.hdf5_path)} -> {self.exp_group_name}"
        )

    def log(self, desired, measured, mocap=None):
        if not self.hdf5_path:
            return

        # Build data row
        row = [time.time() - self.start_time]
        row.extend(desired)
        for sensors in measured:
            row.extend(sensors)
        if mocap:
            row.extend(mocap)

        with self.lock:
            self.data_buffer.append(row)

    def stop(self, description=None):
        if not self.hdf5_path or not self.data_buffer:
            return

        # Convert buffer to numpy array
        data_array = np.array(self.data_buffer)

        # Create column names
        col_names = ["time"]
        for aid in ARDUINO_IDS:
            col_names.append(f"pd_{aid}")
        for aid in ARDUINO_IDS:
            for s in range(1, 5):
                col_names.append(f"pm_{aid}_{s}")

        if USE_MOCAP and MOCAP_AVAILABLE:
            for body_num in range(1, 4):
                col_names.append(f"mocap_{body_num}_x")
                col_names.append(f"mocap_{body_num}_y")
                col_names.append(f"mocap_{body_num}_z")
                col_names.append(f"mocap_{body_num}_qx")
                col_names.append(f"mocap_{body_num}_qy")
                col_names.append(f"mocap_{body_num}_qz")
                col_names.append(f"mocap_{body_num}_qw")

        # Save to HDF5
        try:
            with h5py.File(self.hdf5_path, "a") as f:
                grp = f.create_group(self.exp_group_name)

                # Save data
                grp.create_dataset(
                    "data", data=data_array, compression="gzip", compression_opts=4
                )

                # Save column names as attribute
                grp.attrs["columns"] = col_names

                # Save metadata - Important experiment parameters
                grp.attrs["timestamp"] = datetime.datetime.now().isoformat()
                grp.attrs["experiment_type"] = WAVE_FUNCTION
                grp.attrs["duration_seconds"] = EXPERIMENT_DURATION
                grp.attrs["mocap_enabled"] = USE_MOCAP and MOCAP_AVAILABLE
                grp.attrs["arduino_ids"] = ARDUINO_IDS
                grp.attrs["target_pressures_psi"] = TARGET_PRESSURES
                grp.attrs["end_after_one_cycle"] = END_AFTER_ONE_CYCLE
                grp.attrs["rampdown_duration_seconds"] = RAMPDOWN_DURATION
                grp.attrs["sample_count"] = len(self.data_buffer)
                grp.attrs["pc_address"] = PC_ADDRESS

                if USE_MOCAP and MOCAP_AVAILABLE:
                    grp.attrs["mocap_port"] = MOCAP_PORT
                    grp.attrs["mocap_data_size"] = MOCAP_DATA_SIZE

                # Use provided description
                if description:
                    grp.attrs["description"] = description
                else:
                    # Fallback if no description is provided
                    grp.attrs["description"] = "No description provided"

                logger.info(
                    f"Saved {len(self.data_buffer)} samples to {self.exp_group_name}"
                )
        except Exception as e:
            logger.error(f"Error saving HDF5: {e}")


# ============================================================================
# WAVE FUNCTIONS
# ============================================================================


def circular(controller):
    """Circular motion with first Arduino at 2.0 psi"""

    # Wave parameters - adjust as needed
    WAVE_FREQ = 0.1  # Hz
    WAVE_CENTER = 1.0  # psi
    WAVE_AMPLITUDE = 2.0  # psi

    phases = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]

    controller.send_all()
    time.sleep(5)

    start = time.time()
    while controller.running:
        t = time.time() - start
        for i in range(0, len(ARDUINO_IDS)):
            phi = phases[(i - 1) % 4]
            p = WAVE_CENTER + WAVE_AMPLITUDE * math.sin(
                2 * math.pi * WAVE_FREQ * t + phi
            )
            controller.desired[i] = max(0, min(100, p))
        controller.send_all()
        time.sleep(0.01)


def axial(controller):
    WAVE_FREQ = 0.1  # Hz (controls speed of oscillation)
    WAVE_CENTER = 5.0  # psi (center of 0-10 psi range)
    WAVE_AMPLITUDE = 3.0  # psi (amplitude for 0-10 psi range)

    # Arduino 3 (index 0)
    controller.desired[0] = 2.0
    # Arduino 6 (index 1)
    controller.desired[1] = 2.0
    # Arduino 7 (index 2) - Start at 0 psi (bottom of its wave)
    controller.desired[2] = WAVE_CENTER - WAVE_AMPLITUDE
    # Arduino 8 (index 3)
    controller.desired[3] = 2.0

    # Send all initial pressures
    controller.send_all()
    time.sleep(5)  # Pause for 0.5s after setting initial state

    # --- Main Loop ---
    start = time.time()
    while controller.running:
        controller.desired[1] = 2.0  # Arduino 6
        controller.desired[3] = 2.0  # Arduino 8

        t = time.time() - start  # Get elapsed time

        p_arduino7 = WAVE_CENTER + WAVE_AMPLITUDE * math.sin(
            2 * math.pi * WAVE_FREQ * t
        )

        # Set desired pressure, clamping between 0 and 100 as a safety
        # (same as the original function's clamping)
        controller.desired[2] = max(0.0, min(10.0, p_arduino7))

        # 3. Send all updated pressures to the controller
        controller.send_all()

        # 4. Short pause before next update
        time.sleep(0.1)


def sequential(controller):
    """Sequential: Arduino 4 constant, Arduino 8 cycles"""
    try:
        idx_4 = ARDUINO_IDS.index(4)
        idx_8 = ARDUINO_IDS.index(8)
    except ValueError:
        logger.error("Sequential needs Arduinos 4 and 8")
        return

    # Ramp up Arduino 4
    for i in range(len(ARDUINO_IDS)):
        controller.desired[i] = 0.0

    target = TARGET_PRESSURES[idx_4]
    for t in range(int(target * 20)):
        controller.desired[idx_4] = min(target, t / 20.0)
        controller.send_all()
        time.sleep(0.01)

    # Cycle Arduino 8
    while controller.running:
        target = TARGET_PRESSURES[idx_8]

        # Ramp up
        for t in range(int(target * 20)):
            controller.desired[idx_8] = min(target, t / 20.0)
            controller.send_all()
            time.sleep(0.01)

        time.sleep(2.0)

        # Ramp down
        for t in range(int(target * 20)):
            controller.desired[idx_8] = max(0, target - t / 20.0)
            controller.send_all()
            time.sleep(0.01)

        time.sleep(2.0)

        if END_AFTER_ONE_CYCLE:
            break


def elliptical(controller):
    """Elliptical motion"""
    WAVE_FREQ = 0.1
    WAVE_CENTER = 3.0
    WAVE_AMPLITUDE = 2.0

    phases = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]

    start = time.time()
    while controller.running:
        t = time.time() - start
        for i in range(len(ARDUINO_IDS)):
            amp = WAVE_AMPLITUDE if i % 2 == 0 else WAVE_AMPLITUDE * 0.5
            phi = phases[i % 4]
            p = WAVE_CENTER + amp * math.sin(2 * math.pi * WAVE_FREQ * t + phi)
            controller.desired[i] = max(0, min(100, p))
        controller.send_all()
        time.sleep(0.01)


WAVES = {
    "circular": circular,
    "sequential": sequential,
    "elliptical": elliptical,
    "axial": axial,
}


# ============================================================================
# MAIN CONTROLLER
# ============================================================================


class Controller:
    def __init__(self):
        self.arduino = ArduinoManager()
        self.mocap = MocapManager() if USE_MOCAP else None
        self.logger = DataLogger()
        self.running = False
        self.desired = [0.0] * len(ARDUINO_IDS)
        self.measured = [[0.0] * 4 for _ in ARDUINO_IDS]

    def initialize(self):
        if not self.arduino.connect():
            return False
        if self.mocap:
            self.mocap.connect()
        return True

    def send_all(self):
        for i in range(len(ARDUINO_IDS)):
            self.measured[i] = self.arduino.send_pressure(i, self.desired[i])

    def graceful_rampdown(self):
        """Smoothly ramp down all pressures to zero"""
        logger.info(f"Ramping down pressures over {RAMPDOWN_DURATION}s...")

        # Capture current pressures as starting point
        start_pressures = self.desired.copy()

        # Calculate ramp steps
        steps = int(RAMPDOWN_DURATION / 0.01)  # 10ms per step

        for step in range(steps + 1):
            if not self.running:
                break

            # Linear interpolation from current pressure to 0
            progress = step / steps
            for i in range(len(ARDUINO_IDS)):
                self.desired[i] = start_pressures[i] * (1.0 - progress)

            self.send_all()
            time.sleep(0.01)

        # Ensure all pressures are exactly zero
        for i in range(len(ARDUINO_IDS)):
            self.desired[i] = 0.0
        self.send_all()

        logger.info("Rampdown complete - all pressures at 0 psi")

    def run(self):
        self.logger.start()
        if self.mocap:
            self.mocap.start()

        self.running = True

        # Start logging thread
        log_thread = threading.Thread(target=self._log_loop, daemon=True)
        log_thread.start()

        # Start wave thread
        wave_thread = threading.Thread(target=self._wave_loop, daemon=True)
        wave_thread.start()

        # Start progress indicator thread
        progress_thread = threading.Thread(target=self._progress_loop, daemon=True)
        progress_thread.start()

        logger.info("Experiment started")

        # Wait
        start = time.time()
        while self.running and (time.time() - start) < EXPERIMENT_DURATION:
            time.sleep(0.1)

        self.stop()

    def _log_loop(self):
        while self.running:
            mocap = self.mocap.get_data() if self.mocap else None
            self.logger.log(self.desired, self.measured, mocap)
            time.sleep(0.01)

    def _wave_loop(self):
        logger.info(f"Running: {WAVE_FUNCTION}")
        WAVES[WAVE_FUNCTION](self)
        if END_AFTER_ONE_CYCLE:
            self.running = False

    def _progress_loop(self):
        """Display experiment progress and live sensor readings"""
        start = time.time()
        total_duration_str = f"{int(EXPERIMENT_DURATION)}s"

        while self.running:
            elapsed = time.time() - start

            # Format time in seconds
            elapsed_str = f"{int(elapsed)}s"

            # Calculate percentage
            progress = min(100, (elapsed / EXPERIMENT_DURATION) * 100)

            # Progress bar
            bar_length = 30
            filled = int(bar_length * progress / 100)
            bar = "█" * filled + "░" * (bar_length - filled)

            # Print progress (overwrite same line)
            print(
                f"\r[{bar}] {progress:.1f}% | {elapsed_str}/{total_duration_str}",
                end="",
                flush=True,
            )

            time.sleep(1.0)

        # Print newline when done
        print()

    def stop(self):
        # Signal threads to stop
        self.running = False

        # Give wave thread a moment to exit cleanly
        time.sleep(0.1)

        # Gracefully ramp down all pressures
        self.graceful_rampdown()

        # Stop mocap
        if self.mocap:
            self.mocap.stop()

        # Prompt for description or use AI
        description = None
        has_api_key = GEMINI_API_KEY and len(GEMINI_API_KEY) > 0

        if USE_GEMINI_AUTO_DESCRIPTION and GEMINI_AVAILABLE and has_api_key:
            # AI is available - generate description and ask for additions
            print("\n" + "=" * 60)
            print("Generating AI description...")
            ai_description = self.logger.generate_ai_description()
            description = ai_description  # Default to AI description

            try:
                additional_desc = input(
                    "Enter additional description (optional, press Enter to skip): "
                ).strip()
                if additional_desc:
                    # Combine descriptions
                    description = f"{ai_description}. {additional_desc}"
            except (EOFError, KeyboardInterrupt):
                print("\nSkipping additional description.")
                # `description` is already set to ai_description, so we're good

        else:
            # No AI available or configured, ask for manual description
            try:
                print("\n" + "=" * 60)
                if not has_api_key and USE_GEMINI_AUTO_DESCRIPTION:
                    print("AI descriptions enabled, but GEMINI_API_KEY is not set.")

                description = input(
                    "Enter experiment description (or press Enter to skip): "
                ).strip()
                if not description:
                    description = "No description provided"
            except (EOFError, KeyboardInterrupt):
                description = "Interrupted - no description"

        self.logger.stop(description)
        self.arduino.cleanup()


def main():
    signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))

    logger.info("=" * 50)
    logger.info(f"Arduinos: {ARDUINO_IDS}")
    logger.info(f"Pressures: {TARGET_PRESSURES} psi")
    logger.info(f"Wave: {WAVE_FUNCTION}")
    logger.info(f"Mocap: {'ON' if USE_MOCAP and MOCAP_AVAILABLE else 'OFF'}")
    logger.info(
        f"AI Descriptions: {'ON' if USE_GEMINI_AUTO_DESCRIPTION and GEMINI_AVAILABLE and GEMINI_API_KEY else 'OFF'}"
    )
    logger.info(f"Rampdown: {RAMPDOWN_DURATION}s")
    logger.info("=" * 50)

    controller = Controller()

    try:
        if not controller.initialize():
            return 1
        controller.run()
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1

    logger.info("Done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
