"""
Simplified Soft Robot Pressure Control System

Configuration: Modify the variables below
Run with: python main.py
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
    import numpy as np
    import zmq

    MOCAP_AVAILABLE = True
except ImportError:
    MOCAP_AVAILABLE = False
    print("Warning: ZMQ/numpy not available. Mocap disabled.")

# ============================================================================
# CONFIGURATION - Edit these values
# ============================================================================

ARDUINO_IDS = [3, 6, 7, 8]
TARGET_PRESSURES = [5.0, 5.0, 5.0, 5.0]
PC_ADDRESS = "10.211.215.251"

# Experiment settings
EXPERIMENT_DURATION = 300.0
END_AFTER_ONE_CYCLE = True

# Mocap settings
USE_MOCAP = True
MOCAP_PORT = "tcp://127.0.0.1:3885"
MOCAP_DATA_SIZE = 21

# WAVE_FUNCTION = "axial"
WAVE_FUNCTION = "circular"


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
        self.file = None
        self.start_time = None

    def start(self):
        now = datetime.datetime.now()
        folder = os.path.join("experiments", now.strftime("%B-%d"))
        os.makedirs(folder, exist_ok=True)

        num = len([f for f in os.listdir(folder) if f.startswith("Experiment_")]) + 1
        path = os.path.join(folder, f"Experiment_{num}.csv")
        self.file = open(path, "w")

        header = ["time"]
        for aid in ARDUINO_IDS:
            header.append(f"pd_{aid}")
        for aid in ARDUINO_IDS:
            for s in range(1, 5):
                header.append(f"pm_{aid}_{s}")

        if USE_MOCAP and MOCAP_AVAILABLE:
            for body_num in range(1, 4):  # 3 rigid bodies
                header.append(f"mocap_{body_num}_x")
                header.append(f"mocap_{body_num}_y")
                header.append(f"mocap_{body_num}_z")
                header.append(f"mocap_{body_num}_qx")
                header.append(f"mocap_{body_num}_qy")
                header.append(f"mocap_{body_num}_qz")
                header.append(f"mocap_{body_num}_qw")

        self.file.write(",".join(header) + "\n")
        self.start_time = time.time()
        logger.info(f"Logging to: {os.path.abspath(path)}")

    def log(self, desired, measured, mocap=None):
        if not self.file:
            return

        line = f"{time.time() - self.start_time:.6f}"
        for p in desired:
            line += f",{p:.3f}"
        for sensors in measured:
            for s in sensors:
                line += f",{s:.6f}"
        if mocap:
            for m in mocap:
                line += f",{m:.6f}"

        self.file.write(line + "\n")
        self.file.flush()

    def stop(self):
        if self.file:
            self.file.close()


# ============================================================================
# WAVE FUNCTIONS
# ============================================================================


def circular(controller):
    """Circular motion with first Arduino at 2.0 psi"""

    # Wave parameters - adjust as needed
    WAVE_FREQ = 0.1  # Hz
    WAVE_CENTER = 3.0  # psi
    WAVE_AMPLITUDE = 2.0  # psi

    phases = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]

    # controller.desired[0] = 2.0
    controller.send_all()
    time.sleep(0.5)

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
    WAVE_AMPLITUDE = 5.0  # psi (amplitude for 0-10 psi range)

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
    time.sleep(0.5)  # Pause for 0.5s after setting initial state

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
        time.sleep(0.01)


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

    def stop(self):
        self.running = False
        if self.mocap:
            self.mocap.stop()

        for i in range(len(ARDUINO_IDS)):
            self.desired[i] = 0.0
        self.send_all()

        self.logger.stop()
        self.arduino.cleanup()


def main():
    signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))

    logger.info("=" * 50)
    logger.info(f"Arduinos: {ARDUINO_IDS}")
    logger.info(f"Pressures: {TARGET_PRESSURES} psi")
    logger.info(f"Wave: {WAVE_FUNCTION}")
    logger.info(f"Mocap: {'ON' if USE_MOCAP and MOCAP_AVAILABLE else 'OFF'}")
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
