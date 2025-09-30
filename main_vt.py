"""
PC Client for Soft Robot Control System

This module provides the main entry point for controlling a soft robot system
with motion capture integration and pressure control.
"""

import logging
import signal
import sys
from time import sleep
from typing import Optional

import vtech


# Configuration Constants
class Config:
    """Configuration constants for the control system"""

    POSITION_PROFILE_FLAG = 3
    USE_MOCAP = True
    TRIAL_DURATION = 600.0  # seconds
    THREAD_START_DELAY = 0.5  # seconds between thread starts


# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class SoftRobotController:
    """Main controller for the soft robot system"""

    def __init__(self):
        self.client: Optional[vtech.pc_client] = None
        self.is_running = False

    def initialize_client(self) -> bool:
        """Initialize the pressure control client

        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            logger.info("Initializing pressure control client...")
            self.client = vtech.pc_client()

            # Configure client settings
            self.client.positionProfile_flag = Config.POSITION_PROFILE_FLAG
            self.client.flag_use_mocap = 1 if Config.USE_MOCAP else 0
            self.client.trailDuriation = Config.TRIAL_DURATION

            logger.info("Client initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize client: {e}")
            return False

    def start_control_threads(self) -> bool:
        """Start the control system threads

        Returns:
            bool: True if threads started successfully, False otherwise
        """
        if not self.client:
            logger.error("Client not initialized")
            return False

        try:
            logger.info("Starting control threads...")

            # Start threads with delays to ensure proper initialization
            self.client.th2.start()  # Motion capture thread
            sleep(Config.THREAD_START_DELAY)

            self.client.th1.start()  # Pressure generation thread
            sleep(Config.THREAD_START_DELAY)

            self.client.th3.start()  # Data exchange thread
            sleep(Config.THREAD_START_DELAY)

            self.is_running = True
            logger.info("All control threads started successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to start threads: {e}")
            return False

    def run(self):
        """Main control loop"""
        if not self.is_running:
            logger.error("Control system not properly initialized")
            return

        logger.info("Control system running. Press Ctrl+C to stop.")
        try:
            while self.is_running:
                sleep(0.1)  # Small delay to prevent excessive CPU usage

        except KeyboardInterrupt:
            logger.info("Shutdown signal received")
            self.shutdown()

    def shutdown(self):
        """Gracefully shutdown the control system"""
        logger.info("Shutting down control system...")

        if self.client:
            try:
                # Stop threads gracefully
                self.client.th1_flag = False
                self.client.th2_flag = False
                self.is_running = False

                logger.info("Control system shutdown complete")

            except Exception as e:
                logger.error(f"Error during shutdown: {e}")


def signal_handler(signum, frame):
    """Handle system signals for graceful shutdown"""
    logger.info(f"Received signal {signum}")
    sys.exit(0)


def main():
    """Main entry point"""
    # Setup signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info("Starting Soft Robot Control System")

    # Create and initialize controller
    controller = SoftRobotController()

    if not controller.initialize_client():
        logger.error("Failed to initialize controller")
        sys.exit(1)

    if not controller.start_control_threads():
        logger.error("Failed to start control threads")
        sys.exit(1)

    # Run the control system
    try:
        controller.run()
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        controller.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
