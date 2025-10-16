"""
PC Client for Soft Robot Control System

This module provides the main entry point for controlling a soft robot system
with motion capture integration and pressure control.

Note: Simplified version with external dependencies removed.
"""

import logging
import signal
import sys
from time import sleep
from typing import Optional

from vtech import pc_client


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
        self.client: Optional[pc_client] = None
        self.is_running = False

    def initialize_client(self) -> bool:
        """Initialize the pressure control client

        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            logger.info("Initializing pressure control client...")
            self.client = pc_client()

            if not self.client.initialize():
                logger.error("Failed to initialize client")
                return False

            logger.info("Client initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize client: {e}")
            return False

    def start_experiment(self, experiment_name: Optional[str] = None) -> bool:
        """Start the experiment

        Args:
            experiment_name: Name for the experiment

        Returns:
            bool: True if experiment started successfully
        """
        if not self.client:
            logger.error("Client not initialized")
            return False

        try:
            logger.info(f"Starting experiment: {experiment_name}")

            # Set the flag to end after one wave cycle (change to False for continuous operation)
            self.client.end_after_wave = True
            logger.info(f"End after wave flag set to: {self.client.end_after_wave}")

            if not self.client.start_experiment(experiment_name):
                logger.error("Failed to start experiment")
                return False

            self.is_running = True
            logger.info("Experiment started successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to start experiment: {e}")
            return False

    def run_experiment(self, duration: Optional[float] = None):
        """Run the experiment for specified duration

        Args:
            duration: Duration in seconds (defaults to Config.TRIAL_DURATION)
        """
        if duration is None:
            duration = Config.TRIAL_DURATION

        logger.info(f"Running experiment for {duration} seconds...")
        logger.info("All pressure control will be handled by vtech.py")

        try:
            # Let vtech.py handle all pressure control logic
            # Main just waits and monitors the experiment

            import time

            start_time = time.time()

            while self.is_running and (time.time() - start_time) < duration:
                # Check if the vtech experiment is still running
                if self.client and not self.client.is_experiment_running():
                    logger.info("Pressure execution completed - ending main loop")
                    break

                # Just wait and let vtech.py handle the pressure patterns
                sleep(0.1)  # Small sleep to prevent busy waiting

        except KeyboardInterrupt:
            logger.info("Experiment interrupted by user")
        except Exception as e:
            logger.error(f"Error during experiment: {e}")
        finally:
            self.stop_experiment()

    def stop_experiment(self):
        """Stop the current experiment"""
        self.is_running = False
        if self.client:
            self.client.stop_experiment()
            logger.info("Experiment stopped")

    def cleanup(self):
        """Clean up resources"""
        if self.client:
            self.client.cleanup()
            self.client = None
        logger.info("Controller cleanup completed")


def signal_handler(signum, frame):
    """Handle system signals for graceful shutdown"""
    logger.info(f"Received signal {signum}, shutting down...")
    sys.exit(0)


def main():
    """Main function"""
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info("Starting Soft Robot Control System...")

    controller = SoftRobotController()

    try:
        # Initialize the system
        if not controller.initialize_client():
            logger.error("Failed to initialize system")
            return 1

        # Start and run experiment (use automatic date-based folder naming)
        if (
            controller.start_experiment()
        ):  # No name = auto-generate date folder + experiment number
            controller.run_experiment()
        else:
            logger.error("Failed to start experiment")
            return 1

    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        return 1
    finally:
        controller.cleanup()

    logger.info("System shutdown complete")
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
