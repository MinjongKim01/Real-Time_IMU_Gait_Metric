"""Device manager for Movella DOT sensors."""

import sys
import movelladot_pc_sdk
from typing import List, Optional
from logging_config import get_logger
from config_loader import get_config

logger = get_logger(__name__)
config = get_config()


class DeviceManager:
    """Manage Movella DOT device lifecycle: discovery, connection, configuration, synchronization."""

    def __init__(self, xdpc_handler, sensor_mapping: dict):
        """
        Initialize device manager.

        Args:
            xdpc_handler: XdpcHandler instance for device communication
            sensor_mapping: Dict mapping MAC addresses to sensor roles
        """
        self.handler = xdpc_handler
        self.sensor_mapping = sensor_mapping
        self.connected_devices: List = []
        self.manager: Optional[movelladot_pc_sdk.XsDotConnectionManager] = None

    def discover_and_connect(self) -> bool:
        """
        Full device setup: scan, connect, and configure devices.

        Returns:
            True if successful, False otherwise
        """
        # Initialize SDK
        if not self.handler.initialize():
            self.handler.cleanup()
            return False

        # Scan for devices
        if not self._scan_for_devices():
            self.handler.cleanup()
            return False

        # Connect to devices
        if not self._connect_to_devices():
            self.handler.cleanup()
            return False

        # Configure devices
        if not self._configure_devices():
            self.handler.cleanup()
            return False

        return True

    def _scan_for_devices(self) -> bool:
        """
        Scan for Movella DOT devices.

        Returns:
            True if devices found, False otherwise
        """
        logger.info("Scanning for DOTs...")
        self.handler.scanForDots()

        if len(self.handler.detectedDots()) == 0:
            logger.error("No device(s) found. Aborting.")
            return False

        return True

    def _connect_to_devices(self) -> bool:
        """
        Connect to devices specified in sensor mapping.

        Returns:
            True if all devices connected successfully, False otherwise
        """
        logger.info("Connecting to DOTs...")
        self.handler.connectDots()

        all_devices = self.handler.connectedDots()
        if not all_devices:
            logger.error("Could not connect to any devices. Aborting.")
            return False

        # Filter devices based on sensor mapping
        for dev in all_devices:
            if dev.bluetoothAddress() in self.sensor_mapping:
                self.connected_devices.append(dev)
                logger.info(f"Found device: {dev.bluetoothAddress()} ({self.sensor_mapping[dev.bluetoothAddress()]})")

        if len(self.connected_devices) != len(self.sensor_mapping):
            logger.error("Could not find all devices specified in SENSOR_MAPPING. Aborting.")
            return False

        return True

    def _configure_devices(self) -> bool:
        """
        Configure device settings (filter profile, output rate).

        Returns:
            True if configuration successful, False otherwise
        """
        filter_profile = config.get('sensors.filter_profile', 'General')
        sampling_rate = config.get('sensors.sampling_rate', 60)

        logger.info(f"Setting up {len(self.connected_devices)} device(s)...")

        for device in self.connected_devices:
            role = self.sensor_mapping[device.bluetoothAddress()]

            # Set filter profile
            if device.setOnboardFilterProfile(filter_profile):
                logger.info(f"[{role}] Set profile to {filter_profile}")
            else:
                logger.error(f"[{role}] Setting profile FAILED!")

            # Set output rate
            if device.setOutputRate(sampling_rate):
                logger.info(f"[{role}] Set output rate to {sampling_rate} Hz")
            else:
                logger.error(f"[{role}] Setting output rate FAILED!")

        return True

    def synchronize_devices(self) -> bool:
        """
        Synchronize devices using the last device as root node.

        Returns:
            True if synchronization successful, False otherwise
        """
        self.manager = self.handler.manager()

        logger.info(f"Starting sync... Root node: {self.connected_devices[-1].bluetoothAddress()}")

        if not self.manager.startSync(self.connected_devices[-1].bluetoothAddress()):
            logger.warning(f"Sync failed. Reason: {self.manager.lastResultText()}. Retrying after stop...")
            self.manager.stopSync()

            if not self.manager.startSync(self.connected_devices[-1].bluetoothAddress()):
                logger.error(f"Sync retry failed. Reason: {self.manager.lastResultText()}. Aborting.")
                return False

        return True

    def start_measurement(self) -> bool:
        """
        Start measurement mode on all connected devices.

        Returns:
            True if measurement started successfully, False otherwise
        """
        logger.info("Putting devices into measurement mode.")

        for device in self.connected_devices:
            payload_mode = movelladot_pc_sdk.XsPayloadMode_CustomMode5
            if not device.startMeasurement(payload_mode):
                role = self.sensor_mapping.get(device.bluetoothAddress(), "Unknown")
                logger.error(f"[{role}] Could not start measurement. Reason: {device.lastResultText()}")
                return False

        return True

    def stop_measurement(self) -> None:
        """Stop measurement on all connected devices."""
        logger.info("Stopping measurement...")
        for device in self.connected_devices:
            if not device.stopMeasurement():
                logger.error("Failed to stop measurement.")

    def stop_sync(self) -> None:
        """Stop device synchronization."""
        if self.manager:
            logger.info("Stopping sync...")
            if not self.manager.stopSync():
                logger.error("Failed to stop sync.")

    def cleanup(self) -> None:
        """Clean up device connections."""
        self.handler.cleanup()
        logger.info("Device manager cleanup complete.")

    def get_connected_devices(self) -> List:
        """
        Get list of connected devices.

        Returns:
            List of connected device objects
        """
        return self.connected_devices
