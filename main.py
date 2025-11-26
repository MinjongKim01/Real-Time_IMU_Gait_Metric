import sys
import time
import threading
import movelladot_pc_sdk
from xdpchandler import *
from gait_analyzer import GaitAnalyzer
from realtime_plotter import RealTimeVisualizer
from user_settings import SENSOR_MAPPING, FS
from logging_config import get_logger
from config_loader import get_config

# Initialize logger
logger = get_logger(__name__)

# Initialize config
config = get_config()

# Optimization: Define constants
MAIN_LOOP_SLEEP_SEC = config.get('performance.main_loop_sleep_sec', 0.005)  # 5ms - Balance CPU usage and responsiveness
MIN_DATA_FOR_ANALYSIS_SEC = 1.0  # Minimum data time required for analysis

def main():
    xdpcHandler = XdpcHandler()
    if not xdpcHandler.initialize():
        xdpcHandler.cleanup()
        sys.exit(-1)

    # ----- Devices Scanning -----
    logger.info("Scanning for DOTs...")
    xdpcHandler.scanForDots()
    if len(xdpcHandler.detectedDots()) == 0:
        logger.error("No device(s) found. Aborting.")
        xdpcHandler.cleanup()
        sys.exit(-1)

    # ----- Devices Connecting -----
    logger.info("Connecting to DOTs...")
    xdpcHandler.connectDots()
    connected_devices = []
    all_devices = xdpcHandler.connectedDots()

    if not all_devices:
        logger.error("Could not connect to any devices. Aborting.")
        xdpcHandler.cleanup()
        sys.exit(-1)

    for dev in all_devices:
        if dev.bluetoothAddress() in SENSOR_MAPPING:
            connected_devices.append(dev)
            logger.info(f"Found device: {dev.bluetoothAddress()} ({SENSOR_MAPPING[dev.bluetoothAddress()]})")

    if len(connected_devices) != len(SENSOR_MAPPING):
        logger.error("Could not find all devices specified in SENSOR_MAPPING. Aborting.")
        xdpcHandler.cleanup()
        sys.exit(-1)
    
    # ----- Measurement Setting -----
    filter_profile = config.get('sensors.filter_profile', 'General')
    logger.info(f"Setting up {len(connected_devices)} device(s)...")
    for device in connected_devices:
        if device.setOnboardFilterProfile(filter_profile):
            logger.info(f"[{SENSOR_MAPPING[device.bluetoothAddress()]}] Set profile to {filter_profile}")
        else:
            logger.error(f"[{SENSOR_MAPPING[device.bluetoothAddress()]}] Setting profile FAILED!")

        if device.setOutputRate(FS):
            logger.info(f"[{SENSOR_MAPPING[device.bluetoothAddress()]}] Set output rate to {FS} Hz")
        else:
            logger.error(f"[{SENSOR_MAPPING[device.bluetoothAddress()]}] Setting output rate FAILED!")

    # ----- Synchronization -----
    manager = xdpcHandler.manager()
    deviceList = connected_devices
    logger.info(f"Starting sync... Root node: {deviceList[-1].bluetoothAddress()}")
    if not manager.startSync(deviceList[-1].bluetoothAddress()):
        logger.warning(f"Sync failed. Reason: {manager.lastResultText()}. Retrying after stop...")
        manager.stopSync()
        if not manager.startSync(deviceList[-1].bluetoothAddress()):
            logger.error(f"Sync retry failed. Reason: {manager.lastResultText()}. Aborting.")
            xdpcHandler.cleanup()
            sys.exit(-1)

    # ----- Initializing -----
    logger.info("Initializing Gait Analyzer and Plotter...")
    analyzer = GaitAnalyzer(fs=FS, sensor_roles=list(SENSOR_MAPPING.values()))
    plotter = RealTimeVisualizer(sensor_roles=list(SENSOR_MAPPING.values()))
    plot_thread = threading.Thread(target=plotter.start, args=(analyzer, xdpcHandler), daemon=True)
    plot_thread.start()

    # ----- Starting Measurement -----
    logger.info("Putting devices into measurement mode.")
    for device in connected_devices:
        payload_mode = movelladot_pc_sdk.XsPayloadMode_CustomMode5
        if not device.startMeasurement(payload_mode):
            role = SENSOR_MAPPING.get(device.bluetoothAddress(), "Unknown")
            logger.error(f"[{role}] Could not start measurement. Reason: {device.lastResultText()}")

    logger.info("Main loop. Streaming data. Close the plot window to stop.")
    logger.info("-----------------------------------------")

    try:
        while not plotter.is_window_closed():
            if xdpcHandler.packetsAvailable():
                current_data_batch = {}
                for device in connected_devices:
                    packet = xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())
                    if not packet:
                        continue

                    device_id = device.bluetoothAddress()
                    device_role = SENSOR_MAPPING.get(device_id)
                    if not device_role:
                        continue

                    packet_data = {}
                    if (packet.containsSampleTimeFine() and
                       packet.containsCalibratedAcceleration() and
                       packet.containsCalibratedGyroscopeData() and
                       packet.containsOrientation()):
                        
                        packet_data['timestamp'] = packet.sampleTimeFine()
                        quat = packet.orientationQuaternion() 
                        packet_data['quaternion'] = [quat[0], quat[1], quat[2], quat[3]]
                        packet_data['acc'] = packet.calibratedAcceleration()
                        packet_data['gyr'] = packet.calibratedGyroscopeData()
                        current_data_batch[device_role] = packet_data

                if current_data_batch:
                    analyzer.process_data_batch(current_data_batch)

            # Optimization: Improve CPU usage (1ms -> 5ms)
            # 5ms sleep maintains sufficient responsiveness with 60Hz sampling rate
            time.sleep(MAIN_LOOP_SLEEP_SEC)

    except KeyboardInterrupt:
        logger.info("User interrupted. Stopping data stream...")
    except Exception as e:
        logger.error(f"An error occurred: {e}")

    # ----- Exitting -----
    finally:
        logger.info("-----------------------------------------")
        logger.info("Stopping measurement...")
        for device in connected_devices:
            if not device.stopMeasurement():
                logger.error("Failed to stop measurement.")

        logger.info("Stopping sync...")
        if not manager.stopSync():
            logger.error("Failed to stop sync.")

        xdpcHandler.cleanup()
        logger.info("Successful exit.")

if __name__ == "__main__":
    main()