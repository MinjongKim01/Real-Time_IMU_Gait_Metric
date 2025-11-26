import sys
import time
import threading
from xdpchandler import *
from gait_analyzer import GaitAnalyzer
from realtime_plotter import RealTimeVisualizer
from device_manager import DeviceManager
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

    # Initialize device manager
    device_manager = DeviceManager(xdpcHandler, SENSOR_MAPPING)

    # Discover, connect, and configure devices
    if not device_manager.discover_and_connect():
        sys.exit(-1)

    # Synchronize devices
    if not device_manager.synchronize_devices():
        device_manager.cleanup()
        sys.exit(-1)

    # Get connected devices
    connected_devices = device_manager.get_connected_devices()

    # ----- Initializing -----
    logger.info("Initializing Gait Analyzer and Plotter...")
    analyzer = GaitAnalyzer(fs=FS, sensor_roles=list(SENSOR_MAPPING.values()))
    plotter = RealTimeVisualizer(sensor_roles=list(SENSOR_MAPPING.values()))
    plot_thread = threading.Thread(target=plotter.start, args=(analyzer, xdpcHandler), daemon=True)
    plot_thread.start()

    # ----- Starting Measurement -----
    if not device_manager.start_measurement():
        device_manager.cleanup()
        sys.exit(-1)

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
        device_manager.stop_measurement()
        device_manager.stop_sync()
        device_manager.cleanup()
        logger.info("Successful exit.")

if __name__ == "__main__":
    main()