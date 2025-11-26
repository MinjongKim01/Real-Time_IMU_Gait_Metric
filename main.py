import sys
import time
import threading
import movelladot_pc_sdk
from xdpchandler import *
from gait_analyzer import GaitAnalyzer
from realtime_plotter import RealTimeVisualizer
from user_settings import SENSOR_MAPPING, FS

# 최적화: 상수 정의
MAIN_LOOP_SLEEP_SEC = 0.005  # 5ms - CPU 사용률과 응답성의 균형
MIN_DATA_FOR_ANALYSIS_SEC = 1.0  # 분석에 필요한 최소 데이터 시간

def main():
    xdpcHandler = XdpcHandler()
    if not xdpcHandler.initialize():
        xdpcHandler.cleanup()
        sys.exit(-1)

    # ----- Devices Scanning -----
    print("Scanning for DOTs...")
    xdpcHandler.scanForDots()
    if len(xdpcHandler.detectedDots()) == 0:
        print("No device(s) found. Aborting.")
        xdpcHandler.cleanup()
        sys.exit(-1)

    # ----- Devices Connecting -----
    print("Connecting to DOTs...")
    xdpcHandler.connectDots()
    connected_devices = []
    all_devices = xdpcHandler.connectedDots()
    
    if not all_devices:
        print("Could not connect to any devices. Aborting.")
        xdpcHandler.cleanup()
        sys.exit(-1)

    for dev in all_devices:
        if dev.bluetoothAddress() in SENSOR_MAPPING:
            connected_devices.append(dev)
            print(f"Found device: {dev.bluetoothAddress()} ({SENSOR_MAPPING[dev.bluetoothAddress()]})")

    if len(connected_devices) != len(SENSOR_MAPPING):
        print("Could not find all devices specified in SENSOR_MAPPING. Aborting.")
        xdpcHandler.cleanup()
        sys.exit(-1)
    
    # ----- Measurement Setting -----
    print(f"\nSetting up {len(connected_devices)} device(s)...")
    for device in connected_devices:
        if device.setOnboardFilterProfile("General"):
            print(f"[{SENSOR_MAPPING[device.bluetoothAddress()]}] Set profile to General")
        else:
            print(f"[{SENSOR_MAPPING[device.bluetoothAddress()]}] Setting profile FAILED!")

        if device.setOutputRate(FS):
            print(f"[{SENSOR_MAPPING[device.bluetoothAddress()]}] Set output rate to {FS} Hz")
        else:
            print(f"[{SENSOR_MAPPING[device.bluetoothAddress()]}] Setting output rate FAILED!")

    # ----- Synchronization -----
    manager = xdpcHandler.manager()
    deviceList = connected_devices
    print(f"\nStarting sync... Root node: {deviceList[-1].bluetoothAddress()}")
    if not manager.startSync(deviceList[-1].bluetoothAddress()):
        print(f"Sync failed. Reason: {manager.lastResultText()}. Retrying after stop...")
        manager.stopSync()
        if not manager.startSync(deviceList[-1].bluetoothAddress()):
            print(f"Sync retry failed. Reason: {manager.lastResultText()}. Aborting.")
            xdpcHandler.cleanup()
            sys.exit(-1)

    # ----- Initializing -----
    print("\nInitializing Gait Analyzer and Plotter...")
    analyzer = GaitAnalyzer(fs=FS, sensor_roles=list(SENSOR_MAPPING.values()))
    plotter = RealTimeVisualizer(sensor_roles=list(SENSOR_MAPPING.values()))
    plot_thread = threading.Thread(target=plotter.start, args=(analyzer, xdpcHandler), daemon=True) 
    plot_thread.start()
    
    # ----- Starting Measurement -----
    print("Putting devices into measurement mode.")
    for device in connected_devices:
        payload_mode = movelladot_pc_sdk.XsPayloadMode_CustomMode5
        if not device.startMeasurement(payload_mode):
            role = SENSOR_MAPPING.get(device.bluetoothAddress(), "Unknown")
            print(f"[{role}] Could not start measurement. Reason: {device.lastResultText()}")
            
    print("\nMain loop. Streaming data. Close the plot window to stop.")
    print("-----------------------------------------")

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

            # 최적화: CPU 사용률 개선 (1ms -> 5ms)
            # 60Hz 샘플링 레이트에서 5ms sleep은 충분히 빠른 응답성 유지
            time.sleep(MAIN_LOOP_SLEEP_SEC)

    except KeyboardInterrupt:
        print("\nUser interrupted. Stopping data stream...")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    
    # ----- Exitting -----
    finally:
        print("\n-----------------------------------------")
        print("Stopping measurement...")
        for device in connected_devices:
            if not device.stopMeasurement():
                print("Failed to stop measurement.")

        print("Stopping sync...")
        if not manager.stopSync():
            print("Failed to stop sync.")

        xdpcHandler.cleanup()
        print("Successful exit.")

if __name__ == "__main__":
    main()