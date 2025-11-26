# Real-Time IMU Gait Analysis

Real-time gait analysis system using foot-mounted Movella DOT IMU sensors with ZUPT-based trajectory estimation and automatic stride detection.

## Features

- **Real-time Data Streaming**: Live data acquisition from Movella DOT sensors via Bluetooth
- **Gait Event Detection**: Automatic detection of heel-strike and toe-off events using gyroscope signals
- **Trajectory Estimation**: ZUPT (Zero Velocity Update) based dead reckoning for stride trajectory calculation
- **Heading Correction**: Automatic alignment of trajectory to target heading direction
- **Live Visualization**: Real-time plotting of gyroscope, acceleration, and trajectory data
- **Stride Metrics**: Automatic calculation and display of stride length
- **Multi-sensor Support**: Simultaneous tracking of left and right foot sensors
- **Configurable Parameters**: YAML-based configuration system for easy parameter tuning

## System Requirements

- Python 3.8 or 3.9, 3.10
- Windows, or Linux
- Bluetooth adapter for wireless sensor communication
- Movella DOT IMU sensors (minimum 2 for bilateral gait analysis)

## Installation

### 1. Create Conda Environment

```bash
# Create a new conda environment
conda create -n IMU python=3.9

# Activate the environment
conda activate IMU
```

### 2. Install Dependencies

```bash
# Install required packages
pip install -r requirements.txt
```

### 3. Sensor Configuration

Edit `config.yaml` to set your sensor MAC addresses:

```yaml
sensors:
  sampling_rate: 60
  filter_profile: "General"
  mapping:
    "D4:22:CD:00:8D:01": "left"    # Replace with your left sensor MAC
    "D4:22:CD:00:86:04": "right"   # Replace with your right sensor MAC
```

To find your sensor MAC addresses:
1. Turn on your Movella DOT sensors
2. Run the initial connection script (it will display detected sensors)
3. Note the MAC addresses and update `config.yaml`

## Quick Start

### Basic Usage

```bash
# Activate conda environment
conda activate IMU

# Run the real-time gait analysis
python main.py
```

The system will:
1. Scan for Movella DOT sensors
2. Connect to sensors specified in `config.yaml`
3. Synchronize sensors
4. Start real-time data streaming and visualization
5. Display live plots of gyroscope, acceleration, and trajectory

### Controls

- **Reset Button**: Click the reset button in the plot window to clear all data and restart analysis
- **Close Window**: Close the plot window to stop data acquisition and exit

## Configuration Guide

All system parameters can be configured in `config.yaml`:

### Sensor Configuration

```yaml
sensors:
  sampling_rate: 60              # Sensor sampling rate in Hz
  filter_profile: "General"      # Filter profile: "General", "Dynamic", "LowLatency"
  mapping:                        # Sensor MAC address to role mapping
    "MAC_ADDRESS_1": "left"
    "MAC_ADDRESS_2": "right"
```

### Analysis Parameters

```yaml
analysis:
  realtime_window_sec: 10.0      # Time window for real-time analysis
  filter_cutoff_hz: 5            # Butterworth filter cutoff frequency
  gait_detection_threshold: 100  # Gyro threshold for gait event detection (deg/s)
  gravity: 9.81                  # Gravity constant (m/s²)
  min_step_distance: 0.1         # Minimum step distance for heading correction (m)
  target_heading_rad: 1.5708     # Target heading angle in radians (π/2 = 90°)
```

### Performance Settings

```yaml
performance:
  main_loop_sleep_sec: 0.005     # Main loop sleep duration (5ms for 60Hz)
  max_packet_buffer_size: 500    # Maximum packet buffer size per sensor
  animation_interval_ms: 100     # Plot update interval in milliseconds
```

### Logging Configuration

```yaml
logging:
  level: "INFO"                  # Log level: DEBUG, INFO, WARNING, ERROR, CRITICAL
  console: true                  # Enable console logging
  file: null                     # Log file path (null to disable file logging)
  format: "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
```

## Architecture Overview

### System Components

```
┌─────────────┐
│   main.py   │  - Entry point and main loop
└──────┬──────┘
       │
       ├──────────────────────────────────────────────┐
       │                                              │
┌──────▼──────────┐                        ┌─────────▼──────────┐
│ DeviceManager   │                        │  GaitAnalyzer      │
│                 │                        │                    │
│ - Device setup  │                        │ - Gait detection   │
│ - Connection    │                        │ - ZUPT trajectory  │
│ - Sync          │                        │ - Heading correct  │
└────────┬────────┘                        └─────────┬──────────┘
         │                                           │
    ┌────▼────┐                                      │
    │ XDPC    │                                      │
    │ Handler │◄─────────────────────────────────────┘
    └────┬────┘                                      │
         │                                           │
         │                                  ┌────────▼──────────┐
         │                                  │ RealTimeVisualizer│
         │                                  │                   │
         └──────────────────────────────────► - Live plotting   │
                                            │ - Stride metrics  │
                                            └───────────────────┘
```

### Data Flow

1. **Acquisition**: XdpcHandler receives IMU packets from sensors
2. **Buffering**: Data stored in thread-safe deques with configurable size
3. **Processing**: GaitAnalyzer processes batches and performs:
   - Butterworth lowpass filtering (5Hz)
   - Quaternion to rotation matrix conversion
   - Global frame transformation
   - Gravity compensation
   - Gait event detection from gyroscope z-axis
   - Mid-stance estimation
   - ZUPT-based velocity integration
   - Drift correction
   - Position integration
   - Heading correction
4. **Visualization**: RealTimeVisualizer updates matplotlib plots at 100ms intervals

### Key Algorithms

#### Gait Event Detection

Uses gyroscope z-axis signal with threshold-based detection:
- Detects toe-off as negative peaks in swing phase
- Detects heel-strike as secondary negative peaks
- Filters events with minimum distance constraints
- Validates heel-strike timing within stride cycle

#### ZUPT (Zero Velocity Update)

Dead reckoning with drift correction:
- Applies zero velocity constraint at stationary phases (toe-off, heel-strike, mid-stance)
- Linear drift correction between ZUPT points
- Trapezoidal velocity integration
- Cumulative position integration

#### Heading Correction

Aligns trajectory to target heading:
- Calculates heading angle from first stride
- Applies 2D rotation to align with target direction (90°)
- Maintains consistent forward progression

## Usage Examples

### Example 1: Basic Real-time Analysis

```python
from gait_analyzer import GaitAnalyzer
from xdpchandler import XdpcHandler
from device_manager import DeviceManager
from user_settings import SENSOR_MAPPING, FS

# Initialize components
xdpc = XdpcHandler()
device_manager = DeviceManager(xdpc, SENSOR_MAPPING)
analyzer = GaitAnalyzer(fs=FS, sensor_roles=["left", "right"])

# Setup and connect devices
device_manager.discover_and_connect()
device_manager.synchronize_devices()
device_manager.start_measurement()

# Process data in main loop
while True:
    if xdpc.packetsAvailable():
        # Get packet for each device
        for device in device_manager.get_connected_devices():
            packet = xdpc.getNextPacket(device.bluetoothAddress())
            # Process packet...

        # Update analysis
        analyzer.update_realtime_analysis()
        realtime_data = analyzer.get_realtime_data()
```

### Example 2: Custom Configuration

```python
from config_loader import get_config

# Load custom configuration
config = get_config('my_custom_config.yaml')

# Access configuration values
sampling_rate = config.get('sensors.sampling_rate', 60)
filter_cutoff = config.get('analysis.filter_cutoff_hz', 5)
```

### Example 3: Offline Analysis

```python
from gait_analyzer import GaitAnalyzer

analyzer = GaitAnalyzer(fs=60)

# Process recorded data
for data_batch in recorded_data:
    analyzer.process_data_batch(data_batch)

# Run full trajectory analysis
analyzer.run_full_trajectory_analysis()
full_results = analyzer.get_full_analysis_data()

# Access results
for role in ["left", "right"]:
    position = full_results[role]["position"]
    heel_strikes = full_results[role]["heel_strikes_idx"]
    toe_offs = full_results[role]["toe_offs_idx"]
```

## File Structure

```
.
├── main.py                  # Main entry point
├── gait_analyzer.py         # Core gait analysis algorithms
├── gait_utils.py           # Utility functions (filtering, integration, detection)
├── device_manager.py       # Device lifecycle management
├── xdpchandler.py          # Movella SDK wrapper
├── realtime_plotter.py     # Real-time visualization
├── user_settings.py        # User configuration interface
├── config_loader.py        # Configuration loader (singleton)
├── logging_config.py       # Logging configuration
├── config.yaml             # Main configuration file
├── requirements.txt        # Python dependencies
├── requirements-dev.txt    # Development dependencies
└── README.md              # This file
```

## Troubleshooting

### Sensors Not Detected

- Ensure sensors are powered on and charged
- Check Bluetooth is enabled on your computer
- Verify sensors are not connected to another device
- Try moving sensors closer to the computer

### Connection Failures

- Restart sensors by turning them off and on
- Restart Bluetooth service on your computer
- Check `config.yaml` has correct MAC addresses
- Try connecting to one sensor at a time

### Synchronization Issues

- Ensure all sensors have sufficient battery
- Check all sensors are the same model
- Verify firmware versions are compatible
- Try reducing the number of sensors

### Performance Issues

- Reduce `animation_interval_ms` in `config.yaml`
- Increase `main_loop_sleep_sec` if CPU usage is high
- Reduce `realtime_window_sec` for less data processing
- Close other applications using Bluetooth

### Trajectory Drift

- Adjust `filter_cutoff_hz` (try 3-7 Hz range)
- Check sensor mounting is secure and rigid
- Ensure `gait_detection_threshold` is appropriate for your gait pattern
- Verify sensors are correctly mapped (left/right)

## Development

### Running Tests

```bash
# Run unit tests
pytest tests/

# Run with coverage
pytest --cov=. tests/
```

### Code Quality

```bash
# Type checking
mypy *.py

# Linting
pylint *.py
flake8 *.py

# Formatting
black *.py
```

## Contact

For questions or support, please contact:
- minjong.kim.snu.ac.kr

## Acknowledgments

- Movella Technologies for the DOT SDK

## Version History

### v1.0.0 (Current)
- Initial release with real-time gait analysis
- ZUPT-based trajectory estimation
- Automatic gait event detection
- YAML-based configuration system
- Comprehensive logging framework
- Modular architecture with DeviceManager


