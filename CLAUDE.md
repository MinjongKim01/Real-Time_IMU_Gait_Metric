# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Real-time IMU gait analysis system for Movella DOT sensors. Captures synchronized foot-mounted IMU data (accelerometer, gyroscope, quaternions) from multiple sensors, processes gait events (heel-strike, toe-off), and visualizes real-time trajectories using ZUPT-based dead reckoning.

## Running the Application

```bash
# Run the main application
python main.py
```

The application will:
1. Scan for Movella DOT devices (20 second timeout or until key press)
2. Connect to devices specified in `user_settings.py` SENSOR_MAPPING
3. Configure sensors (60Hz sampling, "General" filter profile)
4. Synchronize devices using the last device as root node
5. Start measurement and open real-time visualization window
6. Close the plot window or Ctrl+C to stop

## Architecture

### Main Components

**main.py**: Entry point that orchestrates the entire pipeline
- Device discovery, connection, and synchronization via XdpcHandler
- Spawns plotting thread (daemon) for visualization
- Main loop: reads packets → batches data by sensor role → feeds to GaitAnalyzer
- 5ms sleep in main loop balances CPU usage with 60Hz responsiveness

**xdpchandler.py**: Movella DOT SDK wrapper (based on official SDK example)
- Implements `XsDotCallback` interface to receive live IMU packets
- Maintains per-device packet buffers as `deque` (optimized for O(1) popleft operations)
- Thread-safe packet buffering with locks
- Max buffer size: 500 packets (configurable)
- Key optimization: replaced `list.pop(0)` with `deque.popleft()` for performance

**gait_analyzer.py**: Core gait analysis engine
- Thread-safe data buffering and processing with locks
- Two analysis modes:
  - **Realtime analysis** (`update_realtime_analysis`): Sliding 10-second window for live plotting
  - **Full trajectory analysis** (`run_full_trajectory_analysis`): Processes entire collected dataset
- Stride-by-stride trajectory calculation with heading correction
- Data flow: local frame IMU → filtering → global frame rotation → gravity removal → gait event detection → ZUPT integration → position

**realtime_plotter.py**: Matplotlib-based real-time visualization
- 6 subplots: gyro-z (L/R), free acceleration (L/R), trajectory (L/R)
- Object pooling for vlines (reuse instead of create/destroy) to reduce overhead
- Reset button to clear all buffers and restart data collection
- 100ms animation interval

**gait_utils.py**: Signal processing and gait event detection utilities
- Butterworth lowpass filter (5Hz cutoff, 5th order by default)
- Gait event detection from gyroscope z-axis signal (threshold crossing + peak clustering)
- Mid-stance index estimation for ZUPT
- Trapezoidal integration with initial position offset

**user_settings.py**: Configuration
- `SENSOR_MAPPING`: Dict mapping Bluetooth MAC addresses to sensor roles ("left", "right")
- `FS`: Sampling frequency (60Hz)
- Whitelist for device filtering (currently unused)

### Data Flow

```
Movella DOT Sensors
    ↓ (Bluetooth packets via SDK)
XdpcHandler (packet buffering)
    ↓ (packet batches by role)
GaitAnalyzer.process_data_batch() (buffering)
    ↓ (periodic updates)
GaitAnalyzer.update_realtime_analysis()
    - Filters IMU data (5Hz lowpass)
    - Rotates to global frame using quaternions
    - Detects gait events (TO/HS from gyro-z)
    - Calculates stride trajectories with ZUPT
    - Applies heading correction (align to 90°)
    ↓
RealTimeVisualizer._update()
    - Plots gyro-z with event markers
    - Plots free acceleration (3-axis)
    - Plots XY trajectory with HS/TO markers
    - Displays stride length annotations
```

### Key Algorithms

**Stride Trajectory Calculation** (`_calculate_stride_trajectory`):
1. Extract TO-HS segment from buffer
2. Apply 5Hz lowpass filter to acc/gyro
3. Rotate acceleration to global frame using quaternions
4. Remove gravity (9.81 m/s² on z-axis)
5. Detect gait events in segment (TO=start, HS=end, plus mid-stance)
6. Integrate acceleration to velocity with ZUPT (zero at TO, mid-stance, HS)
7. Apply velocity drift correction between ZUPT points (linear detrending)
8. Integrate velocity to position (trapezoidal)
9. Calculate heading correction angle from first stride (align to π/2 radians = 90°)
10. Apply heading correction rotation to XY plane
11. For first stride: shift TO to origin; for later strides: continue from last HS position

**Gait Event Detection** (`detect_gait_events`):
1. Find threshold crossings in gyro-z signal (default 80 deg/s, falling edge)
2. Filter crossings by minimum distance (30 samples)
3. Between consecutive crossings, find negative peaks (local minima)
4. Cluster peaks into two groups (TO and HS)
5. TO = deepest peak in first cluster
6. HS = deepest peak in second cluster within time window (30-60% of TO-TO interval)

**Heading Correction**:
- First stride: calculate angle from TO→HS vector to target heading (90°)
- Store correction angle per sensor role
- Apply 2D rotation matrix to all subsequent position data
- Ensures trajectory points "forward" regardless of initial sensor orientation

### Threading Model

- **Main thread**: Device management, data collection loop
- **Daemon plot thread**: Matplotlib animation (started in main.py:78)
- Both threads access GaitAnalyzer with lock protection
- Window close event sets `_is_closed` flag to stop main loop

### Performance Optimizations

1. **Buffer management**: `deque` instead of `list` for O(1) removal
2. **Object pooling**: Reuse matplotlib vline objects instead of create/destroy
3. **Helper functions**: `_extract_buffer_slice`, `_convert_quaternions_to_rotations` to reduce code duplication
4. **Shallow copying**: Use `.copy()` for numpy arrays instead of `deepcopy` when thread-safe
5. **Constants**: Pre-defined filter cutoffs, thresholds, etc. to avoid magic numbers

## Sensor Configuration

Movella DOT sensors must be configured with specific Bluetooth MAC addresses in `user_settings.py`:

```python
SENSOR_MAPPING = {
    "D4:22:CD:00:8D:01": "left",
    "D4:22:CD:00:86:04": "right",
}
```

Available sensors are listed in comments in `user_settings.py`. To use different sensors, update the mapping.

## Important Constants

- **Sampling frequency**: 60Hz (user_settings.FS)
- **Filter cutoff**: 5Hz (gait_analyzer.FILTER_CUTOFF)
- **Gait detection threshold**: 100 deg/s for analysis, 80 deg/s default in utils (gait_analyzer.GAIT_DETECTION_THRESHOLD)
- **Realtime window**: 10 seconds (gait_analyzer.realtime_window_sec)
- **Main loop sleep**: 5ms (main.MAIN_LOOP_SLEEP_SEC)
- **Animation interval**: 100ms (realtime_plotter.ANIMATION_INTERVAL)
- **Target heading**: π/2 radians = 90° (gait_analyzer.TARGET_HEADING_ANGLE)
- **Gravity**: 9.81 m/s² (gait_analyzer.GRAVITY)

## Dependencies

The code uses the Movella DOT PC SDK (`movelladot_pc_sdk`), which is a proprietary Python package. SDK documentation is in `Xsens DOT BLE Services Specifications.pdf`. Other dependencies:
- numpy
- scipy (signal processing, spatial transforms)
- matplotlib (visualization)
- pynput (keyboard input for device scanning)

## Common Modifications

**Change sampling rate**: Modify `FS` in `user_settings.py` (must be supported by hardware: 1, 4, 10, 12, 15, 20, 30, 60, 120 Hz)

**Adjust filter cutoff**: Modify `FILTER_CUTOFF` in `GaitAnalyzer.__init__()` (affects smoothness vs responsiveness)

**Tune gait event detection**: Modify `GAIT_DETECTION_THRESHOLD` in `GaitAnalyzer.__init__()` (higher = fewer false positives, may miss events; lower = more sensitive)

**Add more sensors**: Add entries to `SENSOR_MAPPING` and ensure `sensor_roles` is passed correctly to GaitAnalyzer and RealTimeVisualizer

**Change trajectory alignment**: Modify `TARGET_HEADING_ANGLE` in `GaitAnalyzer.__init__()` (default π/2 points forward along +Y axis)

**Adjust ZUPT points**: Modify `MIDSTANCE_RATIO` in `gait_utils.py` or the mid-stance detection logic in `_calculate_stride_trajectory`

## Debugging Tips

- **No devices found**: Check Bluetooth is enabled, sensors are powered on, MAC addresses in SENSOR_MAPPING are correct
- **Sync failed**: Device order matters; last device in `deviceList` becomes root node (see main.py:65-66)
- **Trajectory drift**: ZUPT points may be incorrectly detected; check gyro-z plot for event markers alignment
- **Heading correction issues**: First stride must be > 10cm (`MIN_STEP_DISTANCE`); initial direction affects all subsequent data
- **Performance issues**: Reduce `realtime_window_sec`, increase animation interval, or reduce filter order
- **Event detection failures**: Visualize gyro-z signal; adjust threshold or `MIN_EVENT_DISTANCE_SAMPLES` in gait_utils.py

## Code Comments

Many comments are in Korean. Key Korean terms:
- "최적화" = optimization
- "버퍼" = buffer
- "감지" = detection
- "분석" = analysis
