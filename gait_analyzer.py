import numpy as np
import collections
import threading
import copy
from typing import Dict, List, Optional, Tuple, Any
from scipy.spatial.transform import Rotation

import gait_utils as gu
from user_settings import FS
from logging_config import get_logger
from config_loader import get_config

# Initialize logger
logger = get_logger(__name__)

# Initialize config
config = get_config()

class GaitAnalyzer:
    def __init__(self, fs: int = FS, sensor_roles: List[str] = None):
        if sensor_roles is None:
            sensor_roles = ["left", "right"]
        self.fs = fs
        self.dt = 1.0 / fs
        self.sensor_roles = sensor_roles
        self.data_buffers = {}
        self.analysis_results = {}
        self.full_analysis_results = {}
        self.heading_correction_angle = {role: None for role in self.sensor_roles}

        # Optimization: Load constants from config
        self.realtime_window_sec = config.get('analysis.realtime_window_sec', 10.0)
        self.FILTER_CUTOFF = config.get('analysis.filter_cutoff_hz', 5)
        self.MIN_SAMPLES_FOR_ANALYSIS = 10
        self.GRAVITY = config.get('analysis.gravity', 9.81)  # m/s²
        self.GAIT_DETECTION_THRESHOLD = config.get('analysis.gait_detection_threshold', 100)  # deg/s
        self.MIN_STEP_DISTANCE = config.get('analysis.min_step_distance', 0.1)  # m (minimum step distance for heading correction)
        self.TARGET_HEADING_ANGLE = config.get('analysis.target_heading_rad', np.pi / 2)  # 90° (forward direction) 
        
        for role in self.sensor_roles:
            self.data_buffers[role] = {
                "timestamp": collections.deque(),
                "acc": collections.deque(),
                "gyr": collections.deque(),
                "quat": collections.deque(),
            }
            self.analysis_results[role] = {
                "heel_strikes_idx": [],
                "toe_offs_idx": [],
                "gyr_z": np.array([]),
                "acc_xyz": np.array([]),
                "time_array": np.array([]),
                "position": np.array([]),  # Cumulative storage of realtime trajectory
                "position_hs_idx": [],  # HS indices in position array
                "position_to_idx": [],  # TO indices in position array
            }
            self.full_analysis_results[role] = {
                "position": np.array([]),
                "heel_strikes_idx": [],
                "toe_offs_idx": [],
            }
        
        # Tracking variables for stride processing
        self.last_processed_stride = {role: {"to_idx": None, "hs_idx": None} for role in self.sensor_roles}
        self.accumulated_position = {role: np.array([0.0, 0.0, 0.0]) for role in self.sensor_roles}
        
        self.lock = threading.Lock()

    def _convert_quaternions_to_rotations(self, quat_list: List[List[float]]) -> Rotation:
        """Convert quaternions to Rotation objects (wxyz -> xyzw)"""
        quat_data_wxyz = np.array(quat_list)
        quat_data_xyzw = np.roll(quat_data_wxyz, -1, axis=1)
        return Rotation.from_quat(quat_data_xyzw)

    def _prepare_imu_data(self, role: str, acc_local: np.ndarray, gyr_local: np.ndarray,
                         quat_list: List[List[float]]) -> Tuple[np.ndarray, np.ndarray, Rotation]:
        """
        Common preprocessing: filter and transform to global frame.

        Args:
            role: Sensor role ('left' or 'right')
            acc_local: Local frame acceleration [N, 3]
            gyr_local: Local frame gyroscope [N, 3]
            quat_list: List of quaternions [N, 4]

        Returns:
            Tuple of (acc_global_corrected, gyr_z, orientations)
            - acc_global_corrected: Global frame acceleration with gravity removed [N, 3]
            - gyr_z: Z-axis gyroscope (sign-corrected for left foot) [N]
            - orientations: Rotation objects
        """
        # Filter
        acc_local_filtered = gu.butter_lowpass_filter(acc_local, self.FILTER_CUTOFF, self.fs)
        gyr_local_filtered = gu.butter_lowpass_filter(gyr_local, self.FILTER_CUTOFF, self.fs)

        # Transform to global frame
        orientations = self._convert_quaternions_to_rotations(quat_list)
        acc_global = orientations.apply(acc_local_filtered)

        # Remove gravity
        acc_global_corrected = acc_global.copy()
        acc_global_corrected[:, 2] -= self.GRAVITY

        # Extract and correct gyro-z
        gyr_z = gyr_local_filtered[:, 2]
        if role == "left":
            gyr_z = -gyr_z

        return acc_global_corrected, gyr_z, orientations

    def _apply_heading_correction(self, role: str, position: np.ndarray, to_idx: int,
                                  hs_idx: int, initial_pos: np.ndarray) -> np.ndarray:
        """
        Calculate and apply heading correction to align trajectory to target heading.

        Args:
            role: Sensor role ('left' or 'right')
            position: Position array [N, 3]
            to_idx: Toe-off index in position array
            hs_idx: Heel-strike index in position array
            initial_pos: Initial position offset [3]

        Returns:
            Corrected position array [N, 3]
        """
        # Calculate heading correction angle from first stride (if not already set)
        if self.heading_correction_angle[role] is None:
            step_vector = position[hs_idx, :2] - position[to_idx, :2]
            if np.linalg.norm(step_vector) > self.MIN_STEP_DISTANCE:
                current_angle_rad = np.arctan2(step_vector[1], step_vector[0])
                self.heading_correction_angle[role] = self.TARGET_HEADING_ANGLE - current_angle_rad
                logger.info(f"[{role}] Heading correction angle set: {np.degrees(self.heading_correction_angle[role]):.1f} degrees")

        # Apply heading correction rotation
        if self.heading_correction_angle[role] is not None:
            angle = self.heading_correction_angle[role]
            c, s = np.cos(angle), np.sin(angle)
            R = np.array([[c, -s], [s, c]])
            initial_pos_xy = initial_pos[:2]
            pos_xy_centered = position[:, :2] - initial_pos_xy
            pos_xy_rotated = pos_xy_centered @ R.T
            position[:, :2] = pos_xy_rotated + initial_pos_xy

        return position

    def _integrate_with_zupt_correction(self, acc: np.ndarray, zupt_indices: List[int],
                                       dt: float) -> np.ndarray:
        """
        Integrate acceleration to velocity with ZUPT corrections and drift correction.

        Args:
            acc: Acceleration array [N, 3]
            zupt_indices: Indices where velocity should be zero (sorted)
            dt: Time step

        Returns:
            Velocity array [N, 3] with ZUPT and drift corrections applied
        """
        n_samples = len(acc)
        velocity = np.zeros_like(acc)

        # Trapezoidal integration with ZUPT
        for i in range(1, n_samples):
            velocity[i, :] = velocity[i - 1, :] + 0.5 * (acc[i, :] + acc[i - 1, :]) * dt
            if i in zupt_indices:
                velocity[i, :] = 0.0

        # Velocity drift correction between ZUPT points
        if len(zupt_indices) > 1:
            for j in range(len(zupt_indices) - 1):
                start_idx = zupt_indices[j]
                end_idx = zupt_indices[j + 1]
                n_points = end_idx - start_idx

                if n_points <= 0:
                    continue

                drift = velocity[end_idx - 1, :]
                correction = np.outer(np.linspace(0, 1, n_points), drift)
                velocity[start_idx:end_idx, :] = velocity[start_idx:end_idx, :] - correction

        return velocity

    def _extract_buffer_slice(self, role: str, start_idx: int,
                             end_idx: Optional[int] = None) -> Tuple[np.ndarray, np.ndarray, List[List[float]]]:
        """Efficiently extract data slice from buffer"""
        buffers = self.data_buffers[role]

        if end_idx is None:
            # Use indexing instead of slicing deque directly
            acc_slice = np.array([buffers["acc"][i] for i in range(start_idx, len(buffers["acc"]))])
            gyr_slice = np.array([buffers["gyr"][i] for i in range(start_idx, len(buffers["gyr"]))])
            quat_slice = [buffers["quat"][i] for i in range(start_idx, len(buffers["quat"]))]
        else:
            acc_slice = np.array([buffers["acc"][i] for i in range(start_idx, end_idx)])
            gyr_slice = np.array([buffers["gyr"][i] for i in range(start_idx, end_idx)])
            quat_slice = [buffers["quat"][i] for i in range(start_idx, end_idx)]

        return acc_slice, gyr_slice, quat_slice
    
    def _calculate_stride_trajectory(self, role: str, to_global_idx: int,
                                     hs_global_idx: int) -> Tuple[Optional[np.ndarray], Optional[int], Optional[int]]:
        """
        Calculate trajectory for one stride (from TO to HS).

        Args:
            role: Sensor role ('left' or 'right')
            to_global_idx: Toe-off index in buffer
            hs_global_idx: Heel-strike index in buffer

        Returns:
            position: Calculated position array [N, 3]
            to_pos_idx: TO index in position array
            hs_pos_idx: HS index in position array
        """
        try:
            # Extract data
            acc_local, gyr_local, quat_list = self._extract_buffer_slice(role, to_global_idx, hs_global_idx + 1)

            if len(acc_local) < 2:
                return None, None, None

            # Preprocess IMU data (filter, transform, remove gravity)
            acc_global_corrected, gyr_z, orientations = self._prepare_imu_data(role, acc_local, gyr_local, quat_list)

            # Relative time array within the interval
            n_samples = len(acc_local)
            time_array = np.arange(n_samples) / self.fs

            # Re-detect gait events (relative indices within interval)
            events = gu.detect_gait_events(gyr_z, time_array, threshold=self.GAIT_DETECTION_THRESHOLD)

            # TO is always index 0, HS is last index
            to_idx_local = 0
            hs_idx_local = n_samples - 1

            # Estimate mid-stance (within interval)
            ms_indices = []
            if events:
                # If events exist, calculate mid-stance between them
                hs_indices_local = [e['heelstrike_idx'] for e in events]
                if hs_indices_local:
                    # Mid-stance between TO and first HS
                    first_hs = hs_indices_local[0]
                    if first_hs > to_idx_local:
                        ms_idx = to_idx_local + int(0.3 * (first_hs - to_idx_local))
                        ms_indices.append(ms_idx)

                    # Mid-stance between HS
                    for i in range(len(hs_indices_local) - 1):
                        cycle_start = hs_indices_local[i]
                        cycle_end = hs_indices_local[i + 1]
                        zmpt_idx = cycle_start + int(0.3 * (cycle_end - cycle_start))
                        ms_indices.append(zmpt_idx)
            
            # Velocity integration with ZUPT and drift correction
            zupt_indices = sorted(list(set([to_idx_local] + ms_indices + [hs_idx_local])))
            velocity = self._integrate_with_zupt_correction(acc_global_corrected, zupt_indices, self.dt)
            
            # Position integration
            initial_pos = self.accumulated_position[role].copy()
            position = gu.integrate(velocity, self.dt, initial_pos=initial_pos)

            # Apply heading correction
            position = self._apply_heading_correction(role, position, to_idx_local, hs_idx_local, initial_pos)

            # For first stride, align TO exactly to origin
            is_first_stride = np.allclose(self.accumulated_position[role], [0.0, 0.0, 0.0])
            if is_first_stride:
                to_offset = position[to_idx_local, :].copy()
                position = position - to_offset

            # Save last position for next stride
            self.accumulated_position[role] = position[hs_idx_local, :].copy()

            return position, to_idx_local, hs_idx_local

        except Exception as e:
            logger.error(f"Error calculating stride trajectory for {role}: {e}")
            return None, None, None

    def process_data_batch(self, data_batch: Dict[str, Dict[str, Any]]) -> None:
        """
        Process a batch of IMU data from multiple sensors.

        Args:
            data_batch: Dict mapping sensor role to packet data
        """
        with self.lock:
            for role, data in data_batch.items():
                if role not in self.sensor_roles:
                    continue
                
                self.data_buffers[role]["timestamp"].append(data["timestamp"])
                self.data_buffers[role]["acc"].append(data["acc"])
                self.data_buffers[role]["gyr"].append(data["gyr"])
                self.data_buffers[role]["quat"].append(data["quaternion"]) 
                    
    def update_realtime_analysis(self) -> None:
        """Update realtime analysis for visualization."""
        with self.lock:
            for role in self.sensor_roles:
                n_total_samples = len(self.data_buffers[role]["timestamp"])
                n_samples_to_process = int(self.fs * self.realtime_window_sec)
                
                if n_total_samples < self.MIN_SAMPLES_FOR_ANALYSIS:
                    continue
                
                start_idx = max(0, n_total_samples - n_samples_to_process)
                
                try:
                    # Optimization: Use helper function to reduce duplication and improve efficiency
                    acc_local, gyr_local, quat_list = self._extract_buffer_slice(role, start_idx)
                except Exception as e:
                    logger.error(f"Error slicing data for {role} (realtime): {e}")
                    continue

                try:
                    # Preprocess IMU data (filter, transform, remove gravity)
                    acc_global_corrected, gyr_z, orientations = self._prepare_imu_data(role, acc_local, gyr_local, quat_list)
                except Exception as e:
                    continue
                
                current_time_array = np.arange(len(gyr_z)) / self.fs

                # Detect gait events
                events = gu.detect_gait_events(gyr_z, current_time_array, threshold=self.GAIT_DETECTION_THRESHOLD)
                hs_indices_local = [e['heelstrike_idx'] for e in events]
                to_indices_local = [e['toeoff_idx'] for e in events]

                # Convert local indices to global buffer indices
                hs_indices_global = [start_idx + idx for idx in hs_indices_local]
                to_indices_global = [start_idx + idx for idx in to_indices_local]
                
                # Store data for realtime display
                self.analysis_results[role]["gyr_z"] = gyr_z
                self.analysis_results[role]["acc_xyz"] = acc_global_corrected
                self.analysis_results[role]["time_array"] = current_time_array
                self.analysis_results[role]["heel_strikes_idx"] = hs_indices_local
                self.analysis_results[role]["toe_offs_idx"] = to_indices_local

                # ----- Calculate trajectory for each stride -----
                # Find completed strides: TO-HS pairs not yet processed
                if len(to_indices_global) > 0 and len(hs_indices_global) > 0:
                    # Get last processed HS index
                    last_processed_hs = self.last_processed_stride[role]["hs_idx"]

                    # Match TO-HS pairs
                    for to_idx in to_indices_global:
                        # Skip already processed stride range
                        if last_processed_hs is not None and to_idx <= last_processed_hs:
                            continue  # Already processed stride or earlier

                        # Find first HS after this TO
                        next_hs_candidates = [hs for hs in hs_indices_global if hs > to_idx]

                        if not next_hs_candidates:
                            continue  # If HS not yet detected, wait for next update
                        
                        hs_idx = next_hs_candidates[0]

                        # Check again if this stride was already processed (exact matching)
                        last_proc = self.last_processed_stride[role]
                        if last_proc["to_idx"] == to_idx and last_proc["hs_idx"] == hs_idx:
                            continue  # Already processed

                        # New stride detected - calculate trajectory
                        logger.debug(f"[{role}] New stride detected: TO={to_idx}, HS={hs_idx}")
                        
                        position, to_pos_idx, hs_pos_idx = self._calculate_stride_trajectory(role, to_idx, hs_idx)

                        if position is not None:
                            # Accumulate position
                            if self.analysis_results[role]["position"].size == 0:
                                # First stride
                                self.analysis_results[role]["position"] = position
                                self.analysis_results[role]["position_to_idx"] = [to_pos_idx]
                                self.analysis_results[role]["position_hs_idx"] = [hs_pos_idx]
                            else:
                                # Append to existing position
                                prev_len = len(self.analysis_results[role]["position"])
                                self.analysis_results[role]["position"] = np.vstack([
                                    self.analysis_results[role]["position"],
                                    position
                                ])
                                self.analysis_results[role]["position_to_idx"].append(prev_len + to_pos_idx)
                                self.analysis_results[role]["position_hs_idx"].append(prev_len + hs_pos_idx)
                            
                            # Mark as processed
                            self.last_processed_stride[role]["to_idx"] = to_idx
                            self.last_processed_stride[role]["hs_idx"] = hs_idx

                            logger.debug(f"[{role}] Stride trajectory calculated. Total positions: {len(self.analysis_results[role]['position'])}")


    def run_full_trajectory_analysis(self) -> None:
        """Run full trajectory analysis on all buffered data."""
        with self.lock:
            logger.info("Starting full trajectory analysis...")
            # Optimization: Copy only necessary data (shallow copy instead of deepcopy)
            local_buffer_sizes = {role: len(self.data_buffers[role]["timestamp"]) for role in self.sensor_roles}
            self.heading_correction_angle = {role: None for role in self.sensor_roles}

            for role in self.sensor_roles:
                n_samples = local_buffer_sizes[role]
                if n_samples < self.fs * 1.0:
                    logger.warning(f"Not enough data for {role} to analyze.")
                    continue

                logger.info(f"Analyzing {role} with {n_samples} samples...")
                
                try:
                    # Optimization: Use helper function for efficient data extraction
                    acc_local, gyr_local, quat_list = self._extract_buffer_slice(role, 0, n_samples)

                    # Preprocess IMU data (filter, transform, remove gravity)
                    acc_global_corrected, gyr_z, orientations = self._prepare_imu_data(role, acc_local, gyr_local, quat_list)

                    time_array = np.arange(n_samples) / self.fs
                except Exception as e:
                    logger.error(f"Error preparing full data for {role}: {e}")
                    continue
                    
                events = gu.detect_gait_events(gyr_z, time_array, threshold=self.GAIT_DETECTION_THRESHOLD)
                hs_indices = [e['heelstrike_idx'] for e in events]
                to_indices = [e['toeoff_idx'] for e in events]

                ms_indices = gu.detect_midstance_indices(hs_indices)

                # Determine ZUPT indices
                zupt_indices = [0]
                if to_indices:
                    first_event_idx = to_indices[0]
                    zupt_indices = sorted(list(set([first_event_idx] + ms_indices)))

                if not zupt_indices:
                    zupt_indices = [0]

                # Velocity integration with ZUPT and drift correction
                velocity = self._integrate_with_zupt_correction(acc_global_corrected, zupt_indices, self.dt)

                initial_pos = np.array([0.0, 0.0, 0.0])
                position = gu.integrate(velocity, self.dt, initial_pos=initial_pos)

                # Apply heading correction (use first stride TO-HS for angle calculation)
                if to_indices and hs_indices:
                    first_to_idx = to_indices[0]
                    first_hs_idx = next((idx for idx in hs_indices if idx > first_to_idx), None)
                    if first_hs_idx is not None:
                        position = self._apply_heading_correction(role, position, first_to_idx, first_hs_idx, initial_pos)

                if to_indices and len(zupt_indices) > 1:
                    last_zupt_idx = zupt_indices[-1]
                    last_analysis_idx = last_zupt_idx + 1
                    last_analysis_idx = min(last_analysis_idx, n_samples)
                else:
                    last_analysis_idx = 1

                final_position = position.copy() 
                if to_indices: 
                    first_to_idx = to_indices[0]
                    first_to_pos_offset = final_position[first_to_idx, :2]
                    final_position[:, :2] = final_position[:, :2] - first_to_pos_offset
                
                self.full_analysis_results[role]["position"] = final_position[:last_analysis_idx]
                self.full_analysis_results[role]["heel_strikes_idx"] = [idx for idx in hs_indices if idx < last_analysis_idx]
                self.full_analysis_results[role]["toe_offs_idx"] = [idx for idx in to_indices if idx < last_analysis_idx]

            logger.info("Full trajectory analysis finished.")

    def get_realtime_data(self) -> Dict[str, Dict[str, Any]]:
        """Get realtime analysis data for visualization."""
        with self.lock:
            # Optimization: numpy arrays use .copy(), dicts use shallow copy
            result = {}
            for role, data in self.analysis_results.items():
                result[role] = {
                    "gyr_z": data["gyr_z"].copy() if data["gyr_z"].size > 0 else data["gyr_z"],
                    "acc_xyz": data["acc_xyz"].copy() if data["acc_xyz"].size > 0 else data["acc_xyz"],
                    "time_array": data["time_array"].copy() if data["time_array"].size > 0 else data["time_array"],
                    "heel_strikes_idx": data["heel_strikes_idx"][:],  # list shallow copy
                    "toe_offs_idx": data["toe_offs_idx"][:],
                    "position": data["position"].copy() if data["position"].size > 0 else data["position"],
                    "position_hs_idx": data["position_hs_idx"][:],
                    "position_to_idx": data["position_to_idx"][:]
                }
            return result
        
    def get_full_analysis_data(self) -> Dict[str, Dict[str, Any]]:
        """Get full trajectory analysis data."""
        with self.lock:
            # Optimization: numpy arrays use .copy()
            result = {}
            for role, data in self.full_analysis_results.items():
                result[role] = {
                    "position": data["position"].copy() if data["position"].size > 0 else data["position"],
                    "heel_strikes_idx": data["heel_strikes_idx"][:],
                    "toe_offs_idx": data["toe_offs_idx"][:]
                }
            return result
        
    def reset_data(self) -> None:
        """Reset all analysis data and buffers."""
        with self.lock:
            logger.info("Resetting gait analyzer data...")
            self.heading_correction_angle = {role: None for role in self.sensor_roles}
            self.last_processed_stride = {role: {"to_idx": None, "hs_idx": None} for role in self.sensor_roles}
            self.accumulated_position = {role: np.array([0.0, 0.0, 0.0]) for role in self.sensor_roles}
            
            for role in self.sensor_roles:
                self.data_buffers[role]["timestamp"].clear()
                self.data_buffers[role]["acc"].clear()
                self.data_buffers[role]["gyr"].clear()
                self.data_buffers[role]["quat"].clear()
                
                self.analysis_results[role] = {
                    "heel_strikes_idx": [],
                    "toe_offs_idx": [],
                    "gyr_z": np.array([]), 
                    "acc_xyz": np.array([]), 
                    "time_array": np.array([]),
                    "position": np.array([]),
                    "position_hs_idx": [],
                    "position_to_idx": [],
                }
                self.full_analysis_results[role] = {
                    "position": np.array([]),
                    "heel_strikes_idx": [],
                    "toe_offs_idx": [],
                }