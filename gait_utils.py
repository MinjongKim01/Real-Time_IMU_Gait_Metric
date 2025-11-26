import numpy as np
from scipy.signal import butter, filtfilt, find_peaks
from scipy.integrate import cumulative_trapezoid

# 최적화: 상수 정의
DEFAULT_FILTER_ORDER = 5
MIN_PADLEN_MULTIPLIER = 3  # padlen = 3 * (2 * order)

# Gait event detection 상수
DEFAULT_GAIT_THRESHOLD = 80  # deg/s (gyro z-axis threshold)
MIN_EVENT_DISTANCE_SAMPLES = 30  # 이벤트 간 최소 거리
MIN_FILTERED_EVENT_DISTANCE = 15  # 필터링 후 이벤트 간 최소 거리
MAX_PEAK_CLUSTER_DISTANCE = 10  # peak 클러스터링 거리
LOOKAHEAD_SAMPLES = 8  # threshold 교차 판단 lookahead
HEELSTRIKE_TIME_WINDOW = (0.3, 0.6)  # toe-off 대비 heel-strike 시간 윈도우 비율
MIDSTANCE_RATIO = 0.3  # heel-strike 사이클에서 mid-stance 비율

def butter_lowpass_filter(data: np.ndarray, cutoff: float, fs: float, order: int = DEFAULT_FILTER_ORDER):
    if len(data) == 0:
        return np.array([])
        
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype="low", analog=False)
    
    padlen = MIN_PADLEN_MULTIPLIER * (2 * order)
    
    if len(data) < padlen + 1:
        padlen = len(data) - 1
        if padlen < 1:
            return data
    
    try:
        y = filtfilt(b, a, data, padlen=padlen, axis=0)
        return y
    except Exception as e:
        print(f"Filtering failed: {e}. Returning unfiltered data.")
        return data

def integrate(data: np.ndarray, dt: float, initial_pos: np.ndarray = None) -> np.ndarray:
    integrated_data = cumulative_trapezoid(data, dx=dt, axis=0, initial=0)
    if initial_pos is not None:
        integrated_data = integrated_data + initial_pos # [N, 3] + [3,] -> [N, 3]
        
    return integrated_data

def detect_gait_events(
    gyro_z: np.ndarray,
    time_array: np.ndarray,
    threshold: int = DEFAULT_GAIT_THRESHOLD,
    max_distance: int = MAX_PEAK_CLUSTER_DISTANCE,
):
    threshold_indices = []
    for i in range(1, len(gyro_z) - LOOKAHEAD_SAMPLES):
        if (gyro_z[i] >= threshold and gyro_z[i + 1] < threshold):
            if gyro_z[i] > gyro_z[i + LOOKAHEAD_SAMPLES]:
                if len(threshold_indices) > 0 and i - threshold_indices[-1] < MIN_EVENT_DISTANCE_SAMPLES:
                    continue
                threshold_indices.append(i)

    if len(threshold_indices) < 2:
        return []
    
    filtered_threshold_indices = [threshold_indices[0]]
    for i in range(1, len(threshold_indices)):
        if threshold_indices[i] - filtered_threshold_indices[-1] >= MIN_FILTERED_EVENT_DISTANCE:
            filtered_threshold_indices.append(threshold_indices[i])
    threshold_indices = filtered_threshold_indices

    events = []
    for i in range(len(threshold_indices) - 1):
        start, end = threshold_indices[i], threshold_indices[i + 1]
        section = gyro_z[start:end]
        section_time = time_array[start:end]
        
        if len(section) == 0:
            continue

        negative_peaks, _ = find_peaks(-section)
        if len(negative_peaks) > 0:
            sorted_peaks = sorted(negative_peaks, key=lambda p: section[p])
            clustered_peaks = [[], []]

            if len(sorted_peaks) == 2:
                clustered_peaks[0].append(sorted_peaks[0])
                clustered_peaks[1].append(sorted_peaks[1])
            else:
                for peak in sorted_peaks:
                    if (
                        len(clustered_peaks[0]) == 0
                        or abs(peak - clustered_peaks[0][-1]) < max_distance
                    ):
                        clustered_peaks[0].append(peak)
                    else:
                        clustered_peaks[1].append(peak)

            try:
                if clustered_peaks[0][0] < clustered_peaks[1][0]:
                    clustered_peaks = clustered_peaks[::-1]
            except IndexError:
                continue

            if len(clustered_peaks[0]) > 0 and len(clustered_peaks[1]) > 0:
                toeoff = min(clustered_peaks[0], key=lambda p: section[p])
                hs_candidates = clustered_peaks[1]
                events.append(
                    {
                        "toeoff_time": section_time[toeoff],
                        "toeoff_value": section[toeoff],
                        "toeoff_idx": start + toeoff,
                        "heelstrike_candidates": hs_candidates,
                        "start_idx": start,
                        "end_idx": end,
                    }
                )

    if not events:
        return []

    # FIXME:
    try:
        section = gyro_z[events[0]["start_idx"] : events[0]["end_idx"]]
        section_time = time_array[events[0]["start_idx"] : events[0]["end_idx"]]
        heelstrike = min(events[0]["heelstrike_candidates"], key=lambda p: section[p])
        events[0]["heelstrike_time"] = section_time[heelstrike]
        events[0]["heelstrike_value"] = section[heelstrike]
        events[0]["heelstrike_idx"] = events[0]["start_idx"] + heelstrike
    except Exception:
        events.pop(0)
        if not events:
            return []

    for idx, event in enumerate(events[1:], start=0):
        hs_candidates = event["heelstrike_candidates"]
        toeoff_time = events[idx]["toeoff_time"]
        next_toeoff_time = event["toeoff_time"]

        section = gyro_z[event["start_idx"] : event["end_idx"]]
        section_time = time_array[event["start_idx"] : event["end_idx"]]

        hs_candidates = [
            candidate
            for candidate in hs_candidates
            if section_time[candidate] > toeoff_time + HEELSTRIKE_TIME_WINDOW[0] * (next_toeoff_time - toeoff_time)
            and section_time[candidate] < toeoff_time + HEELSTRIKE_TIME_WINDOW[1] * (next_toeoff_time - toeoff_time)
        ]

        try:
            hs = min(hs_candidates, key=lambda p: section[p])
            event["heelstrike_time"] = section_time[hs]
            event["heelstrike_value"] = section[hs]
            event["heelstrike_idx"] = event["start_idx"] + hs
        except Exception:
            event["heelstrike_time"] = None
            event["heelstrike_value"] = None
            event["heelstrike_idx"] = None

    for event in events:
        event.pop("heelstrike_candidates", None)

    final_events = [e for e in events if e.get("heelstrike_time") is not None]
    return final_events


def detect_midstance_indices(hs_indices):
    """heel-strike 인덱스로부터 mid-stance 인덱스 추정"""
    zmpt = []
    for i in range(len(hs_indices) - 1):
        cycle_start = hs_indices[i]
        cycle_end = hs_indices[i + 1]
        zmpt_idx = cycle_start + int(MIDSTANCE_RATIO * (cycle_end - cycle_start))
        zmpt.append(zmpt_idx)
    return zmpt