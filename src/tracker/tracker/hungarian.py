import numpy as np
from scipy.optimize import linear_sum_assignment

# fallback 기본값(yaml로 gate_mahal_sq를 넘기면 associate(gate=...)로 덮어씀).
GATE_MAHAL_SQ = 25.0   # ~3.5σ gate


def associate(tracks, detections, gate=GATE_MAHAL_SQ):
    """
    Match tracks to detections using Mahalanobis distance + Hungarian.

    Args:
        gate : Mahalanobis² 게이트. 예측-검출 거리가 이보다 크면 매칭 후보에서
               제외(=새 트랙으로 분리). 빠른 보행자 fragmentation의 주 조절값.

    Returns:
        matched       : list of (track_idx, det_idx)
        unmatched_trk : list of track_idx with no match
        unmatched_det : list of det_idx with no match
    """
    if not tracks or not detections:
        return [], list(range(len(tracks))), list(range(len(detections)))

    # Build cost matrix
    cost = np.full((len(tracks), len(detections)), fill_value=1e9)
    for ti, trk in enumerate(tracks):
        for di, det in enumerate(detections):
            d = trk.innovation_distance_sq(det['z'], det['R'])
            if d < gate:
                cost[ti, di] = d

    row_ind, col_ind = linear_sum_assignment(cost)

    matched, unmatched_trk, unmatched_det = [], [], []

    matched_trk = set()
    matched_det = set()
    for ti, di in zip(row_ind, col_ind):
        if cost[ti, di] < gate:
            matched.append((ti, di))
            matched_trk.add(ti)
            matched_det.add(di)

    unmatched_trk = [i for i in range(len(tracks))     if i not in matched_trk]
    unmatched_det = [i for i in range(len(detections)) if i not in matched_det]

    return matched, unmatched_trk, unmatched_det
