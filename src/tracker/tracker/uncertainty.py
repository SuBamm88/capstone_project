import numpy as np

# Base measurement noise [m²] at confidence = 1.0
_R_BASE = 0.10

# Confidence scaling: lower confidence → larger R
# R = base / confidence²  (clamped)
_CONF_MIN = 0.01
_R_MAX = 2.0


def compute_R(confidence: float) -> np.ndarray:
    """Return 2x2 measurement noise covariance from detection confidence."""
    conf = max(confidence, _CONF_MIN)
    r = min(_R_BASE / (conf ** 2), _R_MAX)
    return np.diag([r, r])
