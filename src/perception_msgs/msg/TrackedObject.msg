# A single tracked object on the unified map layer.
# Filled cooperatively by CCTV (class) and LiDAR (motion).

int32 id                 # unique track id on the map layer
string class_id          # YOLO class name, or "unknown" for LiDAR-only objects
float32 class_confidence # classification confidence [0..1]
string source            # "CCTV" | "LiDAR" | "FUSED"

geometry_msgs/Point position    # map frame [m]
geometry_msgs/Vector3 velocity  # map frame [m/s] (zero for CCTV-only)
float32 heading                 # yaw [rad], valid for person objects (else 0)
float32 footprint_area          # ground-contact area [m^2] (0 if unknown)
float32 tracking_stability      # hits / (hits + missed), 1.0 for fresh CCTV-only

geometry_msgs/Point[] predicted_trajectory  # future positions in map frame (dynamic objects)
