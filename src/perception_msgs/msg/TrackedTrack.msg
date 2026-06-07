# A robot-side stable track fused from CCTV object observations and optional pose direction.

int32 id
string class_id
float32 class_confidence
string source

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float32 heading                 # motion heading yaw [rad], from velocity (else 0)
float32 facing                  # body/gaze facing yaw [rad], valid only if has_facing
bool has_facing                 # true when facing was estimated from vision pose
float32 footprint_area
float32 tracking_stability

geometry_msgs/Point[] predicted_trajectory
