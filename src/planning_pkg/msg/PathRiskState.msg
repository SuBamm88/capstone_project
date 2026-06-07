std_msgs/Header header

bool    dynamic_path_occupied      # IsDynamicPathOccupied 조건 필드

float32 closest_obstacle_dist      # 가장 가까운 객체까지 거리 [m] (-1 = 없음)
float32 path_occupancy_duration    # 동적 위협 점유 지속 시간 [s]
string  debug_message              # 현재 상태 한 줄 요약