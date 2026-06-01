#!/usr/bin/env python3
"""각 CCTV의 바닥 커버 영역(FOV)을 map 위 부채꼴로 시각화하는 노드.

파라미터로 카메라의 바닥 위치[m], 수평 방향 yaw[deg], 수평 FOV[deg]를 받아
카메라마다 map 평면(z=0)에 평평한 삼각형 부채꼴 마커를 발행한다.
모든 부채꼴 바깥 영역이 곧 CCTV가 경고해 줄 수 없는 사각지대다 —
데모 시연과 추후 경로 패널티 양쪽에 쓸 수 있다.

yaw는 직접 측정하거나 homography로 역산한 값을 써도 된다(Test.md 참고).
FOV 기본값은 C922/C930 웹캠 기준이다.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class CctvFovNode(Node):
    def __init__(self):
        super().__init__("cctv_fov")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.marker_topic = self.declare_parameter(
            "marker_topic", "/cctv/fov_markers"
        ).value
        self.publish_rate = float(self.declare_parameter("publish_rate", 1.0).value)

        # 평탄 리스트, 카메라당 6개 값:
        #   [x[m], y[m], yaw[deg], hfov[deg], range[m], color_id]
        # color_id로 부채꼴 색을 고른다.
        self.cameras = self.declare_parameter(
            "cameras", [0.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        ).value
        if len(self.cameras) % 6 != 0 or not self.cameras:
            raise ValueError("cameras must be a flat list of N*[x,y,yaw,hfov,range,color]")

        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 1)
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 0.1), self._publish)
        self.get_logger().info(
            f"cctv_fov publishing {len(self.cameras)//6} FOV wedge(s) on {self.marker_topic}"
        )

    def _publish(self):
        palette = [
            (1.0, 0.55, 0.0),  # 주황
            (0.0, 0.6, 1.0),   # 파랑
            (0.2, 0.8, 0.2),   # 초록
            (0.8, 0.2, 0.8),   # 자홍
        ]
        array = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        array.markers.append(delete)

        n = len(self.cameras) // 6
        for i in range(n):
            x, y, yaw_deg, hfov_deg, rng, color_id = self.cameras[i * 6:(i + 1) * 6]
            color = palette[int(color_id) % len(palette)]
            array.markers.append(self._wedge(i, x, y, yaw_deg, hfov_deg, rng, color))
        self.pub.publish(array)

    def _wedge(self, idx, x, y, yaw_deg, hfov_deg, rng, color):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cctv_fov"
        marker.id = idx
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.25)

        # 카메라 위치를 꼭짓점으로, yaw±(hfov/2) 범위를 rng[m]까지 부채꼴로 채운다.
        # 부채꼴을 16등분해 삼각형들로 그린다. z=0.02 m로 바닥보다 살짝 띄움.
        yaw = math.radians(yaw_deg)       # 수평 방향 [rad]
        half = math.radians(hfov_deg) / 2.0  # 반각 [rad]
        steps = 16
        apex = Point(x=x, y=y, z=0.02)    # 부채꼴 꼭짓점(카메라 위치) [m]
        prev = None
        for s in range(steps + 1):
            a = yaw - half + (2 * half) * (s / steps)  # 현재 각도 [rad]
            edge = Point(x=x + rng * math.cos(a), y=y + rng * math.sin(a), z=0.02)
            if prev is not None:
                marker.points.extend([apex, prev, edge])
            prev = edge
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = CctvFovNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
