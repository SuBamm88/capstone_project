#!/usr/bin/env python3
"""캘리브레이션용: 카메라 영상을 클릭해 픽셀 좌표[px]를 읽는 도우미.

카메라 이미지 토픽을 구독해 OpenCV 창을 띄우고, 좌클릭마다 (u, v)[px]를 출력한다.
출력된 픽셀을 같은 마커의 map frame 미터 좌표[m](로봇을 마커 위에 올려 측정)와
짝지어 config/cctv_to_map.yaml의 image_points / map_points에 넣는다.

사용법:
    ros2 run perception_pkg pixel_picker_node --ros-args -p image_topic:=/camera2/image_raw
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class PixelPickerNode(Node):
    def __init__(self):
        super().__init__("pixel_picker")
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera2/image_raw"
        ).value
        self.window = f"pixel_picker [{self.image_topic}] - click markers, q to quit"

        self.bridge = CvBridge()
        self.frame = None
        self.clicks = []

        self.sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, 10
        )
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self._on_mouse)
        self.timer = self.create_timer(0.03, self._render)

        self.get_logger().info(
            f"pixel_picker on {self.image_topic}: left-click markers to print "
            f"pixel coords, press 'q' in the window to quit."
        )

    def _image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _on_mouse(self, event, x, y, flags, param):
        # 좌클릭 지점의 픽셀 좌표[px]를 기록/출력
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicks.append((x, y))
            self.get_logger().info(f"#{len(self.clicks)} pixel = [{x}, {y}]")

    def _render(self):
        if self.frame is None:
            return
        disp = self.frame.copy()
        for i, (x, y) in enumerate(self.clicks, start=1):
            cv2.circle(disp, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(
                disp, f"{i}:({x},{y})", (x + 6, y - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
            )
        cv2.imshow(self.window, disp)
        # 'q' 누르면 모은 픽셀들을 yaml에 붙여넣기 좋은 형태로 출력하고 종료
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Collected image_points (paste into yaml):")
            flat = [c for xy in self.clicks for c in xy]
            self.get_logger().info(f"image_points: {[float(v) for v in flat]}")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PixelPickerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
