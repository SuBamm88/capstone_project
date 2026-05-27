import math
from typing import List

import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from rclpy.node import Node
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


class RvizClickToPoseArray(Node):
    def __init__(self) -> None:
        super().__init__('rviz_click_to_pose_array')

        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('objects_topic', '/cctv/objects_map')
        self.declare_parameter('marker_topic', '/cctv/objects_marker')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('marker_radius', 0.25)

        self.clicked_point_topic = (
            self.get_parameter('clicked_point_topic').get_parameter_value().string_value
        )
        self.objects_topic = (
            self.get_parameter('objects_topic').get_parameter_value().string_value
        )
        self.marker_topic = (
            self.get_parameter('marker_topic').get_parameter_value().string_value
        )
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.marker_radius = self.get_parameter('marker_radius').get_parameter_value().double_value

        self.poses: List[Pose] = []

        self.pose_pub = self.create_publisher(PoseArray, self.objects_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.clicked_sub = self.create_subscription(
            PointStamped,
            self.clicked_point_topic,
            self.clickedPointCallback,
            10,
        )

        self.clear_srv = self.create_service(Empty, '/cctv/clear_objects', self.clearObjects)
        self.remove_last_srv = self.create_service(
            Empty,
            '/cctv/remove_last_object',
            self.removeLastObject,
        )

        timer_period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(timer_period, self.publishObjects)

        self.get_logger().info(
            f'Publishing {self.objects_topic} from RViz {self.clicked_point_topic} clicks'
        )

    def clickedPointCallback(self, msg: PointStamped) -> None:
        frame_id = msg.header.frame_id or self.output_frame
        if frame_id != self.output_frame:
            self.get_logger().warn(
                f'Ignoring clicked point in frame "{frame_id}". '
                f'Set RViz Fixed Frame to "{self.output_frame}".'
            )
            return

        pose = Pose()
        pose.position.x = msg.point.x
        pose.position.y = msg.point.y
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        self.poses.append(pose)
        self.get_logger().info(
            f'Added CCTV demo object #{len(self.poses)} at '
            f'({pose.position.x:.2f}, {pose.position.y:.2f})'
        )

    def clearObjects(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        del request
        self.poses.clear()
        self.publishObjects()
        self.get_logger().info('Cleared all CCTV demo objects')
        return response

    def removeLastObject(self, request: Empty.Request, response: Empty.Response) -> Empty.Response:
        del request
        if self.poses:
            self.poses.pop()
            self.get_logger().info('Removed last CCTV demo object')
        self.publishObjects()
        return response

    def publishObjects(self) -> None:
        stamp = self.get_clock().now().to_msg()

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.output_frame
        pose_array.poses = list(self.poses)
        self.pose_pub.publish(pose_array)

        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for index, pose in enumerate(self.poses):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.output_frame
            marker.ns = 'cctv_objects'
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose
            marker.pose.position.z = 0.05
            marker.scale.x = self.marker_radius * 2.0
            marker.scale.y = self.marker_radius * 2.0
            marker.scale.z = 0.10
            marker.color.r = 1.0
            marker.color.g = 0.15
            marker.color.b = 0.05
            marker.color.a = 0.75
            marker.lifetime.sec = max(1, math.ceil(2.0 / max(self.publish_rate, 0.1)))
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizClickToPoseArray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
