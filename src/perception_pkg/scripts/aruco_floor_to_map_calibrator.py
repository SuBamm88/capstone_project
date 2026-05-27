#!/usr/bin/env python3

import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class ArucoFloorToMapCalibrator(Node):
    def __init__(self):
        super().__init__("aruco_floor_to_map_calibrator")

        self.marker_ids = [
            int(value)
            for value in self.declare_parameter("marker_ids", [1, 2, 3, 4]).value
        ]
        self.marker_floor_points = [
            float(value)
            for value in self.declare_parameter(
                "marker_floor_points",
                [0.0, 0.0, 0.4, 0.0, 0.4, 0.3, 0.0, 0.3],
            ).value
        ]
        self.marker_map_points = [
            float(value)
            for value in self.declare_parameter(
                "marker_map_points",
                [0.0, 0.0, 0.4, 0.0, 0.4, 0.3, 0.0, 0.3],
            ).value
        ]
        self.parent_frame = self.declare_parameter("parent_frame", "map").value
        self.child_frame = self.declare_parameter("child_frame", "floor").value
        self.output_yaml = self.declare_parameter(
            "output_yaml", "config/floor_to_map_tf.yaml"
        ).value
        self.output_yaml = self._package_path(self.output_yaml)

        self.compute_and_save()

    def compute_and_save(self):
        floor_points = self._points_from_flat_array(self.marker_floor_points)
        map_points = self._points_from_flat_array(self.marker_map_points)

        if len(floor_points) != len(self.marker_ids):
            raise ValueError("marker_floor_points must contain x/y for each marker id")
        if len(map_points) != len(self.marker_ids):
            raise ValueError("marker_map_points must contain x/y for each marker id")
        if len(floor_points) < 2:
            raise ValueError("floor-to-map calibration needs at least 2 markers")

        rotation, translation = self._estimate_rigid_transform(
            np.array(floor_points), np.array(map_points)
        )
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        residual = (rotation @ np.array(floor_points).T).T + translation - map_points
        rmse = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))

        self.save_tf(float(translation[0]), float(translation[1]), yaw, rmse)

    def _points_from_flat_array(self, values):
        if len(values) % 2 != 0:
            raise ValueError("point arrays must contain x/y pairs")

        points = []
        for index in range(0, len(values), 2):
            points.append([float(values[index]), float(values[index + 1])])
        return points

    def _estimate_rigid_transform(self, source_points, target_points):
        source_center = np.mean(source_points, axis=0)
        target_center = np.mean(target_points, axis=0)

        source_zero = source_points - source_center
        target_zero = target_points - target_center

        covariance = source_zero.T @ target_zero
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T

        if np.linalg.det(rotation) < 0:
            vt[-1, :] *= -1
            rotation = vt.T @ u.T

        translation = target_center - rotation @ source_center
        return rotation, translation

    def save_tf(self, x, y, yaw, rmse):
        output_dir = os.path.dirname(os.path.abspath(self.output_yaml))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        data = {
            "floor_to_map_tf_broadcaster_node": {
                "ros__parameters": {
                    "parent_frame": self.parent_frame,
                    "child_frame": self.child_frame,
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": yaw,
                }
            }
        }

        with open(self.output_yaml, "w", encoding="utf-8") as yaml_file:
            yaml.safe_dump(data, yaml_file, sort_keys=False)

        self.get_logger().info(
            f"Saved floor-to-map TF: {self.output_yaml} "
            f"(x={x:.6f}, y={y:.6f}, yaw={yaw:.6f}, rmse={rmse:.6f} m)"
        )

    def _package_path(self, path):
        if os.path.isabs(path):
            return path

        package_share = get_package_share_directory("perception_pkg")
        return os.path.join(package_share, path)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFloorToMapCalibrator()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
