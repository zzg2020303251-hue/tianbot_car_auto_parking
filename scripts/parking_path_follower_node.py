#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ParkingPathFollower:
    def __init__(self):
        self.global_frame = rospy.get_param("~global_frame", "tianbot_mini/map")
        self.base_frame = rospy.get_param("~base_frame", "tianbot_mini/base_link")
        self.path_topic = rospy.get_param("~path_topic", "/parking_path")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/tianbot_mini/cmd_vel")
        self.scan_topic = rospy.get_param("~scan_topic", "/tianbot_mini/scan")

        self.control_rate = rospy.get_param("~control_rate", 15.0)
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 0.22)
        self.goal_xy_tolerance = rospy.get_param("~goal_xy_tolerance", 0.06)
        self.goal_yaw_tolerance = math.radians(rospy.get_param("~goal_yaw_tolerance_deg", 8.0))
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.12)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.8)
        self.heading_gain = rospy.get_param("~heading_gain", 1.8)
        self.final_yaw_gain = rospy.get_param("~final_yaw_gain", 1.2)
        self.reverse_heading_threshold = math.radians(rospy.get_param("~reverse_heading_threshold_deg", 105.0))
        self.front_stop_distance = rospy.get_param("~front_stop_distance", 0.18)
        self.rear_stop_distance = rospy.get_param("~rear_stop_distance", 0.18)

        self.path: Optional[Path] = None
        self.front_clearance = float("inf")
        self.rear_clearance = float("inf")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.path_sub = rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)
        rospy.on_shutdown(self.publish_stop)
        rospy.loginfo("Parking path follower ready. Waiting for %s.", self.path_topic)

    def path_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.path = None
            self.publish_stop()
            rospy.loginfo("Parking follower cleared path and stopped.")
            return
        self.path = msg
        rospy.loginfo("Parking follower received path with %d poses.", len(msg.poses))

    def scan_callback(self, msg: LaserScan) -> None:
        front = float("inf")
        rear = float("inf")
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) < math.radians(25.0):
                front = min(front, r)
            if abs(normalize_angle(angle - math.pi)) < math.radians(25.0):
                rear = min(rear, r)
        self.front_clearance = front
        self.rear_clearance = rear

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.05)
            )
        except Exception as exc:
            try:
                query_time = rospy.Time.now() - rospy.Duration(0.20)
                tf_msg = self.tf_buffer.lookup_transform(
                    self.global_frame, self.base_frame, query_time, rospy.Duration(0.10)
                )
            except Exception as fallback_exc:
                rospy.logwarn_throttle(1.0, "Parking follower TF failed: %s", fallback_exc)
                return None
        t = tf_msg.transform.translation
        yaw = yaw_from_quaternion(tf_msg.transform.rotation)
        return t.x, t.y, yaw

    def control_loop(self, _event) -> None:
        if self.path is None or not self.path.poses:
            return
        robot = self.get_robot_pose()
        if robot is None:
            return
        x, y, yaw = robot

        goal = self.path.poses[-1].pose
        goal_yaw = yaw_from_quaternion(goal.orientation)
        goal_dist = math.hypot(goal.position.x - x, goal.position.y - y)
        goal_yaw_error = normalize_angle(goal_yaw - yaw)

        cmd = Twist()
        if goal_dist <= self.goal_xy_tolerance:
            if abs(goal_yaw_error) <= self.goal_yaw_tolerance:
                self.publish_stop()
                rospy.loginfo_throttle(2.0, "Parking goal reached.")
                return
            cmd.angular.z = self.clamp(self.final_yaw_gain * goal_yaw_error,
                                       -self.max_angular_speed, self.max_angular_speed)
            self.cmd_pub.publish(cmd)
            return

        target = self.select_lookahead_pose(x, y)
        dx = target[0] - x
        dy = target[1] - y
        target_angle = math.atan2(dy, dx)
        heading_error = normalize_angle(target_angle - yaw)

        reverse = abs(heading_error) > self.reverse_heading_threshold
        if reverse:
            heading_error = normalize_angle(heading_error + math.pi)

        speed_scale = min(1.0, max(0.25, goal_dist / max(self.lookahead_distance, 0.01)))
        cmd.linear.x = self.max_linear_speed * speed_scale * (-1.0 if reverse else 1.0)
        cmd.angular.z = self.clamp(self.heading_gain * heading_error,
                                   -self.max_angular_speed, self.max_angular_speed)

        if cmd.linear.x > 0.0 and self.front_clearance < self.front_stop_distance:
            rospy.logwarn_throttle(1.0, "Front obstacle too close; stopping parking follower.")
            cmd = Twist()
        if cmd.linear.x < 0.0 and self.rear_clearance < self.rear_stop_distance:
            rospy.logwarn_throttle(1.0, "Rear obstacle too close; stopping parking follower.")
            cmd = Twist()

        self.cmd_pub.publish(cmd)

    def select_lookahead_pose(self, x: float, y: float) -> Tuple[float, float]:
        poses = self.path.poses
        closest = 0
        best = float("inf")
        for i, pose in enumerate(poses):
            p = pose.pose.position
            d = math.hypot(p.x - x, p.y - y)
            if d < best:
                best = d
                closest = i

        accumulated = 0.0
        prev = poses[closest].pose.position
        for pose in poses[closest + 1:]:
            p = pose.pose.position
            accumulated += math.hypot(p.x - prev.x, p.y - prev.y)
            if accumulated >= self.lookahead_distance:
                return p.x, p.y
            prev = p
        p = poses[-1].pose.position
        return p.x, p.y

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("parking_path_follower")
    ParkingPathFollower()
    rospy.spin()
