#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from motor_api.msg import MotorCommand
from jetson_camera.msg import PointArray
import tf
from geometry_msgs.msg import Point
from coordinator.msg import CurrentMode
from visualization_msgs.msg import Marker

NOMINAL_SPEED = 0.3
kW = 5.0
kF = 2.0

def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

class LaneFollowerNode:
    def __init__(self):
        rospy.init_node('lane_follower_node', anonymous=True)
        self.targets = []
        self.robot_pose = None
        self.is_enabled = False
        self.direction = 1
        self.mode_subscriber = rospy.Subscriber('/co/current', CurrentMode, self.mode_callback)
        
        self.subscriber = rospy.Subscriber('/lane_detection/targets', PointArray, self.targets_callback)
        self.odometer_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.marker_pub = rospy.Publisher('/lane_follower/marker', Marker, queue_size=10)
        self.angle_to_next_target = 0.0

    def targets_callback(self, msg):
        self.follow_lane()
        if self.robot_pose is None:
            return
        if len(msg.points) <= 2:
            return
        
        if msg.points[0].y < msg.points[-1].y:
            self.direction = 1
        else:
            self.direction = -1
        self.targets = []
        
        for point in msg.points:
            # Convert point from robot frame to world frame using self.robot_pose
            robot_orientation = self.robot_pose.orientation
            robot_position = self.robot_pose.position

            # Convert quaternion to Euler angles
            quaternion = (
                robot_orientation.x,
                robot_orientation.y,
                robot_orientation.z,
                robot_orientation.w
            )
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

            # Rotation matrix for yaw
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)

            # Transform point
            world_x = robot_position.x + point.x * cos_yaw - point.y * sin_yaw
            world_y = robot_position.y + point.x * sin_yaw + point.y * cos_yaw

            # Create a new point object with world coordinates
            world_point = Point()
            world_point.x = world_x
            world_point.y = world_y
            world_point.z = point.z  # Assuming flat ground

            self.targets.append(world_point)
        
        # Create a marker to visualize the transformed world points
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lane_follower"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.points = self.targets

        self.marker_pub.publish(marker)
    
    def mode_callback(self, msg):
        if msg.current == 0:
            self.is_enabled = True
            rospy.loginfo("Lane following mode enabled")
        else:
            self.is_enabled = False
            rospy.loginfo("Lane following mode disabled")
            # Stop the robot when not in lane following mode
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd)

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
        
    def follow_lane(self):
        if not self.is_enabled:
            return

        # Remove targets closer than 5 cm
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        
        for target in self.targets[:]:
            dx = target.x - robot_x
            dy = target.y - robot_y
            # Transform dx, dy from world frame to robot frame
            orientation = self.robot_pose.orientation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            _, _, robot_yaw = tf.transformations.euler_from_quaternion(quaternion)
            cos_yaw = np.cos(-robot_yaw)
            sin_yaw = np.sin(-robot_yaw)
            dx_robot = dx * cos_yaw - dy * sin_yaw
            dy_robot = dx * sin_yaw + dy * cos_yaw
            print("dx_robot: {}, dy_robot: {}".format(dx_robot, dy_robot))
            if dx_robot < 0.02:
                self.targets.remove(target)
        
        # Get robot yaw
        orientation = self.robot_pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, robot_yaw = tf.transformations.euler_from_quaternion(quaternion)

        if not self.targets:
            # No targets left, rotate to angle_to_next_target if available
            heading_error = self.angle_to_next_target - robot_yaw
            heading_error = normalize_angle(heading_error)
            print("Heading error non target: ", heading_error)
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 2.0 * self.direction
            self.publisher.publish(cmd)
            return

        # Use the first target as the goal
        target = self.targets[0]
        dx = target.x - robot_x
        dy = target.y - robot_y
        distance = np.hypot(dx, dy)
        angle_to_target = np.arctan2(dy, dx)

        if len(self.targets) > 1:
            next_target = self.targets[1]
            dx_next = next_target.x - target.x
            dy_next = next_target.y - target.y
            self.angle_to_next_target = np.arctan2(dy_next, dx_next)


        # Compute heading error
        heading_error2 = self.angle_to_next_target - robot_yaw
        heading_error2 = normalize_angle(heading_error2)
        heading_error = angle_to_target - robot_yaw
        heading_error = normalize_angle(heading_error)
        
        print("Heading error: ", heading_error)
        print("Heading error2: ", heading_error2)

        # Simple proportional controller
        linear_speed = NOMINAL_SPEED
        angular_speed = kW * (heading_error * np.tanh(5 * distance) ** 2 + heading_error2* 0.05)
        print("Angular speed: ", angular_speed)

        # Slow down if very close to target
        if distance < 0.2 and len(self.targets) < 2:
            linear_speed *= distance / 0.2

        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self.publisher.publish(cmd)


if __name__ == '__main__':
    try:
        node = LaneFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
