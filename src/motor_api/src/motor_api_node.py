#!/usr/bin/env python2

import rospy
from motor_api.msg import MotorCommand, MotorCommandStatus
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from motorDriver import *
from encoderDriver import WheelEncoderDriver, WheelDirection
import math

import tf

# MotorCommand.forward : in m
# MotorCommand.right : in degrees
# MotorCommand.speed : [0, 1]
# MotorCommand.do_immediately : bool, if true then other commands are cleared from queue

DISTANCE_BETWEEN_WHEELS = 0.2
WHEEL_RADIUS = 0.029 # SLightly below measured value because rads_to_pwm is calibrated based on free spin
MAX_SPEED_RADS = 535/137 * 2 * math.pi


def rads_to_pwm_left(radians_per_second):
    sign = 1 if radians_per_second > 0 else -1
    ticks_per_second = abs(radians_per_second) /2/math.pi*137
    return sign * (5.055e-07 * ticks_per_second * ticks_per_second + 0.001957 * ticks_per_second - 0.1866)

def rads_to_pwm_right(radians_per_second):
    sign = 1 if radians_per_second > 0 else -1
    ticks_per_second = abs(radians_per_second) /2/math.pi*137
    return sign * (-9.477e-07 * ticks_per_second * ticks_per_second + 0.002961 * ticks_per_second - 0.3549)

class QueueCommand:
    def __init__(self, command):
        self.cmd = command
        self.start_time = None

    def calculate(self):
        turn_radians = self.cmd.right / 180*math.pi
        turn_distance = (DISTANCE_BETWEEN_WHEELS/2)*turn_radians
        left_distance = self.cmd.forward + turn_distance
        right_distance = self.cmd.forward - turn_distance
        max_distance = max(abs(left_distance), abs(right_distance))
        left_relative = left_distance / max_distance * self.cmd.speed
        right_relative = right_distance / max_distance * self.cmd.speed

        self.left = rads_to_pwm_left(left_relative * MAX_SPEED_RADS)
        self.right = rads_to_pwm_right(right_relative * MAX_SPEED_RADS)
        if self.cmd.speed != 0:
            self.duration = abs(left_distance) / (abs(left_relative) * MAX_SPEED_RADS * WHEEL_RADIUS)
        else:
            self.duration = 0.0


class MotorNode:
    def __init__(self, node_name):
        self.initialized = False
        self.node_name = node_name
        rospy.loginfo("Initializing Motor node...")
        rospy.init_node(self.node_name, anonymous=True)
        self.command_queue = []
        self._motor = DaguWheelsDriver()
        self.set_wheels_speed(0, 0)
        
        # To compute odometry
        self.encoders = {
            'last_right': 0,
            'last_left': 0,
            'right_direction': WheelDirection.FORWARD,
            'left_direction': WheelDirection.FORWARD,
        }
        
        self.current_command = None
        self.result_queue = []


        # Construct publishers
        self.publisher = rospy.Publisher(
            "/motor/results",
            MotorCommandStatus,
            queue_size=20,
        )
        
        self.odom_publisher = rospy.Publisher(
            "/odom",
            Odometry,
            queue_size=10,
        )
        
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.initialized = True
        rospy.loginfo("Node initialized!")
        self.timer = rospy.Timer(rospy.Duration(0.1), lambda e: self.update_motor_speeds(e))
        self.odom_stamp = None
        self.odom = Odometry()
        self.odom_angle = 0.0
        
        # Construct subscribers
        self.subscriber = rospy.Subscriber(
            "/motor/endpoint",
            MotorCommand,
            self.subscriber_cb,
            queue_size=1,
        )
        
        self.cmd_vel_subscriber = rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.cmd_vel_subscriber_cb,
            queue_size=1,
        )
        
        self.encoders_subscriber = rospy.Subscriber(
            "/encoder_ticks",
            Int32MultiArray,
            self.encoder_subscriber_cb,
            queue_size=1,
        )
    
    def __del__(self):
        if self.motor is not None:
            self.set_wheels_speed(0, 0)
            self.motor.close()

    def subscriber_cb(self, data):
        if not self.initialized:
            return
        if data.do_immediately:
            self.clear_current_command()
            self.result_queue += self.command_queue
            self.command_queue[:] = []
        self.command_queue.append(QueueCommand(data))
        self.publish_results()
        
    def cmd_vel_subscriber_cb(self, data):
        if not self.initialized:
            return
        
        left_speed = data.linear.x - data.angular.z * DISTANCE_BETWEEN_WHEELS / 2
        right_speed = data.linear.x + data.angular.z * DISTANCE_BETWEEN_WHEELS / 2
        left_pwm = left_speed
        right_pwm = right_speed
        if left_pwm > 0:
            left_pwm = min(left_pwm, 1)
            self.encoders['left_direction'] = WheelDirection.FORWARD
        else:
            left_pwm = max(left_pwm, -1)
            self.encoders['left_direction'] = WheelDirection.REVERSE
        if right_pwm > 0:
            right_pwm = min(right_pwm, 1)
            self.encoders['right_direction'] = WheelDirection.FORWARD
        else:
            right_pwm = max(right_pwm, -1)
            self.encoders['right_direction'] = WheelDirection.REVERSE
        self.set_wheels_speed(left_pwm, right_pwm)
        
    def encoder_subscriber_cb(self, msg):
        if self.odom_stamp is None:
            self.odom_stamp = rospy.Time.now()
            self.encoders['last_right'] = msg.data[1]
            self.encoders['last_left'] = msg.data[0]
            return
        
        current_time = rospy.Time.now()
        dt = (current_time - self.odom_stamp).to_sec()
        self.odom_stamp = current_time
        right_ticks = msg.data[1]
        left_ticks = msg.data[0]
        right_ticks_inc = (right_ticks - self.encoders['last_right']) * self.encoders['right_direction']
        left_ticks_inc = (left_ticks - self.encoders['last_left']) * self.encoders['left_direction']
        self.encoders['last_right'] = right_ticks
        self.encoders['last_left'] = left_ticks
        right_distance = (right_ticks_inc / 137.0) * (2 * math.pi * WHEEL_RADIUS)
        left_distance = (left_ticks_inc / 137.0) * (2 * math.pi * WHEEL_RADIUS)
        
        forward_distance = (right_distance + left_distance) / 2.0
        rotational_angle = (right_distance - left_distance) / DISTANCE_BETWEEN_WHEELS
        if abs(rotational_angle) < 1e-6:
            rotational_angle = 0.0
        if abs(forward_distance) < 1e-6:
            forward_distance = 0.0
        
        # Publish odometry information
        self.odom.twist.twist.linear.x = forward_distance / dt
        self.odom.twist.twist.angular.z = rotational_angle / dt
        self.odom.header.stamp = current_time
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.odom.pose.pose.position.x += forward_distance * math.cos(self.odom_angle)
        self.odom.pose.pose.position.y += forward_distance * math.sin(self.odom_angle)
        self.odom_angle += rotational_angle
        
        self.odom.pose.pose.orientation.w = math.cos(self.odom_angle / 2)
        self.odom.pose.pose.orientation.z = math.sin(self.odom_angle / 2) 
        
        self.odom_publisher.publish(self.odom)
        
        # Broadcast odometry transform
        odom_transform = TransformStamped()
        odom_transform.header.stamp = current_time
        odom_transform.header.frame_id = "odom"
        odom_transform.child_frame_id = "base_link"
        odom_transform.transform.translation.x = self.odom.pose.pose.position.x
        odom_transform.transform.translation.y = self.odom.pose.pose.position.y
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation = self.odom.pose.pose.orientation
        self.odom_broadcaster.sendTransformMessage(odom_transform)
            
    
    def clear_current_command(self):
        if self.current_command is None:
            return
        
        self.current_command.last_update = rospy.get_time()
        self.result_queue.append(self.current_command)
        self.current_command = None
        
    def publish_results(self):
        for cmd in self.result_queue:
            msg = MotorCommandStatus()
            msg.command = cmd.cmd
            if cmd.start_time is not None:
                msg.completion = (cmd.last_update - cmd.start_time) / cmd.duration
            else:
                msg.completion = 0.0
            self.publisher.publish(msg)
        self.result_queue[:] = []

    def update_motor_speeds(self, event):
        if self.current_command is None:
            if len(self.command_queue) > 0:
                self.current_command = self.command_queue.pop(0)
            if self.motor_left != 0 or self.motor_right != 0:
                self.set_wheels_speed(0, 0)
        if self.current_command is not None:
            if self.current_command.start_time is None:
                self.current_command.calculate()
                self.set_wheels_speed(self.current_command.left, self.current_command.right)
                self.current_command.start_time = event.current_real.to_sec()
                self.current_command.end_time = self.current_command.start_time + self.current_command.duration
            self.last_update = event.current_real.to_sec()
            if self.current_command.end_time < event.current_real.to_sec():
                self.clear_current_command()
                self.update_motor_speeds(event)
        self.publish_results()
    
    def set_wheels_speed(self, left, right):
        self._motor.set_wheels_speed(left=left, right=right)
        self.motor_left = left
        self.motor_right = right

if __name__ == "__main__":
    try:
        motor_api_node = MotorNode(node_name = "motor_api_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
