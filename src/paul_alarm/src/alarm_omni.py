#!/usr/bin/python3

import traceback
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayLayout, MultiArrayDimension, String, Bool, Empty
from motor_api.msg import MotorCommand, MotorCommandStatus
import numpy as np

from paul_alarm.msg import DualImage
from coordinator.msg import CurrentMode

NODE_ID = 2

# Significant amounts of code taken from 
# https://pyimagesearch.com/2015/06/01/home-surveillance-and-motion-detection-with-the-raspberry-pi-python-and-opencv/
class AlarmOmniNode:
    def __init__(self):
        self.initialized = False
        rospy.init_node('alarm_omni', anonymous=False, xmlrpc_port=45202, tcpros_port=45203)
        self.image_sub = None
        self.mode_sub = rospy.Subscriber('/co/current', CurrentMode, self.mode_callback)
        self.result_image_pub = rospy.Publisher('/motion/video', DualImage, queue_size=1)
        self.stable_pub = rospy.Publisher('/motion/stable', Bool, queue_size=1)
        self.area_pub = rospy.Publisher('/motion/area', Float32, queue_size=1)
        self.area_sub = rospy.Subscriber('/motion/area', Float32, self.area_callback)
        self.motor_sub = rospy.Subscriber('/motor/results', MotorCommandStatus, self.motor_callback)
        self.motor_pub = rospy.Publisher('/motor/endpoint', MotorCommand, queue_size=1)
        self.alarm_pub = rospy.Publisher('/alarm', Empty, queue_size=5)

        self.bridge = CvBridge()
        self.bg = None
        self.first_received = False
        self.current_mode = 0
        self.go_right = True
        self.motor_is_running = False
        self.background_stable = False
        self.last_id = 1000

        self.initialized = True
        rospy.loginfo("Motion Detector Node Initialized")

    def area_callback(self, msg):
        if not self.initialized:
            return
        if self.current_mode != NODE_ID:
            return
        area = msg.data
        rospy.loginfo("mode {} area {} motor {} stable {}".format(self.current_mode, area, self.motor_is_running, self.background_stable))
        try:
            if not self.motor_is_running:
                if not self.background_stable and area < 400:
                    rospy.loginfo("Stable in 1 second...")
                    rospy.sleep(1)
                    rospy.loginfo("Background stabilized")
                    msg = Bool()
                    msg.data = True
                    self.stable_pub.publish(msg)
                    self.background_stable = True
                    rospy.loginfo("background_stable True")
                elif self.background_stable and area > 400:
                    rospy.loginfo("ALARM!")
                    self.background_stable = False
                    rospy.loginfo("background_stable False")
                    self.motor_is_running = True
                    rospy.loginfo("motor_is_running True")

                    msg = Bool()
                    msg.data = True
                    self.stable_pub.publish(msg)
                    
                    cmd = MotorCommand()
                    cmd.id = self.last_id
                    self.last_id += 1
                    cmd.forward = 0
                    cmd.right = (1 if self.go_right else -1) * 20
                    self.go_right = not self.go_right
                    cmd.speed = 1.0
                    cmd.do_immediately = True
                    self.motor_pub.publish(cmd)

                    msg = Empty()
                    self.alarm_pub.publish(msg)
        except Exception as e:
            rospy.logerr("Error area_callback: %s\n%s\n", e, traceback.format_exc())
    
    def motor_callback(self, msg):
        if not self.initialized:
            return
        rospy.loginfo("motor_callback")
        try:
            rospy.loginfo("Completed ID {}".format(msg.command.id))
            self.motor_is_running = False
        except Exception as e:
            rospy.logerr("Error in motor_callback: %s\n%s\n", e, traceback.format_exc())

    def mode_callback(self, msg):
        if not self.initialized:
            return
        try:
            self.current_mode = msg.current
            if msg.current == NODE_ID:
                self.image_sub = rospy.Subscriber('/camera/image_undistorted_paul', CompressedImage, self.image_callback)

                msg = Bool()
                msg.data = True
                self.stable_pub.publish(msg)
                rospy.loginfo("background_stable False (mode switch)")
                self.background_stable = False
                rospy.loginfo("motor_is_running False (mode switch)")
                self.motor_is_running = False
            else:
                if self.image_sub is not None:
                    self.image_sub.unregister()
                    self.image_sub = None
        except Exception as e:
            rospy.logerr("Error mode_callback: %s\n%s\n", e, traceback.format_exc())


    def image_callback(self, msg):
        if not self.initialized:
            return
        try:
            # Convert ROS Image message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode image from compressed format
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # # Convert ROS Image message to OpenCV image
            # cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if not self.first_received:
                rospy.loginfo("First image received for motion detection.")
                self.first_received = True
            # Convert to grayscale
            grayscale = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            grayscale = cv2.GaussianBlur(grayscale, (5, 5), 0)
            # Init self.bg, if necessary
            if self.bg is None:
                self.bg = grayscale.copy().astype("float")

            # Find difference with background
            delta = cv2.absdiff(grayscale, self.bg.astype("uint8"))
            # Generate contours
            thresh = cv2.threshold(delta, 20, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=4)
            contours = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0]
            cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)

            motion_area = 0
            for c in contours:
                area = cv2.contourArea(c)
                if area < 200:
                    continue
                motion_area += area
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(cv_image, (x,y), (x+w,y+h), (255,0,255), 2)
                cv2.putText(cv_image, "A: {}".format(area), (x,y+h), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            self.bg = cv2.accumulateWeighted(grayscale.astype("float"), self.bg, 0.5)

            area_msg = Float32()
            area_msg.data = motion_area
            self.area_pub.publish(area_msg)

            msg_out = DualImage()
            msg_out.image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            msg_out.image_raw = self.bridge.cv2_to_imgmsg(cv2.cvtColor(self.bg.astype("uint8"), cv2.COLOR_GRAY2BGR), encoding='bgr8')
            self.result_image_pub.publish(msg_out)
        except Exception as e:
            rospy.logerr("Error detecting motion: %s\n%s\n", e, traceback.format_exc())
        

if __name__ == '__main__':
    try:
        node = AlarmOmniNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass