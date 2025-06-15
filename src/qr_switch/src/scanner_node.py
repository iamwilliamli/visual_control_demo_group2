#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from pyzbar import pyzbar
from coordinator.msg import GoToMode, NextMode
import numpy as np

MODE_IDS = {
    "Lane_Follow": 0,
    "Count_People": 1,
    "Motion_Detect": 2,
    "QR_mode": 3,
}

NODE_NAME = "scanner_node"
NODE_ID = 3

class ScannerNode:
    """
    This node subscribes to an image topic, scans for QR codes, and publishes
    mode-change commands based on the data decoded from the QR codes.
    """
    def __init__(self):
        rospy.loginfo("Initializing {}...".format(NODE_NAME))
        rospy.init_node(NODE_NAME, anonymous=False, xmlrpc_port=45206, tcpros_port=45207)

        self.bridge = CvBridge()
        self.goto_pub = rospy.Publisher("/co/goto", GoToMode, queue_size=10)
        self.next_pub = rospy.Publisher("/co/next", NextMode, queue_size=10)

        self.image_sub = rospy.Subscriber(
            "/camera/image_undistorted_paul", 
            CompressedImage, 
            self.image_callback, 
            queue_size=1, 
            buff_size=2**24
        )
        
        # self.last_qr_data = None
        # self.last_qr_time = rospy.Time.now()

        rospy.loginfo("{} initialized. Subscribing to /camera/image_undistorted.".format(NODE_NAME))

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode image from compressed format
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        qrcodes = pyzbar.decode(cv_image)

        if not qrcodes:
            return

        qr = qrcodes[0]
        qr_data = qr.data.decode("utf-8")

        # if qr_data == self.last_qr_data and (rospy.Time.now() - self.last_qr_time).to_sec() < 2.0:
        #     return
            
        # self.last_qr_data = qr_data
        # self.last_qr_time = rospy.Time.now()

        rospy.loginfo("Detected QR code with data: '{}'".format(qr_data))
        self.process_qr_command(qr_data)

    def process_qr_command(self, command):
        """
        Parses the command from the QR code and publishes the appropriate message.
        Expected formats:
        - "next" -> Publishes to /co/next
        - "goto:[mode_name]" (e.g., "goto:Lane_Follow") -> Publishes to /co/goto
        - "goto:[mode_id]" (e.g., "goto:1") -> Publishes to /co/goto
        """
        command = command.strip().lower()

        if command == "next":
            msg = NextMode()
            msg.me = NODE_ID
            self.next_pub.publish(msg)
            rospy.loginfo("Published 'NextMode' command.")

        elif command.startswith("goto:"):
            target_str = command.split(":", 1)[1].strip()
            target_mode_id = -1

            if target_str.isdigit():
                target_mode_id = int(target_str)
            else:
                target_name = '_'.join(word.capitalize() for word in target_str.split('_'))
                if target_name in MODE_IDS:
                    target_mode_id = MODE_IDS[target_name]

            if target_mode_id != -1:
                msg = GoToMode()
                msg.me = NODE_ID
                msg.go_to_this_mode = target_mode_id
                self.goto_pub.publish(msg)
                rospy.loginfo("Published 'GoToMode' command for mode ID {}.".format(target_mode_id))
            else:
                rospy.logwarn("Invalid mode '{}' requested in QR code.".format(target_str))
        else:
            rospy.logwarn("Unrecognized command '{}' from QR code.".format(command))

if __name__ == '__main__':
    try:
        scanner_node = ScannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("{} shutting down.".format(NODE_NAME))
    finally:
        cv2.destroyAllWindows()