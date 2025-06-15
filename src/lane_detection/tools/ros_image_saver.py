#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver:
    def __init__(self):
        rospy.init_node('ros_image_saver', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/image_undistorted', Image, self.image_callback)
        self.bridge = CvBridge()
        self.save_path = os.path.join(os.path.dirname(__file__), '../data')
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        self.image_count = 0
        rospy.loginfo("Image saver node initialized. Saving images to: %s", self.save_path)

    def image_callback(self, msg):
        try:
            if self.image_count % 10 == 0:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                file_name = os.path.join(self.save_path, f'image_{self.image_count:04d}.jpg')
                cv2.imwrite(file_name, cv_image)
                rospy.loginfo("Saved image: %s", file_name)
            self.image_count += 1
        except Exception as e:
            rospy.logerr("Failed to save image: %s", str(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_saver = ImageSaver()
        image_saver.run()
    except rospy.ROSInterruptException:
        pass