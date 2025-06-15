#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from jetson_camera.msg import DualImages
import numpy as np

class ImageUndistortionNode:
    def __init__(self):
        rospy.init_node('image_undistortion_node', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/image_raw', CompressedImage, self.image_callback)
        self.image_pub = rospy.Publisher('/camera/image_undistorted', CompressedImage, queue_size=1)
        self.dual_images_pub = rospy.Publisher('/dual_images', DualImages, queue_size=1)
        self.bridge = CvBridge()

        # Camera intrinsic parameters and distortion coefficients
        # Replace these with your camera's calibration data
        self.camera_matrix = np.array([[398.27014315, 0, 340.66012128],
                                       [0, 532.09476339, 193.68073413],
                                       [0,  0,  1]])
        self.dist_coeffs = np.array([-3.39305268e-01, 1.36928699e-01, 
                                     -1.06684742e-03, -6.72601258e-05, -2.88115370e-02])
                
        self.newcameramtx = None
        self.roi = None
        self.mapx = None
        self.mapy = None
        

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode image from compressed format
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Perform undistortion
            if self.newcameramtx is None:
                rospy.loginfo("First image received for undistortion.")
                h,  w = cv_image.shape[:2]
                self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(
                    self.camera_matrix, self.dist_coeffs, (w,h), 1, (w,h))
                self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                    self.camera_matrix, self.dist_coeffs, None, self.newcameramtx, (w,h), 5)
            
            dst = cv2.remap(cv_image, self.mapx, self.mapy, cv2.INTER_LINEAR)
            
            # crop the image
            x, y, w, h = self.roi
            dst = dst[y:y+h, x:x+w]

            # Encode undistorted image as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, encimg = cv2.imencode('.jpg', dst, encode_param)
            if not result:
                rospy.logerr("Failed to encode image!")
                return

            # Create and publish CompressedImage message
            comp_msg = CompressedImage()
            comp_msg.header = msg.header
            comp_msg.format = "jpeg"
            comp_msg.data = np.array(encimg).tobytes()
            self.image_pub.publish(comp_msg)
            
        except Exception as e:
            rospy.logerr("[Image processing] Error processing image: %s", e)
        

if __name__ == '__main__':
    try:
        node = ImageUndistortionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass