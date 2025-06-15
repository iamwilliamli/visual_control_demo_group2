#!/usr/bin/env python
import traceback
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
import numpy as np

from paul_alarm.msg import DualImage

SAVE_VIDEO = False

class DualImageSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.init_node('dual_images_subscriber_node', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("/motion/video", DualImage, self.callback)
        rospy.Subscriber("/motion/stable", Bool, self.stable_callback)
        self.is_stable = False
        self.out_video = None
        self.initialized = True
        rospy.loginfo("Dual Images Subscriber Node Initialized")

    def callback(self, msg):
        if not self.initialized:
            return
        try:
            # Convert ROS Image messages to OpenCV images
            image_raw = self.bridge.imgmsg_to_cv2(msg.image_raw, "bgr8")
            image_undistorted = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")

            # Add legends to the images
            cv2.putText(image_raw, "Background", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(image_undistorted, "Processed Image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(image_undistorted, "Stable" if self.is_stable else "Unstable", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            # Concatenate images horizontally for display
            # Resize images to the same height for proper concatenation
            height = min(image_raw.shape[0], image_undistorted.shape[0])
            image_raw_resized = cv2.resize(image_raw, (int(image_raw.shape[1] * height / image_raw.shape[0]), height))
            image_undistorted_resized = cv2.resize(image_undistorted, (int(image_undistorted.shape[1] * height / image_undistorted.shape[0]), height))

            # Concatenate images horizontally for display
            combined_image = np.hstack((image_raw_resized, image_undistorted_resized))

            if SAVE_VIDEO:
                if self.out_video is None:
                    # rospy.loginfo(dir(combined_image))
                    # rospy.loginfo("{}".format(combined_image.shape))
                    self.out_video = cv2.VideoWriter('output.avi',  
                        cv2.VideoWriter_fourcc(*'XVID'), 
                        10, (combined_image.shape[1], combined_image.shape[0]))
                self.out_video.write(combined_image) 

            # Display the combined image
            cv2.imshow("Dual Images", combined_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error processing image: %s\n%s\n", e, traceback.format_exc())
    
    def stable_callback(self, msg):
        if not self.initialized:
            return
        try:
            self.is_stable = msg.data
            rospy.loginfo("Received stable {}".format(msg.data))
        except Exception as e:
            rospy.logerr("Error: %s\n%s\n", e, traceback.format_exc())

        

if __name__ == '__main__':
    try:
        node = DualImageSubscriberNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        if node.out_video is not None:
            node.out_video.release()
        pass
    
    cv2.destroyAllWindows()