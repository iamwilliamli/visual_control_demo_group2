#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from jetson_camera.msg import DualImages

class DualImagesSubscriberNode:
    def __init__(self):
        rospy.init_node('dual_images_subscriber_node', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("dual_images", DualImages, self.callback)
        rospy.loginfo("Dual Images Subscriber Node Initialized")

    def callback(self, msg):
        try:
            # Convert ROS Image messages to OpenCV images
            image_raw = self.bridge.imgmsg_to_cv2(msg.image_raw, "bgr8")
            image_undistorted = self.bridge.imgmsg_to_cv2(msg.image_undistorted, "bgr8")

            # Add legends to the images
            cv2.putText(image_raw, "Raw Image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(image_undistorted, "Undistorted Image", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Concatenate images horizontally for display
            # Resize images to the same height for proper concatenation
            height = min(image_raw.shape[0], image_undistorted.shape[0])
            image_raw_resized = cv2.resize(image_raw, (int(image_raw.shape[1] * height / image_raw.shape[0]), height))
            image_undistorted_resized = cv2.resize(image_undistorted, (int(image_undistorted.shape[1] * height / image_undistorted.shape[0]), height))

            # Concatenate images horizontally for display
            combined_image = np.hstack((image_raw_resized, image_undistorted_resized))

            # Display the combined image
            if not hasattr(self, 'video_writer'):
                # Initialize video writer if not already done
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter('/home/hoangdung/study/embeded_visual_control/workshop2_2211025/undistortion.avi', 
                                                    fourcc, 30.0, (combined_image.shape[1], combined_image.shape[0]))
            
            # Write the combined image to the video file
            self.video_writer.write(combined_image)

        except Exception as e:
            rospy.logerr("Error processing images: %s", str(e))        
    
    def release_resources(self):
        # Release the video writer if it exists
        if hasattr(self, 'video_writer'):
            self.video_writer.release()
            rospy.loginfo("Video writer released.")

if __name__ == '__main__':
    try:
        node = DualImagesSubscriberNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    cv2.destroyAllWindows()
    node.release_resources()