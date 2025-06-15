#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import easyocr
import os
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class OCRNode(object):
    def __init__(self):
        rospy.init_node('ocr_node', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/image_undistorted_william", CompressedImage, self.callback)
        self.text_pub = rospy.Publisher("/ocr/text", String, queue_size=10)
        
        # Initialize EasyOCR reader
        self.reader = easyocr.Reader(['en'], gpu=True)
        rospy.loginfo("EasyOCR reader initialized")
        
        # Create thread pool for OCR processing
        self.executor = ThreadPoolExecutor(max_workers=2)
        
        # Lock for thread-safe operations
        self.processing_lock = threading.Lock()
        self.is_processing = False
        
        # For display purposes
        self.latest_annotated_image = None
        self.display_lock = threading.Lock()
        
        # Create ocr_annotated directory in the same directory as this script
        current_file_directory = os.path.dirname(os.path.abspath(__file__))
        self.ocr_annotated_dir = os.path.join(current_file_directory, "ocr_annotated")
        try:
            os.makedirs(self.ocr_annotated_dir)
        except OSError:
            if not os.path.isdir(self.ocr_annotated_dir):
                raise
        
        rospy.loginfo("OCR Node is running...")
        
        # Start display timer
        self.display_timer = rospy.Timer(rospy.Duration(0.1), self.display_callback)

    def callback(self, msg):
        # Check if OCR is already processing to avoid queue buildup
        with self.processing_lock:
            if self.is_processing:
                return  # Skip this frame if still processing previous one
            self.is_processing = True
        
        try:
            # Convert the undistorted image from ROS to OpenCV
            #  Convert ROS Image message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode image from compressed format
            image_undistorted = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # image_undistorted = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Submit OCR processing to thread pool (non-blocking)
            self.executor.submit(self.process_ocr, image_undistorted.copy())
            
        except CvBridgeError as e:
            rospy.logerr("CVBridge error: %s", str(e))
            with self.processing_lock:
                self.is_processing = False
        except Exception as e:
            rospy.logerr("Callback error: %s", str(e))
            with self.processing_lock:
                self.is_processing = False

    def process_ocr(self, image):
        """Process OCR in a separate thread"""
        try:
            annotated_image = image.copy()

            # OCR with EasyOCR
            result = self.reader.readtext(annotated_image)
            full_text = []

            for (bbox, text, confidence) in result:
                # bbox = [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
                pts = np.array(bbox, dtype=np.int32)

                # draw the four-point polygon
                cv2.polylines(annotated_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                # pick upper-left corner for the label
                x, y = pts[0]
                label = f'{text} ({confidence:.2f})'
                
                # Handle Unicode characters in OpenCV's putText
                try:
                    cv2.putText(annotated_image, label, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 0, 0), 2, cv2.LINE_AA)
                except:
                    # If putText fails with Unicode, use ASCII replacement
                    safe_label = label.encode('ascii', 'replace').decode('ascii')
                    cv2.putText(annotated_image, safe_label, (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 0, 0), 2, cv2.LINE_AA)
                
                full_text.append(text)

            # Join all text into one string
            final_text = " ".join(full_text)
            
            safe_text = final_text
            
            # Log safely
            try:
                rospy.loginfo("OCR Detected: %s", safe_text)
            except UnicodeEncodeError:
                rospy.loginfo("OCR Detected text (with non-ASCII characters)")
            
            # Publish as ROS message
            if safe_text == "":
                safe_text = "No text detected"
            self.text_pub.publish(safe_text)
            rospy.loginfo("Published OCR text: %s", safe_text)

            # Save the annotated image
            timestamp = time.time()
            timestamp_str = "{:.6f}".format(timestamp).replace('.', '_')
            annotated_image_path = os.path.join(self.ocr_annotated_dir, "ocr_annotated_{}.jpg".format(timestamp_str))
            cv2.imwrite(annotated_image_path, annotated_image)
            
            # Update latest annotated image for display
            with self.display_lock:
                self.latest_annotated_image = annotated_image.copy()

        except Exception as e:
            rospy.logerr("OCR processing error: %s", str(e))
        finally:
            # Mark processing as complete
            with self.processing_lock:
                self.is_processing = False

    def display_callback(self, event):
        """Display the latest annotated image in the main thread"""
        with self.display_lock:
            if self.latest_annotated_image is not None:
                cv2.imshow("Annotated Image", self.latest_annotated_image)
                cv2.waitKey(1)

if __name__ == '__main__':
    try:
        OCRNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()