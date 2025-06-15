#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math
import time
import random

from people_count.msg import MotorCommand, MotorCommandStatus
from sensor_msgs.msg import Image, CompressedImage
from coordinator.msg import CurrentMode

NODE_ID = 1

class TrackedObject:
    def __init__(self, object_id, centroid):
        self.id = object_id
        self.centroid = centroid
        self.last_seen = time.time()
        self.history = [centroid]

class DualImagesSubscriberNode:
    def __init__(self):
        self.initialized = False
        rospy.init_node('dual_images_subscriber_node_alex', anonymous=False, xmlrpc_port=45204, tcpros_port=45205)
        self.bridge = CvBridge()

        self.image_sub = None
        self.motor_pub = rospy.Publisher("/motor/endpoint", MotorCommand, queue_size=1)
        self.motor_status_sub = rospy.Subscriber("/motor/results", MotorCommandStatus, self.motor_status_callback)
        self.mode_sub = rospy.Subscriber("/co/current", CurrentMode, self.mode_callback)

        self.command_completed = True
        self.last_motor_command = None
        self.last_command_id = 0
        self.current_mode = 0

        self.operational_speed = 0.5

        self.bg_subtractor = cv2.createBackgroundSubtractorKNN(history=1000, dist2Threshold=400, detectShadows=False)
        self.previous_frame = None

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.hog_threshold = 0.6

        self.tracked_people = {}
        self.next_id = 1
        self.max_distance = 50
        self.expiry_time = 0.8

        self.initialized = True
        rospy.loginfo("Dual Images Subscriber Node Initialized")

    def mode_callback(self, msg):
        if not self.initialized:
            return
        self.current_mode = msg.current
        if self.current_mode == NODE_ID:
            self.image_sub = rospy.Subscriber("/camera/image_undistorted_paul", CompressedImage, self.callback)
        else:
            if self.image_sub is not None:
                self.image_sub.unregister()
                self.image_sub = None

    def motor_status_callback(self, msg):
        rospy.loginfo("Motor status received: command id=%d, completion=%.2f" % (msg.command.id, msg.completion))
        if msg.command.id == self.last_command_id:
            self.command_completed = (msg.completion >= 1.0)

    def filter_contours(self, contours):
        filtered = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h if h else 0
            solidity = float(area) / (w * h) if w * h else 0
            min_area_threshold = 800 if h < 120 else 1200
            if area > min_area_threshold and 0.2 < aspect_ratio < 0.8 and solidity > 0.3 and h > 60 and w > 30:
                filtered.append((x, y, w, h))
        return filtered

    def merge_contours(self, contours):
        if not contours:
            return []
        combined = []
        contours.sort(key=lambda c: c[0])
        for (x, y, w, h) in contours:
            merged = False
            for i in range(len(combined)):
                x2, y2, w2, h2 = combined[i]
                overlap_x = max(0, min(x + w, x2 + w2) - max(x, x2))
                overlap_y = max(0, min(y + h, y2 + h2) - max(y, y2))
                if overlap_x * overlap_y > 0 or (abs(x - x2) < 30 and abs(y - y2) < 30):
                    new_x = min(x, x2)
                    new_y = min(y, y2)
                    new_w = max(x + w, x2 + w2) - new_x
                    new_h = max(y + h, y2 + h2) - new_y
                    combined[i] = (new_x, new_y, new_w, new_h)
                    merged = True
                    break
            if not merged:
                combined.append((x, y, w, h))
        return combined

    def compute_centroid(self, x, y, w, h):
        return (int(x + w / 2), int(y + h / 2))

    def update_tracked_people(self, detections):
        current_time = time.time()
        current_centroids = [self.compute_centroid(x, y, w, h) for x, y, w, h in detections]
        matched_ids = []
        for i, centroid in enumerate(current_centroids):
            best_match_id = -1
            min_dist = self.max_distance
            for obj_id, person in self.tracked_people.items():
                dist = math.hypot(centroid[0] - person.centroid[0], centroid[1] - person.centroid[1])
                if dist < min_dist:
                    best_match_id = obj_id
                    min_dist = dist
            if best_match_id != -1:
                self.tracked_people[best_match_id].centroid = centroid
                self.tracked_people[best_match_id].last_seen = current_time
                self.tracked_people[best_match_id].history.append(centroid)
                if len(self.tracked_people[best_match_id].history) > 10:
                    self.tracked_people[best_match_id].history.pop(0)
                matched_ids.append(best_match_id)
                current_centroids[i] = None
        for centroid in current_centroids:
            if centroid:
                self.tracked_people[self.next_id] = TrackedObject(self.next_id, centroid)
                matched_ids.append(self.next_id)
                self.next_id += 1
        self.tracked_people = {obj_id: p for obj_id, p in self.tracked_people.items() if current_time - p.last_seen <= self.expiry_time}
        return len(self.tracked_people)

    def callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode image from compressed format
            image_undistorted = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # image_undistorted = self.bridge.imgmsg_to_cv2(msg.data, "bgr8")

            gray = cv2.cvtColor(cv2.convertScaleAbs(image_undistorted, alpha=1.2, beta=10), cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)

            fg_mask = self.bg_subtractor.apply(gray)
            _, fg_mask = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)

            fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
            fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))

            if self.previous_frame is not None:
                motion = cv2.absdiff(self.previous_frame, gray)
                _, motion_mask = cv2.threshold(motion, 30, 255, cv2.THRESH_BINARY)
                motion_mask = cv2.dilate(motion_mask, None, iterations=2)
                fg_mask = cv2.bitwise_or(fg_mask, motion_mask)

            self.previous_frame = gray.copy()

            contours_result = cv2.findContours(fg_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours_result[1] if len(contours_result) == 3 else contours_result[0]
            bs_detections = self.filter_contours(contours)

            hog_rects, hog_weights = self.hog.detectMultiScale(gray, winStride=(8, 8), padding=(16, 16), scale=1.05,
                                                                finalThreshold=1.5, hitThreshold=self.hog_threshold)
            all_detections = bs_detections[:]
            for i, weight in enumerate(hog_weights):
                if weight > self.hog_threshold + 0.2:
                    all_detections.append(hog_rects[i])

            people_rects = self.merge_contours(all_detections)
            people_count = self.update_tracked_people(people_rects)

            for (x, y, w, h) in people_rects:
                cv2.rectangle(image_undistorted, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if self.command_completed and people_count > 0 and self.current_mode == NODE_ID:
                motor_cmd = MotorCommand()
                self.last_command_id += 1
                motor_cmd.id = self.last_command_id
                motor_cmd.speed = self.operational_speed
                motor_cmd.do_immediately = True
                motor_cmd.forward = 0.0
                motor_cmd.right = random.uniform(-180.0, 180.0)  # Random turn

                rospy.loginfo("Publishing RANDOM TURN Command: id=%d, right=%.2f" % (motor_cmd.id, motor_cmd.right))
                self.motor_pub.publish(motor_cmd)
                self.command_completed = False
                self.last_motor_command = motor_cmd

            display_height = 480

            cv2.putText(image_undistorted, "People Count: %d" % people_count, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("Dual Images", image_undistorted)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s" % e)
        except Exception as e:
            rospy.logerr("Exception in callback: %s" % e)

    def cleanup(self):
        rospy.loginfo("Shutting down node, cleaning up...")
        cv2.destroyAllWindows()
        

if __name__ == '__main__':
    try:
        node = DualImagesSubscriberNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()
