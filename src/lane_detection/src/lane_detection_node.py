from ultralytics import YOLO
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
from jetson_camera.msg import PointArray

class LaneDetectionModel:
    def __init__(self, model_path: str):
        self.model = YOLO(model_path)
        self.model.fuse()  # Fuse model layers for faster inference
        self.xy_map = np.load('/home/hoangdung/study/embeded_visual_control/workshop3_2211025/src/lane_detection/data/xy_map_image.npy')
        print('xy_map:', self.xy_map)
        print("Model loaded successfully.")
        
    def predict(self, image):
        results = self.model.predict(image, conf=0.25, iou=0.45, agnostic_nms=True)
        return results
    
    def get_lane_lines(self, results):
        lane_lines = []
        middle_line = []
        for result in results:
            if result.masks is not None:
                masks = result.masks.data.cpu().numpy()  # (N, H, W)
                for mask in masks:
                    lane_lines.append(mask)
                    
        if lane_lines:
            # Combine all lane masks into a single binary mask
            combined_mask = np.sum(lane_lines, axis=0).astype(bool)
            prev_middle = None # To filter out break middle points
            # Calculate the middle line
            for row in reversed(range(combined_mask.shape[0])):
                if row < 150:
                    continue
                
                id = np.where(combined_mask[row, :])[0]
                if len(id) > 1:
                    min_x = np.min(id)
                    max_x = np.max(id)
                    if np.abs(max_x - min_x) < 150:  # Ensure there's a significant gap
                        continue
                    
                    middle = (min_x + max_x) // 2
                    if prev_middle is not None and abs(middle - prev_middle) > 50:
                        # Terminate when detection a break
                        break
                    prev_middle = middle
                    middle_line.append((row, middle))
        
        return lane_lines, middle_line

    def draw_lane_lines(self, image, lane_lines, middle_line):
        h, w = image.shape[:2]
        for lane in lane_lines:
            # Resize the mask to match the image shape
            resized_mask = cv2.resize(lane.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
            mask_bool = resized_mask.astype(bool)

            image[mask_bool] = [0, 255, 0]  # Green
            
        # For middle line
        middle_points = []
        resolution = 20  # Adjust resolution for middle line points
        resolution_count = -1
        
        if len(middle_line) > 10:
            for point in middle_line:
                resized_point = (int(point[1] * w / lane_lines[0].shape[1]), int(point[0] * h / lane_lines[0].shape[0]))
                cv2.circle(image, resized_point, 2, (0, 0, 255), -1)  # Red dots for middle line
                resolution_count += 1
                if resolution_count % resolution != 0:
                    continue
                
                row, col = resized_point
                # Ensure indices are within bounds
                if 0 <= row < self.xy_map.shape[0] and 0 <= col < self.xy_map.shape[1]:
                    x, y = self.xy_map[col, row][0], -self.xy_map[col, row][1]
                    cv2.putText(
                        image,
                        f"({x * 100:.2f}, {y * 100:.2f})",
                        (resized_point[0] + 5, resized_point[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        1,
                        cv2.LINE_AA
                    )
                    
                    print(f"Middle point: ({x:.2f}, {y:.2f}) at ({resized_point[0]}, {resized_point[1]})")
                    
                    middle_points.append(Point(x, y, 0.0))
                else:
                    x, y = None, None  # Out of bounds  
            
        return image, middle_points

class LaneDetectionNode:
    def __init__(self):
        rospy.init_node('lane_detection_node', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/image_undistorted', CompressedImage, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher('/lane_detection/image_with_lanes', Image, queue_size=1)
        self.bridge = CvBridge()
        self.is_running = False  # Flag to prevent multiple image processing
        
        self.target_pub = rospy.Publisher('/lane_detection/targets', PointArray, queue_size=1)
        
        # Load the lane detection model
        model_path = '/home/hoangdung/study/embeded_visual_control/workshop3_2211025/src/lane_detection/weight/best2.pt'  # Update with your model path
        print("Loading lane detection model...")
        self.model = LaneDetectionModel(model_path)
        
    def image_callback(self, msg):
        if self.is_running:
            rospy.logwarn("Image processing is already running. Ignoring new image.")
            return
        
        self.is_running = True
        try:
            # Convert ROS Image message to OpenCV image
            # Decompress the image if it's compressed
            if msg._type == "sensor_msgs/CompressedImage":
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            original_image = cv_image.copy()
            
            # Perform lane detection
            targets_msg = PointArray()
            results = self.model.predict(cv_image)
            lane_lines, middle_line = self.model.get_lane_lines(results)
                            
            # Draw lane lines on the image
            image_with_lanes, targets_msg.points = self.model.draw_lane_lines(cv_image, lane_lines, middle_line)
            self.target_pub.publish(targets_msg)
            
            # Convert OpenCV image back to ROS Image message
            ros_image_with_lanes = self.bridge.cv2_to_imgmsg(image_with_lanes, encoding='bgr8')
            
            # Publish the image with lane lines
            self.image_pub.publish(ros_image_with_lanes)
            
            combined_image = np.hstack((original_image, image_with_lanes))

            # Display the combined image
            if not hasattr(self, 'video_writer'):
                # Initialize video writer if not already done
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter('/home/hoangdung/study/embeded_visual_control/workshop3_2211025/lane_detection.avi', 
                                                    fourcc, 30.0, (combined_image.shape[1], combined_image.shape[0]))
            
            # Write the combined image to the video file
            print("Writing to video file...")
            self.video_writer.write(combined_image)
        
        except Exception as e:
            rospy.logerr("Failed to process image: %s", str(e))
            
        self.is_running = False
        
        
    def release_resources(self):
        # Release the video writer if it exists
        if hasattr(self, 'video_writer'):
            self.video_writer.release()
            rospy.loginfo("Video writer released.")
                
            
if __name__ == '__main__':
    try:
        lane_detection_node = LaneDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    cv2.destroyAllWindows()
    lane_detection_node.release_resources()