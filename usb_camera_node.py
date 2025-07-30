#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Header

class GestureCameraNode(Node):
    """Combined camera and gesture recognition node with camera_frame"""

    def __init__(self):
        super().__init__('gesture_camera_node')

        # Parameters - Ensure camera_frame is default
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('topic', '/camera/image_raw/uncompressed')
        self.declare_parameter('flip_handedness', False)  # Changed default to False
        self.declare_parameter('visualization', True)
        
        device_id = self.get_parameter('device_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.topic = self.get_parameter('topic').value
        self.flip_handedness = self.get_parameter('flip_handedness').value
        self.visualization = self.get_parameter('visualization').value
        
        # Validate frame_id
        if not self.frame_id:
            self.get_logger().error("frame_id parameter is empty!")
            rclpy.shutdown()
            return
        
        # OpenCV bridge and camera
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Unable to open camera device {device_id}")
            rclpy.shutdown()
            return

        # Initialize MediaPipe
        self.setup_mediapipe()
        
        # Visualization window
        if self.visualization:
            cv2.namedWindow('Gesture Camera', cv2.WINDOW_NORMAL)

        # Callback groups
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.processing_callback_group = MutuallyExclusiveCallbackGroup()

        # Publishers - CORRECTED TOPIC NAMES
        self.image_pub = self.create_publisher(Image, self.topic, 10, callback_group=self.timer_callback_group)
        self.annotated_pub = self.create_publisher(Image, 'cv_image', 10, callback_group=self.processing_callback_group)
        self.left_waypoint_pub = self.create_publisher(PoseStamped, 'left_hand_waypoint', 10, callback_group=self.processing_callback_group)  # FIXED
        self.right_waypoint_pub = self.create_publisher(PoseStamped, 'right_hand_waypoint', 10, callback_group=self.processing_callback_group)  # FIXED
        self.left_gesture_pub = self.create_publisher(String, 'left_gesture', 10, callback_group=self.processing_callback_group)
        self.right_gesture_pub = self.create_publisher(String, 'right_gesture', 10, callback_group=self.processing_callback_group)
        self.hand_data_pub = self.create_publisher(String, '/hand_data', 10, callback_group=self.processing_callback_group)

        # Timer for processing
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame, callback_group=self.timer_callback_group)

        self.get_logger().info(f"Gesture Camera Node started with frame_id: {self.frame_id}")

    def setup_mediapipe(self):
        """Initialize MediaPipe components"""
        # MediaPipe Hands for tracking
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,  # Increased for stability
            min_tracking_confidence=0.7
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # MediaPipe Gesture Recognition
        try:
            package_name = 'gesture_recognizer'
            model_path = os.path.join(
                get_package_share_directory(package_name),
                'gesture_recognizer.task'
            )
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"Model file not found at {model_path}")
                return
            
            base_options = python.BaseOptions(model_asset_path=model_path)
            options = vision.GestureRecognizerOptions(
                base_options=base_options,
                running_mode=vision.RunningMode.VIDEO,
                num_hands=2
            )
            
            self.recognizer = vision.GestureRecognizer.create_from_options(options)
            self.get_logger().info("MediaPipe Gesture Recognizer initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error initializing MediaPipe: {str(e)}")
            
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture image')
            return
        
        # Create proper header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        
        # Publish raw image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header = header
        self.image_pub.publish(img_msg)
        
        # Process frame
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.process_hands(rgb, frame, header)
        
        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = header
        self.annotated_pub.publish(out_msg)
        
        # Show visualization
        if self.visualization:
            cv2.imshow('Gesture Camera', frame)
            cv2.waitKey(1)

    def process_hands(self, rgb_image, cv_image, header):
        """Detect hands, gestures, and publish data"""
        # Process hand landmarks
        hands_results = self.hands.process(rgb_image)
        
        # Process gestures
        gesture_results = None
        if hasattr(self, 'recognizer'):
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
            timestamp = int(self.get_clock().now().nanoseconds / 1e6)
            gesture_results = self.recognizer.recognize_for_video(mp_image, timestamp)
        
        # Default values
        left_gesture = 'None'
        right_gesture = 'None'
        left_hand = None
        right_hand = None
        hand_data = String()
        gesture_info = []
        
        # Process hand landmarks for waypoints
        if hands_results.multi_hand_landmarks and hands_results.multi_handedness:
            for hand_landmarks, handedness in zip(
                hands_results.multi_hand_landmarks,
                hands_results.multi_handedness
            ):
                # Draw landmarks
                if self.visualization:
                    self.mp_drawing.draw_landmarks(
                        cv_image,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS
                    )

                # Get handedness classification
                label = handedness.classification[0].label
                confidence = handedness.classification[0].score
                
                # CORRECTED HANDEDNESS FLIPPING
                if self.flip_handedness:
                    label = "Left" if label == "Right" else "Right"
                
                # Compute centroid
                key_idxs = [0, 5, 8, 9, 13, 17]
                h, w, _ = cv_image.shape
                coords = np.array([
                    [lm.x * w, lm.y * h]
                    for i, lm in enumerate(hand_landmarks.landmark)
                    if i in key_idxs
                ])
                centroid = coords.mean(axis=0)
                
                # Store hand data
                if label == 'Left':
                    left_hand = centroid
                elif label == 'Right':
                    right_hand = centroid
        
        # Process gesture recognition results
        if gesture_results and gesture_results.gestures:
            for idx, gesture_list in enumerate(gesture_results.gestures):
                if gesture_list:
                    # Get gesture information
                    gesture = gesture_list[0].category_name
                    score = gesture_list[0].score
                    
                    # Get handedness
                    handedness = "Unknown"
                    if idx < len(gesture_results.handedness) and gesture_results.handedness[idx]:
                        handedness = gesture_results.handedness[idx][0].category_name
                    
                    # Update gesture strings
                    if handedness == 'Left':
                        left_gesture = gesture
                    elif handedness == 'Right':
                        right_gesture = gesture
                    
                    # Store hand information
                    gesture_info.append(f"{handedness}:{gesture}:{score:.2f}")
                    
                    # Visualize results
                    if self.visualization:
                        self.visualize_detection(cv_image, gesture_results, idx, handedness, gesture, score)
        
        # Create hand data message
        if gesture_info:
            hand_data.data = "|".join(gesture_info)
        else:
            hand_data.data = "No hands detected"
        
        # Publish hand data
        self.hand_data_pub.publish(hand_data)
        
        # Publish waypoints
        if left_hand is not None:
            waypoint = PoseStamped()
            waypoint.header = header
            waypoint.pose.position.x = float(left_hand[0])
            waypoint.pose.position.y = float(left_hand[1])
            waypoint.pose.position.z = 0.0
            self.left_waypoint_pub.publish(waypoint)
            self.get_logger().debug(f"Published left waypoint: {left_hand}", throttle_duration_sec=1.0)

        if right_hand is not None:
            waypoint = PoseStamped()
            waypoint.header = header
            waypoint.pose.position.x = float(right_hand[0])
            waypoint.pose.position.y = float(right_hand[1])
            waypoint.pose.position.z = 0.0
            self.right_waypoint_pub.publish(waypoint)
            self.get_logger().debug(f"Published right waypoint: {right_hand}", throttle_duration_sec=1.0)

        # Publish gestures
        self.left_gesture_pub.publish(String(data=left_gesture))
        self.right_gesture_pub.publish(String(data=right_gesture))

    def visualize_detection(self, image, result, idx, handedness, gesture, score):
        """Visualize gesture detection on the image"""
        try:
            # Get hand landmarks
            landmarks = result.hand_landmarks[idx]
            
            # Draw landmarks
            for landmark in landmarks:
                x = int(landmark.x * image.shape[1])
                y = int(landmark.y * image.shape[0])
                cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
            
            # Draw connections between landmarks
            self.draw_hand_connections(image, landmarks)
            
            # Position text at hand base
            x = int(landmarks[0].x * image.shape[1])
            y = int(landmarks[0].y * image.shape[0])
            
            # Create label text
            label = f"{handedness}: {gesture} ({score:.2f})"
            
            # Draw text background
            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            cv2.rectangle(image, (x, y - 30), (x + text_size[0], y), (0, 0, 0), -1)
            
            # Draw text
            cv2.putText(
                image, label, (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )
            
        except Exception as e:
            self.get_logger().warn(f"Visualization error: {str(e)}")

    def draw_hand_connections(self, image, landmarks):
        """Draw connections between hand landmarks"""
        # Define connections between landmarks
        connections = [
            (0, 1), (1, 2), (2, 3), (3, 4),         # Thumb
            (0, 5), (5, 6), (6, 7), (7, 8),         # Index finger
            (0, 9), (9, 10), (10, 11), (11, 12),    # Middle finger
            (0, 13), (13, 14), (14, 15), (15, 16),  # Ring finger
            (0, 17), (17, 18), (18, 19), (19, 20)   # Pinky
        ]
        
        # Draw connections
        for connection in connections:
            start_idx, end_idx = connection
            start_point = (
                int(landmarks[start_idx].x * image.shape[1]),
                int(landmarks[start_idx].y * image.shape[0])
            )
            end_point = (
                int(landmarks[end_idx].x * image.shape[1]),
                int(landmarks[end_idx].y * image.shape[0])
            )
            cv2.line(image, start_point, end_point, (255, 0, 0), 2)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        self.hands.close()
        if hasattr(self, 'recognizer'):
            self.recognizer.close()
        if self.visualization:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GestureCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()