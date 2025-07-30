#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from rclpy.duration import Duration
import time

class SimDualArmBridge(Node):
    def __init__(self):
        super().__init__('sim_dual_arm_bridge')
        
        # Parameters
        self.declare_parameter('fixed_z', 0.5)
        self.declare_parameter('kp', 0.3)
        self.declare_parameter('max_speed', 0.1)
        self.declare_parameter('workspace_scale', 0.001)
        self.declare_parameter('rotation_scale', 0.01)
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('world_frame', 'world')  # Add world frame parameter
        
        # Get parameters
        self.fixed_z = self.get_parameter('fixed_z').value
        self.kp = self.get_parameter('kp').value
        self.max_speed = self.get_parameter('max_speed').value
        self.workspace_scale = self.get_parameter('workspace_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value  # Get world frame
        
        # TF for camera-robot transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Current states
        self.tracking_enabled = False
        self.left_waypoint = None
        self.right_waypoint = None
        
        # Publishers
        self.left_target_pub = self.create_publisher(PoseStamped, '/left_arm_target', 10)
        self.right_target_pub = self.create_publisher(PoseStamped, '/right_arm_target', 10)
        
        # Subscribers
        self.create_subscription(PoseStamped, 'left_hand_waypoint', self.left_waypoint_callback, 10)
        self.create_subscription(PoseStamped, 'right_hand_waypoint', self.right_waypoint_callback, 10)
        self.create_subscription(String, 'left_gesture', self.left_gesture_callback, 10)
        self.create_subscription(String, 'right_gesture', self.right_gesture_callback, 10)
        
        # Wait for world frame to be available
        self.world_frame_ready = False
        self.check_world_frame_timer = self.create_timer(1.0, self.check_world_frame)
        
        # Control timer (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("CV Franka Bridge initialized")

    def check_world_frame(self):
        """Check if world frame is available in TF"""
        try:
            # Check if transform is available
            if self.tf_buffer.can_transform(
                self.world_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            ):
                self.world_frame_ready = True
                self.get_logger().info(f"World frame '{self.world_frame}' is available")
                self.destroy_timer(self.check_world_frame_timer)
        except Exception as e:
            self.get_logger().warn(f"World frame check failed: {str(e)}")

    def transform_to_world(self, pose_camera):
        """Transform pose from camera frame to world frame"""
        if not self.world_frame_ready:
            self.get_logger().warn("World frame not ready, skipping transform")
            return None
            
        try:
            # Get transform with proper timing
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                pose_camera.header.frame_id,
                pose_camera.header.stamp,
                timeout=Duration(seconds=0.1)
            return tf2_geometry_msgs.do_transform_pose(pose_camera, transform)
        except Exception as e:
            self.get_logger().warn(f"TF error: {str(e)}")
            return None

    def left_gesture_callback(self, msg):
        if msg.data == "Thumb_Up":
            self.tracking_enabled = True
            self.get_logger().info("Thumb Up - STARTING TRACKING")
        elif msg.data == "Thumb_Down":
            self.tracking_enabled = False
            self.get_logger().info("Thumb Down - STOPPING TRACKING")

    def right_gesture_callback(self, msg):
        # Mirror left hand control logic
        self.left_gesture_callback(msg)

    def left_waypoint_callback(self, msg):
        if not self.world_frame_ready:
            return
            
        self.get_logger().debug(f"Received left waypoint with frame: {msg.header.frame_id}")
        transformed = self.transform_to_world(msg)
        if transformed:
            self.left_waypoint = transformed
        else:
            self.get_logger().debug("Transform failed for left waypoint")

    def right_waypoint_callback(self, msg):
        if not self.world_frame_ready:
            return
            
        self.get_logger().debug(f"Received right waypoint with frame: {msg.header.frame_id}")
        transformed = self.transform_to_world(msg)
        if transformed:
            self.right_waypoint = transformed
        else:
            self.get_logger().debug("Transform failed for right waypoint")

    def map_to_workspace(self, pixel_x, pixel_y):
        """Convert camera pixels to robot workspace"""
        robot_x = pixel_x * self.workspace_scale
        robot_y = pixel_y * self.workspace_scale
        return robot_x, robot_y

    def control_loop(self):
        if not self.world_frame_ready or not self.tracking_enabled:
            return
            
        # Process left arm
        if self.left_waypoint:
            target_x, target_y = self.map_to_workspace(
                self.left_waypoint.pose.position.x,
                self.left_waypoint.pose.position.y
            )
            self.publish_target_pose('left', target_x, target_y)
        
        # Process right arm
        if self.right_waypoint:
            target_x, target_y = self.map_to_workspace(
                self.right_waypoint.pose.position.x,
                self.right_waypoint.pose.position.y
            )
            self.publish_target_pose('right', target_x, target_y)

    def publish_target_pose(self, side, x, y):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.world_frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = self.fixed_z
        
        # Simple downward orientation
        pose_msg.pose.orientation.w = 0.707
        pose_msg.pose.orientation.x = 0.707
        
        if side == 'left':
            self.left_target_pub.publish(pose_msg)
        else:
            self.right_target_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimDualArmBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()