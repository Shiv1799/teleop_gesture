#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from moveit_servo import Servo, ServoParameters
from moveit_servo.utilities import pose_to_eigen
import threading
import numpy as np

class DualArmServoControl(Node):
    def __init__(self):
        super().__init__('dual_arm_servo_control')
        
        # Parameters
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('rotational_scale', 0.3)
        self.declare_parameter('collision_check', True)
        
        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        linear_scale = self.get_parameter('linear_scale').value
        rotational_scale = self.get_parameter('rotational_scale').value
        collision_check = self.get_parameter('collision_check').value
        
        # Servo parameters
        self.servo_params = ServoParameters(self)
        self.servo_params.command_out_topic = "/joint_trajectory_controller/joint_trajectory"
        self.servo_params.scale.linear = linear_scale
        self.servo_params.scale.rotational = rotational_scale
        self.servo_params.collision_check = collision_check
        
        # Initialize servo for left arm
        self.left_servo = Servo(
            node=self,
            parameters=self.servo_params,
            planning_frame="panda_link0",
            move_group_name="left_panda_arm"
        )
        
        # Initialize servo for right arm
        self.right_servo = Servo(
            node=self,
            parameters=self.servo_params,
            planning_frame="panda_link0",
            move_group_name="right_panda_arm"
        )
        
        # Current target poses
        self.left_target = None
        self.right_target = None
        self.lock = threading.Lock()
        
        # Create subscribers with callback groups
        callback_group = ReentrantCallbackGroup()
        self.left_target_sub = self.create_subscription(
            PoseStamped,
            '/left_arm_target',
            self.left_target_callback,
            10,
            callback_group=callback_group
        )
        self.right_target_sub = self.create_subscription(
            PoseStamped,
            '/right_arm_target',
            self.right_target_callback,
            10,
            callback_group=callback_group
        )
        
        # Start control thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        self.get_logger().info("Dual Arm Servo Control initialized")

    def left_target_callback(self, msg):
        with self.lock:
            self.left_target = msg

    def right_target_callback(self, msg):
        with self.lock:
            self.right_target = msg

    def control_loop(self):
        rate = self.create_rate(self.control_rate)
        
        while rclpy.ok():
            # Process left arm
            if self.left_target:
                try:
                    # Convert to Eigen Isometry3d
                    target_pose = pose_to_eigen(self.left_target.pose)
                    
                    # Send to servo
                    self.left_servo.set_command(target_pose)
                except Exception as e:
                    self.get_logger().error(f"Left arm servo error: {str(e)}", throttle_duration_sec=1.0)
            
            # Process right arm
            if self.right_target:
                try:
                    # Convert to Eigen Isometry3d
                    target_pose = pose_to_eigen(self.right_target.pose)
                    
                    # Send to servo
                    self.right_servo.set_command(target_pose)
                except Exception as e:
                    self.get_logger().error(f"Right arm servo error: {str(e)}", throttle_duration_sec=1.0)
            
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = DualArmServoControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()