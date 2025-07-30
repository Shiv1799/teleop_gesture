from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1) USB camera + gesture node publishes:
        #      - raw images on `/camera/image_raw/uncompressed`
        #      - PoseStamped on `/left_hand_waypoint` & `/right_hand_waypoint`
        #      - gestures on `/left_gesture` & `/right_gesture`
        Node(
            package='gesture_recognizer',
            executable='usb_camera_node',
            name='gesture_camera',
            parameters=[{
                'device_id': 0,
                'frame_id': 'camera_frame',     # <-- matches our static TF below
                'flip_handedness': False,
                'visualization': True
            }]
        ),

        # 2) CV→MoveIt bridge takes those waypoints (now in 'camera_frame'),
        #    transforms them into 'world', and publishes `/left_arm_target` etc.
        Node(
            package='cv_franka_bridge_plus',
            executable='cv_franka_bridge',
            name='cv_franka_bridge',
            parameters=[{
                'fixed_z': 0.5,
                'kp': 0.3,
                'max_speed': 0.1,
                'workspace_scale': 0.001,
                # rotation_scale is declared in code with a default; override here if you like
            }]
        ),

        # 3) Servo control sends those targets into MoveIt Servo
        Node(
            package='dual_arm_servo',
            executable='servo_control_node',
            name='servo_control_node',
            parameters=[{
                'control_rate': 50.0,
                'linear_scale': 0.3,
                'rotational_scale': 0.2
            }]
        ),

        # 4) Visualization & planning in RViz
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('dual_arm_panda_moveit_config'),
                'launch', 'demo.launch.py'
            ]),
            launch_arguments={
                'use_rviz': 'true',
                'pipeline': 'ompl',
                'use_sim_time': 'false'
            }.items()
        ),

        # 5) Static TF: world → camera_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.5', '0', '1.0',        # X Y Z
                '-1.57', '0', '-1.57',    # R P Y
                'world', 'camera_frame'   # <-- must match `frame_id` above
            ]
        ),
    ])
