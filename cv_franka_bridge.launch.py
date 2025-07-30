from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the cv_franka_bridge node in cv_franka_bridge_plus package.
    """
    # Launch the bridge listener node
    bridge_node = Node(
        package='cv_franka_bridge_plus',
        executable='cv_franka_bridge',
        name='cv_franka_bridge',
        output='screen',
        parameters=[
            # You can override parameters here if needed
            # {'device_id': 0},
            # {'frame_id': 'camera_frame'},
            # {'topic': '/camera/image_raw/uncompressed'}
        ],
        remappings=[
            # Ensure topic names match the publishers
            ('waypoint', '/waypoint'),
            ('right_gesture', '/right_gesture'),
            ('left_gesture', '/left_gesture'),
        ]
    )

    return LaunchDescription([bridge_node])
