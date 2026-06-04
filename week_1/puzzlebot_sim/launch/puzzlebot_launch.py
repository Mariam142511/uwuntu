from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # map -> odom  (estático)
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '1', '--y', '1', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    puzzlebot_node = Node(
        name='puzzlebot_publisher',
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        output='screen'
    )

    rviz_config = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'rviz',
        'puzzlebot_rviz.rviz'
    )

    rviz_node = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

    rqt_tf_tree_node = Node(
        name='rqt_tf_tree',
        package='rqt_tf_tree',
        executable='rqt_tf_tree'
    )

    return LaunchDescription([
        static_transform_node,
        puzzlebot_node,
        rviz_node,
        rqt_tf_tree_node
    ])