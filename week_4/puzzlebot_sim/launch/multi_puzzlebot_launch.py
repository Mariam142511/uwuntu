import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot 1: group1
    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='group1',
        parameters=[{
            'frame_prefix': 'group1/',
            'robot_description': robot_desc
            }]
    )

    robot1_localisation = Node(
        package='puzzlebot_sim',
        executable='localisation',
        namespace='group1'
    )

    robot1_controller = Node(
        package='puzzlebot_sim',
        executable='controller', 
        namespace='group1'
    )

    robot1_node = Node(
        name='puzzlebot',
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        namespace='group1',
        parameters=[{
                    'init_pose_x':2.0,
                    'init_pose_y': 2.0,
                    'init_pose_z': 1.0,
                    'init_pose_yaw': 1.57,
                    'init_pose_pitch': 0.0,
                    'init_pose_roll': 0.0,
                    'odom_frame':'odom'
                }]
            )
    
    # Robot 2: group2  
    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='group2',
        parameters=[{
            'frame_prefix': 'group2/',
            'robot_description': robot_desc
            }]
    )

    robot2_localisation = Node(
        package='puzzlebot_sim',
        executable='localisation',
        namespace='group2'
    )

    robot2_controller = Node(
        package='puzzlebot_sim',
        executable='controller',
        namespace='group2'
    )

    robot2_node = Node(
        name='puzzlebot',
        package='puzzlebot_sim',
        executable='joint_state_publisher',
        namespace='group2',
        parameters=[{
                    'init_pose_x':-2.0,
                    'init_pose_y': 2.0,
                    'init_pose_z': 1.0,
                    'init_pose_yaw': 1.57,
                    'init_pose_pitch': 0.0,
                    'init_pose_roll': 0.0,
                    'odom_frame':'odom'
                }]
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
                    arguments=['-d', rviz_config],
                    )

    rqt_tf_tree_node = Node(name='rqt_tf_tree',
                    package='rqt_tf_tree',
                    executable='rqt_tf_tree'
                    )
    
    rqt_graph_node = Node(name='rqt_graph',
                    package='rqt_graph',
                    executable='rqt_graph'
                    )
    
    rqt_plot_node = Node(
                            package='rqt_plot',
                            executable='rqt_plot',
                            name='rqt_plot',
                            arguments=['/pose/pose/position/x', '/odom/pose/pose/position/x', '/pose/pose/position/y', '/odom/pose/pose/position/y']     
    )
    
    return LaunchDescription([
        robot1_state_pub,
        robot1_localisation,
        robot1_controller, 
        robot1_node,
        robot2_state_pub,
        robot2_localisation,
        robot2_controller,  
        robot2_node,
        rviz_node,
        rqt_tf_tree_node,
        rqt_graph_node,
        rqt_plot_node
    ])