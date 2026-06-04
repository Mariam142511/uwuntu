import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

DEFAULT_WORLD = 'puzzlebot_office.world'


def launch_setup(context, *args, **kwargs):
    world_name  = LaunchConfiguration('world').perform(context)
    pkg_gazebo  = get_package_share_directory('puzzlebot_gazebo')
    pkg_sim     = get_package_share_directory('puzzlebot_sim')
    params_file = os.path.join(pkg_sim, 'config', 'params_office.yaml')

    # ── URDF propio (sin puzzlebot_description) ───────────────
    urdf_path = os.path.join(pkg_sim, 'urdf', 'puzzlebot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # ── Gazebo Sim: cargar el mundo ───────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo_world_launch.py')),
        launch_arguments={
            'world':     world_name,
            'pause':     'false',
            'verbosity': '1',
        }.items()
    )

    # ── robot_state_publisher con URDF propio ─────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
        }]
    )

    # ── Spawnear el robot en Gazebo ───────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='robot_spawner',
        arguments=[
            '-name',  'puzzlebot',
            '-topic', 'robot_description',
            '-x', '-2.50',
            '-y', '-2.50',
            '-Y', '0.00',
        ],
        output='screen'
    )

    # ── Bridges Gazebo → ROS ──────────────────────────────────
    # Traduce tópicos de Gazebo a mensajes ROS estándar
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robot_bridge',
        arguments=[
            '/VelocityEncL@std_msgs/msg/Float32[gz.msgs.Float',
            '/VelocityEncR@std_msgs/msg/Float32[gz.msgs.Float',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen'
    )

    # ── Bridge de imagen RGB ──────────────────────────────────
    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=['/camera'],
        output='screen'
    )

    # ── Transformadas estáticas: world → map → odom ───────────
    static_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'world', '--child-frame-id', 'map']
    )

    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    # ── Joint state publisher: ruedas + TF estáticas chasis ──
    joint_state_publisher_node = Node(
    package='puzzlebot_sim',
    executable='joint_state_publisher',
    name='puzzlebot_custom_publisher',
    output='screen',
    parameters=[{'use_sim_time': True}]   # ← agregar esto
    )

    # ── EKF: fusión encoders + ArUco → /odom + TF dinámico ───
    localisation_node = Node(
    package='puzzlebot_sim',
    executable='localisation',
    name='localisation_node',
    parameters=[params_file, {'use_sim_time': True}],
    output='screen'
    )

    # ── Generador de waypoints ────────────────────────────────
    trajectory_generator_node = Node(
        package='puzzlebot_sim',
        executable='trajectory_generator',
        name='trajectory_generator',
        parameters=[params_file],
        output='screen'
    )

    # ── Controlador Bug0 ─────────────────────────────────────
    controller_node = Node(
        package='puzzlebot_sim',
        executable='controller_bug0',
        name='controllerBug0_node',
        parameters=[params_file],
        output='screen'
    )

    # ── Detector ArUco ────────────────────────────────────────
    aruco_detector_node = Node(
        package='puzzlebot_sim',
        executable='aruco_detector',
        name='aruco_detector_node',
        parameters=[params_file],
        output='screen'
    )

    # ── Visualización ─────────────────────────────────────────
    rviz_config = os.path.join(pkg_sim, 'rviz', 'puzzlebot_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    rqt_image_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_node',
        output='screen'
    )

    rqt_graph_node = Node(
        name='rqt_graph',
        package='rqt_graph',
        executable='rqt_graph'
    )

    return [
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        gz_image_bridge,
        static_world_map,
        static_map_odom,
        joint_state_publisher_node,
        localisation_node,
        trajectory_generator_node,
        controller_node,
        aruco_detector_node,
        rqt_image_node,
        rqt_graph_node,
        rviz_node,
    ]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=DEFAULT_WORLD,
        description='Mundo a cargar en Gazebo'
    )
    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup),
    ])