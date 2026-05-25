import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Diccionario con las metas fijas para cada escenario en Gazebo
WORLD_GOALS = {
    'obstacle_avoidance_1.world': (1.45,   1.20),
    'obstacle_avoidance_2.world': (-1.20,  1.50),
    'obstacle_avoidance_3.world': (0.00,  -2.50),
    'obstacle_avoidance_4.world': (0.00,  -2.45),
}
DEFAULT_WORLD = 'obstacle_avoidance_1.world'


def launch_setup(context, *args, **kwargs):
    # Obtener el mundo seleccionado desde la terminal y sus coordenadas meta
    world_name = LaunchConfiguration('world').perform(context)
    goal_x, goal_y = WORLD_GOALS.get(world_name, (1.45, 1.20))

    # Directorios de los paquetes (Usando tu paquete puzzlebot_sim)
    pkg_gazebo = get_package_share_directory('puzzlebot_gazebo')
    pkg_puzzlebot_sim = get_package_share_directory('puzzlebot_sim')
    
    # Ruta al URDF
    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(pkg_puzzlebot_sim, 'urdf', urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = f.read() if 'f' in locals() else "" 
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo_world_launch.py')
        ),
        launch_arguments={
            'world':     world_name,
            'pause':     'false',
            'verbosity': '1',
        }.items()
    )
    
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo_puzzlebot_launch.py')
        ),
        launch_arguments={
            'robot':        'puzzlebot_jetson_lidar_ed',
            'robot_name':   'puzzlebot',
            'x':            '0.0',
            'y':            '0.0',
            'yaw':          '0.0',
            'prefix':       '',
            'lidar_frame':  'laser_frame',
            'camera_frame': 'camera_link_optical',
            'tof_frame':    'tof_link',
            'use_sim_time': 'true',
        }.items()
    )

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    localisation_node = Node(
        package='puzzlebot_sim',
        executable='localisation',
        name='localisation_node',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('wl', '/VelocityEncL'),
            ('wr', '/VelocityEncR'),
        ],
        output='screen'
    )

    controllerBug2_node = Node(
        package='puzzlebot_sim',
        executable='controller_bug2',
        name='controllerBug2_node',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'use_sim_time': True
        }],
        remappings=[
            ('odom',    '/ground_truth'),     # Enlaza la odometría del mundo virtual
            ('cmd_vel', '/cmd_vel'),          # Envía velocidad directo a los motores de Gazebo
            ('scan',    '/scan'),             # Se suscribe al LiDAR real del Puzzlebot
        ]
    )

    return [
        gazebo_launch,
        robot_launch,
        static_transform_node,
        localisation_node,
        controllerBug2_node
    ]


def generate_launch_description():
    # Declaración del argumento para cambiar de mapa desde consola
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=DEFAULT_WORLD,
        description='Mundo a cargar en Gazebo (ej: obstacle_avoidance_1.world)'
    )
    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup),
    ])