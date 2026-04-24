import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    urdf_file_name = 'puzzlebot.urdf'
    urdf = os.path.join(
        get_package_share_directory('puzzlebot_sim'),
        'urdf',
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    static_transform_node = Node(
                                package='tf2_ros',
                                executable='static_transform_publisher',
                                arguments = ['--x', '2', '--y', '1', '--z', '0.0',
                                            '--yaw', '0.0', '--pitch', '0', '--roll', '0.0',
                                            '--frame-id', 'map', '--child-frame-id', 'odom']
                                )
    
    # Nodo estándar que publica el estado del robot (TF) basado en el URDF
    robot_state_pub_node = Node(
                            package='robot_state_publisher',
                            executable='robot_state_publisher',
                            name='robot_state_publisher',
                            output='screen',
                            parameters=[{'robot_description': robot_desc}],
                            arguments=[urdf]
                            )

    # Este nodo solo publica /pose_sim
    sim_node = Node(
                            package='puzzlebot_sim',
                            executable='kinematic_model', 
                            output='screen'
    )

    # rqt_plot
    rqt_plot_node = Node(
                            package='rqt_plot',
                            executable='rqt_plot',
                            name='rqt_plot',
                            arguments=['/pose/pose/position/x', '/pose/pose/position/y']     
    )

    # Configuración de la interfaz visual RViz2
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
                            on_exit=Shutdown(),
                    )
    
   # Crea una lista de manejadores: si 'robot_state_publisher' o 'puzzlebot_node' se cierran,
    # se lanza un evento de Shutdown para cerrar todo el sistema de una vez.
    shutdown_on_exit = [RegisterEventHandler(
                            OnProcessExit(
                                target_action=node,
                                on_exit=[
                                    LogInfo(msg=(EnvironmentVariable(name='USER'),
                                            ' exited the node')),
                                    EmitEvent(event=Shutdown(
                                        reason='Node Exited'))
                                ]
                            )
                        ) for node in [robot_state_pub_node, sim_node]
                    ]

    return LaunchDescription([
        static_transform_node,
        robot_state_pub_node,
        sim_node,
        rqt_plot_node,
        rviz_node,
        *shutdown_on_exit
    ])