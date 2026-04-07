import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LocalSubstitution

def generate_launch_description():
    # Ruta al archivo URDF
    pkg_share = get_package_share_directory('puzzlebot_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')

    # Leer el contenido del URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Nodo: Robot State Publisher (Publica el árbol TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Nodo: Joint State Publisher GUI (Las barritas para mover ruedas)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Nodo: RVIZ2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Crear la descripción del launch agregando los nodos
    l_d = LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        # Log de aviso al cerrar
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(msg=['Cerrando la simulacion del Puzzlebot...'])]
            )
        )
    ])

    return l_d
