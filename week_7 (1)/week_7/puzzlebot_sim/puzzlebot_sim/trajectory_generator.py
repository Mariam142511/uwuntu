import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class TrajectoryGenerator(Node):
    """
    Nodo generador de trayectoria para el Puzzlebot.

    Publica waypoints secuenciales al controlador Bug0.
    Avanza al siguiente waypoint cuando recibe confirmación
    de meta alcanzada en el tópico /next_point.

    Suscripciones:
      - /next_point (Bool)  : señal del controller indicando meta alcanzada

    Publicaciones:
      - /set_point (Point)  : waypoint activo (x, y, z=theta)
    """

    def __init__(self):
        super().__init__('trajectory_generator')

        # ── Parámetros de waypoints ───────────────────────────
        self.declare_parameter('waypoint1_x',      0.0)
        self.declare_parameter('waypoint1_y',      0.0)
        self.declare_parameter('waypoint1_theta',  0.0)
        self.declare_parameter('waypoint2_x',      0.0)
        self.declare_parameter('waypoint2_y',      0.0)
        self.declare_parameter('waypoint2_theta',  0.0)
        self.declare_parameter('waypoint3_x',      0.0)
        self.declare_parameter('waypoint3_y',      0.0)
        self.declare_parameter('waypoint3_theta',  0.0)
        self.declare_parameter('goal_x',           2.50)
        self.declare_parameter('goal_y',           2.50)
        self.declare_parameter('goal_theta',       0.0)

        # ── Construcción de la lista de waypoints ─────────────
        # Cada waypoint es (x, y, theta) — theta se pasa en z del Point
        self.waypoints = [
            (self.get_parameter('waypoint1_x').value,
             self.get_parameter('waypoint1_y').value,
             self.get_parameter('waypoint1_theta').value),
            (self.get_parameter('waypoint2_x').value,
             self.get_parameter('waypoint2_y').value,
             self.get_parameter('waypoint2_theta').value),
            (self.get_parameter('waypoint3_x').value,
             self.get_parameter('waypoint3_y').value,
             self.get_parameter('waypoint3_theta').value),
            (self.get_parameter('goal_x').value,
             self.get_parameter('goal_y').value,
             self.get_parameter('goal_theta').value),
        ]

        # Índice del waypoint activo
        self.indice_actual = 0

        # Protección contra pulsos repetidos de /next_point
        self.ultimo_estado = False

        # ── Publicadores y suscriptores ───────────────────────
        self.pub_set_point = self.create_publisher(Point, '/set_point', 10)
        self.create_subscription(Bool, '/next_point', self.cb_next_point, 10)

        # Publica el waypoint activo a 10 Hz
        self.create_timer(0.1, self.publicar_waypoint)

        self.get_logger().info(
            f'TrajectoryGenerator listo | {len(self.waypoints)} waypoints')
        tx, ty, tth = self.waypoints[0]
        self.get_logger().info(
            f'Waypoint 1/{len(self.waypoints)}: ({tx:.2f}, {ty:.2f}, {tth:.2f})')

    def cb_next_point(self, msg: Bool) -> None:
        """Avanza al siguiente waypoint en flanco ascendente."""
        if msg.data and not self.ultimo_estado:
            if self.indice_actual < len(self.waypoints) - 1:
                self.indice_actual += 1
                tx, ty, tth = self.waypoints[self.indice_actual]
                self.get_logger().info(
                    f'[TrajGen] Waypoint {self.indice_actual + 1}/'
                    f'{len(self.waypoints)}: ({tx:.2f}, {ty:.2f}, {tth:.2f})')
            else:
                self.get_logger().info('[TrajGen] ¡Todos los waypoints completados!')
        self.ultimo_estado = msg.data

    def publicar_waypoint(self) -> None:
        """Publica el waypoint activo — theta va en el campo z del Point."""
        if self.indice_actual >= len(self.waypoints):
            return
        tx, ty, tth = self.waypoints[self.indice_actual]
        msg = Point()
        msg.x = float(tx)
        msg.y = float(ty)
        msg.z = float(tth)   # theta codificado en z
        self.pub_set_point.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = TrajectoryGenerator()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        nodo.destroy_node()


if __name__ == '__main__':
    main()