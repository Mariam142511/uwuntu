import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ControllerBug2(Node):

    def __init__(self):
        super().__init__('controller_bug2')

        # Declaramos los parámetros para que ROS2 los acepte desde el Launch
        self.declare_parameter('goal_x', 1.45)
        self.declare_parameter('goal_y', 1.20)

        # Leemos los valores que envió el Launch (si no envia nada, toma el valor por defecto)
        g_x = self.get_parameter('goal_x').value
        g_y = self.get_parameter('goal_y').value
        
        # Guardamos la meta dinámicamente
        self.waypoints = [(g_x, g_y)]
        self.current_idx = 0
        self.rho_threshold = 0.15  # Umbral de llegada a la meta (15 cm)

        # Ganancias del controlador de movimiento hacia la meta
        self.kv = 0.20
        self.ka = 0.65

        # Velocidades constantes puras para seguir la pared (Estrategia Máquina de Estados)
        self.wall_linear_speed = 0.12
        self.wall_angular_speed = 0.45
        self.wall_dist_target = 0.35  # Distancia deseada al muro por la derecha (35 cm)

        # Máquina de estados: "GO_TO_GOAL" o "WALL_FOLLOWING"
        self.state = 'GO_TO_GOAL'
        self.obstacle_threshold = 0.40  # Distancia para activar evasión (40 cm)
        
        # Almacenamiento del LiDAR obtenido del nodo de referencia
        self.scan_front = float('inf')
        self.scan_right = float('inf')
        self.scan_ready = False

        # Memoria geométrica de la Línea M
        self.start_x = 0.0  # Punto inicial (A)
        self.start_y = 0.0
        self.dist_at_collision = float('inf')  # Hit Point
        self.first_run = True

        # Pose del robot actualizada por la odometría de tu nodo 'localisation'
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # SUSCRIPCIONES Y PUBLICACIONES ROS
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    @staticmethod
    def _sector_min(ranges, center_idx, half_width, n):
        """Calcula el valor mínimo de un sector del LiDAR ignorando infinitos y ceros."""
        idxs = [(center_idx + i) % n for i in range(-half_width, half_width + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.01) & np.isfinite(vals)]
        return float(np.min(vals)) if len(vals) > 0 else float('inf')

    def scan_callback(self, msg: LaserScan):
        """Filtra y obtiene de forma precisa las distancias del Frente y de la Derecha (-90°)."""
        ranges = np.array(msg.ranges, dtype=float)
        n = len(ranges)
        inc = msg.angle_increment

        # Calculamos los tamaños de los sectores basados en los grados (FOV) del nodo de referencia
        half_fwd = max(1, int(math.radians(40.0 / 2) / inc))   # Frente (Cono de 40 grados)
        half_side = max(1, int(math.radians(30.0 / 2) / inc))  # Derecha (Cono de 30 grados)
        idx_right = int(round(math.radians(-90) / inc)) % n     # Índice exacto de los -90 grados

        # Extraemos las lecturas reales puras
        self.scan_front = self._sector_min(ranges, 0, half_fwd, n)
        self.scan_right = self._sector_min(ranges, idx_right, half_side, n)
        self.scan_ready = True

    def odom_cb(self, msg):
        """Obtiene la pose del robot y fija el origen de la línea M en el primer ciclo."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Cuaternión a Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.th = np.arctan2(siny_cosp, cosy_cosp)

        # Captura el punto inicial exacto del robot para definir la Recta M
        if self.first_run:
            self.start_x = self.x
            self.start_y = self.y
            self.first_run = False
            self.get_logger().info(f'M-Line inicializada desde ({self.start_x:.2f}, {self.start_y:.2f})')

    def control_loop(self):
        """Máquina de estados aplicando las reglas estrictas de Bug2."""
        if not self.scan_ready or self.first_run:
            return

        if self.current_idx >= len(self.waypoints):
            msg_stop = Twist()
            self.pub.publish(msg_stop)

            goal_reached_msg = Bool()
            goal_reached_msg.data = True
            self.goal_reached_pub.publish(goal_reached_msg)
            return

        # Coordenadas del objetivo actual
        target_x, target_y = self.waypoints[self.current_idx]

        # Errores en Coordenadas Polares hacia la meta
        dx = target_x - self.x
        dy = target_y - self.y
        rho = np.sqrt(dx**2 + dy**2)

        angle_to_goal = np.arctan2(dy, dx)
        alpha = angle_to_goal - self.th
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        # CÁLCULO GEOMÉTRICO DE LA LÍNEA M
        # Distancia perpendicular del robot a la recta que pasa por (start) y (target)
        num = abs((target_y - self.start_y) * self.x - (target_x - self.start_x) * self.y + target_x * self.start_y - target_y * self.start_x)
        den = np.sqrt((target_y - self.start_y)**2 + (target_x - self.start_x)**2)
        distance_to_m_line = num / den if den > 0 else 0.0

        msg_vel = Twist()

        # MAQUINA DE ESTADOS - LÓGICA BUG2
        if self.state == 'GO_TO_GOAL':
            # Si hay obstáculo en la línea M, pasamos a seguir la pared
            if self.scan_front < self.obstacle_threshold:
                self.get_logger().info(f'[Bug 2] ¡Obstáculo a {self.scan_front:.2f}m! Guardando Hit Point a {rho:.2f}m de la meta.')
                self.dist_at_collision = rho  # Registramos la distancia del impacto
                self.state = 'WALL_FOLLOWING'
            else:
                # Controlador hacia el objetivo
                if rho > self.rho_threshold:
                    if abs(alpha) > 0.20:  # Giro sobre su propio eje si está muy desalineado
                        msg_vel.linear.x = 0.0
                        msg_vel.angular.z = self.ka * alpha
                    else:  # Avanza y corrige rumbo simultáneamente
                        msg_vel.linear.x = self.kv * rho if rho > 0.3 else 0.08
                        msg_vel.angular.z = self.ka * alpha
                else:
                    self.get_logger().info('¡Meta final alcanzada con éxito!')
                    self.current_idx += 1

        elif self.state == 'WALL_FOLLOWING':
            # REGLAS DE ESCAPE DE BUG2 COMPLETAS:
            on_m_line = distance_to_m_line < 0.12  # Tolerancia estrecha de la M-line
            closer_to_goal = rho < (self.dist_at_collision - 0.10)  # Más cerca que cuando chocamos (Leave Point)
            frente_libre = self.scan_front > self.obstacle_threshold  # Frente despejado

            # Si se cumplen todas las condiciones geométricas, dejamos la pared
            if on_m_line and closer_to_goal and frente_libre:
                self.get_logger().info(f'[Bug 2] M-Line cruzada en punto despejado ({rho:.2f}m < {self.dist_at_collision:.2f}m). Volviendo a GO_TO_GOAL')
                self.state = 'GO_TO_GOAL'
            else:
                # COMPORTAMIENTO MÁQUINA DE ESTADOS PARA COMPORTASE REACTIVO ANTE EL MURO
                if not frente_libre:
                    # Bloqueo total al frente: detiene avance lineal y gira a la izquierda sobre su propio eje
                    msg_vel.linear.x = 0.0
                    msg_vel.angular.z = self.wall_angular_speed
                else:
                    # El frente está libre; usamos lógica de histéresis/máquina pura para costear el muro derecho
                    msg_vel.linear.x = self.wall_linear_speed
                    
                    if self.scan_right > (self.wall_dist_target + 0.06):
                        # Se está alejando mucho de la pared derecha -> Cerrarse (Girar a la derecha)
                        msg_vel.angular.z = -self.wall_angular_speed * 0.5
                    elif self.scan_right < (self.wall_dist_target - 0.06):
                        # Se está pegando demasiado a la pared derecha -> Abrirse (Girar a la izquierda)
                        msg_vel.angular.z = self.wall_angular_speed * 0.5
                    else:
                        # Distancia ideal -> Avanzar en línea recta con suavidad
                        msg_vel.angular.z = 0.0

        self.pub.publish(msg_vel)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBug2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Forzar detención del robot al apagar el nodo
        stop_msg = Twist()
        node.pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()