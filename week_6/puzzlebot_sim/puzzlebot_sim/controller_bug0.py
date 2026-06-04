import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ControllerBug0(Node):

    def __init__(self):
        super().__init__('controller_bug0')

        # Declaramos los parámetros para que ROS2 los acepte desde el Launch
        self.declare_parameter('goal_x', 1.45)
        self.declare_parameter('goal_y', 1.20)

        # Leemos los valores que envió el Launch
        g_x = self.get_parameter('goal_x').value
        g_y = self.get_parameter('goal_y').value

        # Guardamos la meta dinámicamente
        self.waypoints = [(g_x, g_y)]
        self.current_idx = 0
        self.rho_threshold = 0.15  # Tolerancia de llegada (15 cm)

        # VARIABLES DE LA MÁQUINA DE ESTADOS
        # Estados posibles: "GIRAR_HACIA_META", "AVANZAR_A_META", "WALL_FOLLOWING", "GOAL_REACHED"
        self.state = 'GIRAR_HACIA_META'

        self.obstacle_threshold = 0.40  # Umbral frontal (40 cm)
        self.wall_dist_target = 0.35    # Distancia deseada a la pared derecha (35 cm)
        
        self.scan_front = float('inf')
        self.scan_right = float('inf')
        self.scan_ready = False
        self.hit_dist = float('inf')    # Memoria de distancia para el escape de Bug 0

        # Velocidades FIJAS (Estrategia Máquina de Estados Pura)
        self.VEL_LINEAL_AVANCE = 0.15   # m/s
        self.VEL_ANGULAR_GIRO = 0.45    # rad/s
        self.VEL_WALL_LINEAL = 0.12     # m/s
        self.VEL_WALL_ANGULAR = 0.50    # rad/s

        # Pose del robot compartida (Localisation)
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

        # Configuración de conos usando FOV discretos
        half_fwd = max(1, int(math.radians(40.0 / 2) / inc))   # Cono frontal de 40°
        half_side = max(1, int(math.radians(30.0 / 2) / inc))  # Cono derecho de 30°
        idx_right = int(round(math.radians(-90) / inc)) % n     # Índice de los -90° exactos

        self.scan_front = self._sector_min(ranges, 0, half_fwd, n)
        self.scan_right = self._sector_min(ranges, idx_right, half_side, n)
        self.scan_ready = True

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Conversión matemática de Cuaternión a Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.th = np.arctan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if not self.scan_ready:
            return

        # Si ya alcanzamos la meta final, detenemos por completo
        if self.current_idx >= len(self.waypoints) or self.state == 'GOAL_REACHED':
            msg_stop = Twist()
            self.pub.publish(msg_stop)
            return

        # Obtener objetivo actual
        target_x, target_y = self.waypoints[self.current_idx]

        # Cálculos geométricos de error
        dx = target_x - self.x
        dy = target_y - self.y
        rho = np.sqrt(dx**2 + dy**2)

        angle_to_goal = np.arctan2(dy, dx)
        alpha = angle_to_goal - self.th
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Acotar entre [-pi, pi]

        # CONDICIÓN GLOBAL DE PARADA
        if rho < self.rho_threshold:
            self.get_logger().info('¡META ALCANZADA!')
            self.state = 'GOAL_REACHED'
            msg_stop = Twist()
            self.pub.publish(msg_stop)

            goal_reached_msg = Bool()
            goal_reached_msg.data = True
            self.goal_reached_pub.publish(goal_reached_msg)
            self.current_idx += 1
            return

        msg_vel = Twist()

        # MÁQUINA DE ESTADOS REACTIVA

        # ESTADO: Girar en el propio eje hasta apuntar a la meta
        if self.state == 'GIRAR_HACIA_META':
            if self.scan_front < self.obstacle_threshold:
                self.hit_dist = rho
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info('[BUG 0] Obstáculo detectado al intentar girar. Pasando a WALL_FOLLOWING.')
            else:
                if abs(alpha) > 0.15:
                    msg_vel.linear.x = 0.0
                    msg_vel.angular.z = np.sign(alpha) * self.VEL_ANGULAR_GIRO
                else:
                    self.state = 'AVANZAR_A_META'

        # ESTADO: Avanzar en línea recta hacia la meta
        elif self.state == 'AVANZAR_A_META':
            if self.scan_front < self.obstacle_threshold:
                self.hit_dist = rho
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[BUG 0] Obstáculo detectado en camino recto. Siguiendo pared (hit_dist={rho:.2f}m).')
            else:
                if abs(alpha) > 0.30:
                    self.state = 'GIRAR_HACIA_META'
                else:
                    msg_vel.linear.x = self.VEL_LINEAL_AVANCE
                    msg_vel.angular.z = 0.0

        # ESTADO: Rodear el obstáculo costeando la pared derecha
        elif self.state == 'WALL_FOLLOWING':
            frente_libre = self.scan_front > self.obstacle_threshold
            
            # Condición de escape de Bug 0: Frente despejado Y estar más cerca de la meta que cuando chocamos
            if frente_libre and rho < (self.hit_dist - 0.10):
                self.get_logger().info('[BUG 0] Escape válido (frente libre y más cerca). Volviendo a orientarse a la meta.')
                self.state = 'GIRAR_HACIA_META'
            else:
                # Comportamiento reactivo On/Off de seguimiento de muros (Idéntico a tu Bug 2 funcional)
                if not frente_libre:
                    # Muro al frente: detiene avance y gira a la izquierda sobre su propio eje
                    msg_vel.linear.x = 0.0
                    msg_vel.angular.z = self.VEL_WALL_ANGULAR
                else:
                    # Frente libre: avanza y mantiene distancia con histéresis discreta en el flanco derecho
                    msg_vel.linear.x = self.VEL_WALL_LINEAL
                    
                    if self.scan_right > (self.wall_dist_target + 0.06):
                        # Se aleja de la pared -> Se cierra a la derecha
                        msg_vel.angular.z = -self.VEL_WALL_ANGULAR * 0.5
                    elif self.scan_right < (self.wall_dist_target - 0.06):
                        # Se pega mucho a la pared -> Se abre a la izquierda
                        msg_vel.angular.z = self.VEL_WALL_ANGULAR * 0.5
                    else:
                        # En zona ideal -> Avanza recto
                        msg_vel.angular.z = 0.0

        # Publicar velocidades calculadas
        self.pub.publish(msg_vel)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBug0()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()