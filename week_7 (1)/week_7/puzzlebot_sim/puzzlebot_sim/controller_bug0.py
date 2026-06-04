import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ControllerBug0(Node):
    """
    Controlador de navegación Bug0 para el Puzzlebot.

    Máquina de estados:
      • GIRAR_HACIA_META : rota en el sitio hasta apuntar al waypoint
      • AVANZAR_A_META   : avanza recto hacia el waypoint
      • WALL_FOLLOWING   : bordea la pared (mano derecha) ante obstáculo

    Regla de escape Bug0 (la más simple de la familia Bug):
      Sale del wall following en cuanto el frente está libre Y está más
      cerca de la meta que cuando chocó. No usa m-line — apenas el frente
      se despeja, vuelve a apuntar a la meta. Más reactivo que Bug2.

    Suscripciones:
      /set_point (Point, z=theta), /odom (Odometry), /scan (LaserScan)
    Publicaciones:
      /cmd_vel (Twist), /next_point (Bool)
    """

    def __init__(self):
        super().__init__('controllerBug0_node')

        # ── Parámetros configurables ──────────────────────────
        self.declare_parameter('linear_speed',               0.15)
        self.declare_parameter('angular_speed',              0.45)
        self.declare_parameter('publish_rate',               20.0)
        self.declare_parameter('goal_tolerance',             0.20)
        self.declare_parameter('yaw_tolerance',              0.25)
        self.declare_parameter('obstacle_distance',          0.40)
        self.declare_parameter('wall_dist_target',           0.35)
        self.declare_parameter('goal_reached_confirm_count', 10)
        self.declare_parameter('goal_reached_min_time_sec',  1.0)
        self.declare_parameter('stuck_timeout_sec',          12.0)

        self.vel_lineal      = float(self.get_parameter('linear_speed').value)
        self.vel_angular     = float(self.get_parameter('angular_speed').value)
        self.frecuencia      = float(self.get_parameter('publish_rate').value)
        self.tol_distancia   = float(self.get_parameter('goal_tolerance').value)
        self.tol_angulo      = float(self.get_parameter('yaw_tolerance').value)
        self.dist_obstaculo  = float(self.get_parameter('obstacle_distance').value)
        self.dist_pared      = float(self.get_parameter('wall_dist_target').value)
        self.confirm_meta    = int(self.get_parameter('goal_reached_confirm_count').value)
        self.tiempo_min_meta = float(self.get_parameter('goal_reached_min_time_sec').value)
        self.stuck_timeout   = float(self.get_parameter('stuck_timeout_sec').value)

        # Velocidades de wall following
        self.vel_wall_lineal  = 0.08
        self.vel_wall_angular = 0.50

        # ── Estado del robot ──────────────────────────────────
        self.x = self.y = self.th = 0.0
        self.meta_x = self.meta_y = self.meta_theta = 0.0
        self.hay_odom = False
        self.hay_meta = False
        self.meta_alcanzada = False

        # ── LiDAR ─────────────────────────────────────────────
        self.scan_front = float('inf')
        self.scan_right = float('inf')
        self.scan_fr_diag = float('inf')
        self.scan_ready = False

        # ── Máquina de estados ────────────────────────────────
        self.state         = 'GIRAR_HACIA_META'
        self.hit_dist      = float('inf')
        self.contador_meta = 0
        self.t_ultimo_cambio = 0.0

        # ── Anti-atasco ───────────────────────────────────────
        self.mejor_dist        = float('inf')
        self.t_ultimo_progreso = 0.0

        # ── ROS ───────────────────────────────────────────────
        self.pub      = self.create_publisher(Twist, '/cmd_vel',    10)
        self.pub_next = self.create_publisher(Bool,  '/next_point', 10)
        self.create_subscription(Odometry,  '/odom',      self.odom_cb,       10)
        self.create_subscription(LaserScan, '/scan',      self.scan_callback, 10)
        self.create_subscription(Point,     '/set_point', self.cb_set_point,  10)
        self.create_timer(1.0 / self.frecuencia, self.control_loop)
        self.get_logger().info('ControllerBug0 listo')

    # ─────────────────────────────────────────────────────────
    # Utilidades
    # ─────────────────────────────────────────────────────────

    @staticmethod
    def _norm(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _sector_min(ranges, center_idx, half_width, n):
        idxs = [(center_idx + i) % n for i in range(-half_width, half_width + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.01) & np.isfinite(vals)]
        return float(np.min(vals)) if len(vals) > 0 else float('inf')

    # ─────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────

    def scan_callback(self, msg):
        ranges    = np.array(msg.ranges, dtype=float)
        n         = len(ranges)
        inc       = msg.angle_increment
        half_fwd  = max(1, int(math.radians(40.0 / 2) / inc))
        half_sid  = max(1, int(math.radians(30.0 / 2) / inc))
        idx_right = int(round(math.radians(-90) / inc)) % n
        idx_fr    = int(round(math.radians(-45) / inc)) % n
        self.scan_front    = self._sector_min(ranges, 0,         half_fwd, n)
        self.scan_right    = self._sector_min(ranges, idx_right, half_sid, n)
        self.scan_fr_diag  = self._sector_min(ranges, idx_fr,    half_sid, n)
        self.scan_ready = True

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.th = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.hay_odom = True

    def cb_set_point(self, msg):
        nx, ny, nth = float(msg.x), float(msg.y), float(msg.z)
        if math.hypot(nx - self.meta_x, ny - self.meta_y) < 0.01 and self.hay_meta:
            return
        self.meta_x, self.meta_y, self.meta_theta = nx, ny, nth
        self.hay_meta       = True
        self.meta_alcanzada = False
        self.contador_meta  = 0
        self.state          = 'GIRAR_HACIA_META'
        self.hit_dist       = float('inf')
        self.mejor_dist     = float('inf')
        ahora = self.get_clock().now().nanoseconds / 1e9
        self.t_ultimo_cambio   = ahora
        self.t_ultimo_progreso = ahora
        self.get_logger().info(f'[Bug0] Waypoint: ({nx:.2f}, {ny:.2f})')

    # ─────────────────────────────────────────────────────────
    # Wall following (mano derecha)
    # ─────────────────────────────────────────────────────────

    def _seguir_pared_derecha(self):
        cmd = Twist()
        frente_libre = self.scan_front > self.dist_obstaculo

        if not frente_libre:
            cmd.linear.x  = 0.0
            cmd.angular.z = self.vel_wall_angular
            return cmd

        cmd.linear.x = self.vel_wall_lineal
        if not math.isfinite(self.scan_right) or self.scan_right > self.dist_pared * 2.5:
            # Pared derecha perdida (apertura/puerta) — avanzar recto un poco
            # y girar suave a la derecha para reencontrarla sin perderse
            cmd.linear.x  = self.vel_wall_lineal
            cmd.angular.z = -self.vel_wall_angular * 0.35
        elif self.scan_right > (self.dist_pared + 0.06):
            cmd.angular.z = -self.vel_wall_angular * 0.5   # acercarse
        elif self.scan_right < (self.dist_pared - 0.06):
            cmd.angular.z =  self.vel_wall_angular * 0.5   # alejarse
        else:
            cmd.angular.z = 0.0
        return cmd

    # ─────────────────────────────────────────────────────────
    # Ciclo principal
    # ─────────────────────────────────────────────────────────

    def control_loop(self):
        cmd = Twist()

        if not self.hay_odom or not self.hay_meta or not self.scan_ready:
            self.pub.publish(cmd)
            return
        if self.meta_alcanzada:
            self.pub.publish(cmd)
            return

        dx    = self.meta_x - self.x
        dy    = self.meta_y - self.y
        rho   = math.hypot(dx, dy)
        alpha = self._norm(math.atan2(dy, dx) - self.th)
        ahora = self.get_clock().now().nanoseconds / 1e9

        # Anti-atasco
        if rho < self.mejor_dist - 0.05:
            self.mejor_dist = rho
            self.t_ultimo_progreso = ahora

        # ¿Meta alcanzada?
        error_theta = self._norm(self.meta_theta - self.th)
        cerca = rho < self.tol_distancia and abs(error_theta) < self.tol_angulo
        if cerca and (ahora - self.t_ultimo_cambio) >= self.tiempo_min_meta:
            self.contador_meta = min(self.confirm_meta, self.contador_meta + 1)
        else:
            self.contador_meta = 0
        if self.contador_meta >= self.confirm_meta:
            self.get_logger().info(f'[Bug0] ¡Meta alcanzada! ({self.meta_x:.2f},{self.meta_y:.2f})')
            self.meta_alcanzada = True
            b = Bool(); b.data = True
            self.pub_next.publish(b)
            self.pub.publish(Twist())
            return

        # ── GIRAR_HACIA_META ──────────────────────────────────
        if self.state == 'GIRAR_HACIA_META':
            if self.scan_front < self.dist_obstaculo:
                self.hit_dist = rho
                self.t_ultimo_progreso = ahora
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[Bug0] Obstáculo → WALL_FOLLOWING (hit={rho:.2f}m)')
                cmd = self._seguir_pared_derecha()
            else:
                if abs(alpha) > 0.15:
                    cmd.angular.z = np.sign(alpha) * self.vel_angular
                    self.get_logger().info(
                        f'[Bug0] Girando | alpha={math.degrees(alpha):.0f}° rho={rho:.2f}m',
                        throttle_duration_sec=1.0)
                else:
                    self.state = 'AVANZAR_A_META'

        # ── AVANZAR_A_META ────────────────────────────────────
        elif self.state == 'AVANZAR_A_META':
            if self.scan_front < self.dist_obstaculo:
                self.hit_dist = rho
                self.t_ultimo_progreso = ahora
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(f'[Bug0] Obstáculo → WALL_FOLLOWING (hit={rho:.2f}m)')
                cmd = self._seguir_pared_derecha()
            else:
                if abs(alpha) > 0.30:
                    self.state = 'GIRAR_HACIA_META'
                else:
                    cmd.linear.x = self.vel_lineal
                    self.get_logger().info(
                        f'[Bug0] Avanzando | rho={rho:.2f}m',
                        throttle_duration_sec=1.0)

        # ── WALL_FOLLOWING ────────────────────────────────────
        elif self.state == 'WALL_FOLLOWING':
            frente_libre = self.scan_front > self.dist_obstaculo

            # Regla de escape Bug0: frente libre, claramente más cerca que
            # el hit point, Y el rumbo hacia la meta está despejado
            rumbo_a_meta_libre = (frente_libre and abs(alpha) < 0.6)
            if frente_libre and rho < (self.hit_dist - 0.20) and rumbo_a_meta_libre:
                self.get_logger().info(
                    f'[Bug0] Escape (rho={rho:.2f} < hit={self.hit_dist:.2f}, '
                    f'alpha={math.degrees(alpha):.0f}°) → GIRAR_HACIA_META')
                self.state = 'GIRAR_HACIA_META'
                self.pub.publish(Twist())
                return

            # Anti-atasco
            if (ahora - self.t_ultimo_progreso) > self.stuck_timeout:
                self.get_logger().warn('[Bug0] Atascado — maniobra de despegue')
                cmd.linear.x  = -0.05
                cmd.angular.z =  self.vel_angular
                self.pub.publish(cmd)
                self.t_ultimo_progreso = ahora
                return

            cmd = self._seguir_pared_derecha()
            self.get_logger().info(
                f'[Bug0] WALL_FOLLOW | front={self.scan_front:.2f} right={self.scan_right:.2f}',
                throttle_duration_sec=1.0)

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBug0()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()