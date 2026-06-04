import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ControllerBug2(Node):
    """
    Controlador de navegación Bug2 para el Puzzlebot.

    Combina dos comportamientos en una máquina de estados:
      • GO_TO_GOAL     : control proporcional polar hacia el waypoint
      • WALL_FOLLOWING : sigue la pared (mano derecha) cuando hay obstáculo

    Reglas de salida de Bug2 (clásicas):
      Sale del wall following solo cuando el robot vuelve a cruzar la
      m-line (recta inicio→meta) en un punto MÁS CERCANO a la meta que
      el punto donde chocó (hit point). Esto evita bucles infinitos.

    Robustez extra:
      • Anti-atasco: si lleva mucho tiempo sin acercarse a la meta,
        fuerza un giro de despegue.
      • Filtrado de sectores LiDAR para front / right / left.
      • Giro hacia el lado correcto al detectar obstáculo de frente.

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
        self.declare_parameter('mline_tolerance',            0.15)
        self.declare_parameter('goal_reached_confirm_count', 10)
        self.declare_parameter('goal_reached_min_time_sec',  1.0)
        self.declare_parameter('stuck_timeout_sec',          12.0)

        # Ganancias del control hacia meta
        self.kv = 0.35   # ganancia lineal
        self.ka = 0.90   # ganancia angular

        self.vel_lineal      = float(self.get_parameter('linear_speed').value)
        self.vel_angular     = float(self.get_parameter('angular_speed').value)
        self.frecuencia      = float(self.get_parameter('publish_rate').value)
        self.tol_distancia   = float(self.get_parameter('goal_tolerance').value)
        self.tol_angulo      = float(self.get_parameter('yaw_tolerance').value)
        self.dist_obstaculo  = float(self.get_parameter('obstacle_distance').value)
        self.dist_pared      = float(self.get_parameter('wall_dist_target').value)
        self.tol_mline       = float(self.get_parameter('mline_tolerance').value)
        self.confirm_meta    = int(self.get_parameter('goal_reached_confirm_count').value)
        self.tiempo_min_meta = float(self.get_parameter('goal_reached_min_time_sec').value)
        self.stuck_timeout   = float(self.get_parameter('stuck_timeout_sec').value)

        # ── Estado del robot ──────────────────────────────────
        self.x = self.y = self.th = 0.0
        self.meta_x = self.meta_y = self.meta_theta = 0.0
        self.start_x = self.start_y = 0.0
        self.hay_odom = False
        self.hay_meta = False
        self.meta_alcanzada = False

        # ── LiDAR ─────────────────────────────────────────────
        self.scan_front = float('inf')
        self.scan_right = float('inf')
        self.scan_left  = float('inf')
        self.scan_ready = False

        # ── Máquina de estados ────────────────────────────────
        self.state             = 'GO_TO_GOAL'
        self.dist_at_collision = float('inf')
        self.contador_meta     = 0
        self.t_ultimo_cambio   = 0.0

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
        self.get_logger().info('ControllerBug2 listo')

    # ─────────────────────────────────────────────────────────
    # Utilidades
    # ─────────────────────────────────────────────────────────

    @staticmethod
    def _norm(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _sector_min(ranges, center_idx, half_width, n):
        """Mínima distancia en un sector circular del LiDAR."""
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
        half_sid  = max(1, int(math.radians(40.0 / 2) / inc))
        idx_right = int(round(math.radians(-90) / inc)) % n
        idx_left  = int(round(math.radians( 90) / inc)) % n
        self.scan_front = self._sector_min(ranges, 0,         half_fwd, n)
        self.scan_right = self._sector_min(ranges, idx_right, half_sid, n)
        self.scan_left  = self._sector_min(ranges, idx_left,  half_sid, n)
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
        if self.hay_odom:
            self.start_x, self.start_y = self.x, self.y
        self.state             = 'GO_TO_GOAL'
        self.dist_at_collision = float('inf')
        self.mejor_dist        = float('inf')
        ahora = self.get_clock().now().nanoseconds / 1e9
        self.t_ultimo_cambio   = ahora
        self.t_ultimo_progreso = ahora
        self.get_logger().info(f'[Bug2] Waypoint: ({nx:.2f}, {ny:.2f})')

    # ─────────────────────────────────────────────────────────
    # Distancia a la m-line
    # ─────────────────────────────────────────────────────────

    def _dist_mline(self):
        num = abs((self.meta_y - self.start_y) * self.x -
                  (self.meta_x - self.start_x) * self.y +
                  self.meta_x * self.start_y - self.meta_y * self.start_x)
        den = math.hypot(self.meta_y - self.start_y, self.meta_x - self.start_x)
        return num / den if den > 1e-6 else 0.0

    # ─────────────────────────────────────────────────────────
    # Wall following (mano derecha) con manejo de esquinas
    # ─────────────────────────────────────────────────────────

    def _seguir_pared_derecha(self):
        cmd = Twist()
        frente_libre = self.scan_front > self.dist_obstaculo

        if not frente_libre:
            # Obstáculo al frente — girar a la izquierda en el sitio
            cmd.linear.x  = 0.0
            cmd.angular.z = self.vel_angular
            return cmd

        # Frente libre — costear la pared derecha
        cmd.linear.x = self.vel_lineal * 0.8
        if not math.isfinite(self.scan_right):
            # Perdió la pared derecha (esquina convexa) — girar a la derecha
            # suavemente para reencontrarla en vez de seguir recto
            cmd.angular.z = -self.vel_angular * 0.6
        elif self.scan_right > (self.dist_pared + 0.06):
            cmd.angular.z = -self.vel_angular * 0.5   # acercarse a la pared
        elif self.scan_right < (self.dist_pared - 0.06):
            cmd.angular.z =  self.vel_angular * 0.5   # alejarse de la pared
        else:
            cmd.angular.z = 0.0                       # distancia ideal
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

        # Geometría hacia la meta
        dx    = self.meta_x - self.x
        dy    = self.meta_y - self.y
        rho   = math.hypot(dx, dy)
        alpha = self._norm(math.atan2(dy, dx) - self.th)
        dist_mline = self._dist_mline()
        ahora = self.get_clock().now().nanoseconds / 1e9

        # ── Actualizar progreso (anti-atasco) ─────────────────
        if rho < self.mejor_dist - 0.05:
            self.mejor_dist = rho
            self.t_ultimo_progreso = ahora

        # ── ¿Meta alcanzada? ──────────────────────────────────
        error_theta = self._norm(self.meta_theta - self.th)
        cerca = rho < self.tol_distancia and abs(error_theta) < self.tol_angulo
        if cerca and (ahora - self.t_ultimo_cambio) >= self.tiempo_min_meta:
            self.contador_meta = min(self.confirm_meta, self.contador_meta + 1)
        else:
            self.contador_meta = 0
        if self.contador_meta >= self.confirm_meta:
            self.get_logger().info(f'[Bug2] ¡Meta alcanzada! ({self.meta_x:.2f},{self.meta_y:.2f})')
            self.meta_alcanzada = True
            b = Bool(); b.data = True
            self.pub_next.publish(b)
            self.pub.publish(Twist())
            return

        # ── GO_TO_GOAL ────────────────────────────────────────
        if self.state == 'GO_TO_GOAL':
            if self.scan_front < self.dist_obstaculo:
                # Chocó — guardar hit point y empezar wall following
                self.dist_at_collision = rho
                self.t_ultimo_progreso = ahora
                self.state = 'WALL_FOLLOWING'
                self.get_logger().info(
                    f'[Bug2] Obstáculo → WALL_FOLLOWING (hit={rho:.2f}m)')
                cmd = self._seguir_pared_derecha()
            else:
                if abs(alpha) > 0.20:
                    # Desalineado — girar en el sitio
                    cmd.linear.x  = 0.0
                    cmd.angular.z = max(min(self.ka * alpha, self.vel_angular),
                                        -self.vel_angular)
                else:
                    # Avanzar corrigiendo rumbo
                    cmd.linear.x  = min(self.kv * rho, self.vel_lineal) if rho > 0.3 else 0.08
                    cmd.angular.z = max(min(self.ka * alpha, self.vel_angular),
                                        -self.vel_angular)
                    self.get_logger().info(
                        f'[Bug2] GO_TO_GOAL | rho={rho:.2f}m alpha={math.degrees(alpha):.0f}°',
                        throttle_duration_sec=1.0)

        # ── WALL_FOLLOWING ────────────────────────────────────
        elif self.state == 'WALL_FOLLOWING':
            on_mline     = dist_mline < self.tol_mline
            closer       = rho < (self.dist_at_collision - 0.10)
            frente_libre = self.scan_front > self.dist_obstaculo

            # Regla de salida Bug2
            if on_mline and closer and frente_libre:
                self.get_logger().info(
                    f'[Bug2] M-line cruzada → GO_TO_GOAL '
                    f'(rho={rho:.2f} < hit={self.dist_at_collision:.2f})')
                self.state = 'GO_TO_GOAL'
                self.pub.publish(Twist())
                return

            # Anti-atasco: si lleva mucho sin progresar, despegar
            if (ahora - self.t_ultimo_progreso) > self.stuck_timeout:
                self.get_logger().warn('[Bug2] Atascado — maniobra de despegue')
                cmd.linear.x  = -0.05
                cmd.angular.z =  self.vel_angular
                self.pub.publish(cmd)
                self.t_ultimo_progreso = ahora
                return

            cmd = self._seguir_pared_derecha()
            self.get_logger().info(
                f'[Bug2] WALL_FOLLOW | front={self.scan_front:.2f} '
                f'right={self.scan_right:.2f} mline={dist_mline:.2f}',
                throttle_duration_sec=1.0)

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBug2()
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