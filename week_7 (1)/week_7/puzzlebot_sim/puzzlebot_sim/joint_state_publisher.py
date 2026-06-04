import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import StaticTransformBroadcaster


class PuzzlebotPublisher(Node):
    """
    Nodo auxiliar del Puzzlebot con tres responsabilidades:

    1. Cinemática inversa: convierte (v, w) de /cmd_vel en velocidades
       de rueda (wr, wl) y las publica para el EKF.
    2. Animación de ruedas: acumula el ángulo girado y publica
       /joint_states para que RViz mueva las ruedas.
    3. Transformadas estáticas internas del chasis:
         base_footprint → base_link
         base_link      → caster
       (El TF dinámico odom → base_footprint lo publica localisation.py)
    """

    # Parámetros físicos del Puzzlebot Jetson Lidar Edition
    WHEEL_BASE   = 0.19   # distancia entre ruedas [m]
    WHEEL_RADIUS = 0.05   # radio de la rueda [m]

    def __init__(self):
        super().__init__('puzzlebot_publisher')

        # ── Estado de las ruedas ──────────────────────────────
        self.wr      = 0.0   # velocidad angular rueda derecha [rad/s]
        self.wl      = 0.0   # velocidad angular rueda izquierda [rad/s]
        self.angle_r = 0.0   # ángulo acumulado rueda derecha [rad]
        self.angle_l = 0.0   # ángulo acumulado rueda izquierda [rad]
        self.dt      = 0.05  # período del timer [s] → 20 Hz

        # ── Publicadores ──────────────────────────────────────
        self.pub_wr    = self.create_publisher(Float32,    'wr',           10)
        self.pub_wl    = self.create_publisher(Float32,    'wl',           10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # ── Suscriptores ──────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel, 10)

        # ── Transformadas estáticas del chasis ────────────────
        # Solo se envían una vez al arrancar
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publicar_transforms_estaticas()

        # ── Timer principal ───────────────────────────────────
        self.create_timer(self.dt, self.cb_timer)
        self.get_logger().info('PuzzlebotPublisher listo | 20 Hz')

    # ─────────────────────────────────────────────────────────
    # Transformadas estáticas internas del robot
    # ─────────────────────────────────────────────────────────

    def _publicar_transforms_estaticas(self):
        """Publica una sola vez las TF fijas del chasis."""
        ahora = self.get_clock().now().to_msg()

        # base_footprint → base_link (eleva el cuerpo 5 cm)
        t1 = TransformStamped()
        t1.header.stamp    = ahora
        t1.header.frame_id = 'base_footprint'
        t1.child_frame_id  = 'base_link'
        t1.transform.translation.z = 0.05
        t1.transform.rotation.w    = 1.0

        # base_link → caster (rueda loca trasera)
        t2 = TransformStamped()
        t2.header.stamp    = ahora
        t2.header.frame_id = 'base_link'
        t2.child_frame_id  = 'caster'
        t2.transform.translation.x = -0.09
        t2.transform.translation.z = -0.043
        t2.transform.rotation.w    = 1.0

        self.static_broadcaster.sendTransform([t1, t2])

    # ─────────────────────────────────────────────────────────
    # Cinemática inversa
    # ─────────────────────────────────────────────────────────

    def cb_cmd_vel(self, msg: Twist):
        """
        Convierte (v, w) → (wr, wl) usando la cinemática diferencial:
          wr = (2v + wL) / (2R)
          wl = (2v - wL) / (2R)
        """
        v = msg.linear.x
        w = msg.angular.z
        L = self.WHEEL_BASE
        R = self.WHEEL_RADIUS

        self.wr = (2.0 * v + w * L) / (2.0 * R)
        self.wl = (2.0 * v - w * L) / (2.0 * R)

        # Publicar velocidades de rueda para el EKF
        self.pub_wr.publish(Float32(data=float(self.wr)))
        self.pub_wl.publish(Float32(data=float(self.wl)))

    # ─────────────────────────────────────────────────────────
    # Timer: animación de ruedas
    # ─────────────────────────────────────────────────────────

    def cb_timer(self):
        """Acumula el ángulo girado y publica joint_states para RViz."""
        self.angle_r += self.wr * self.dt
        self.angle_l += self.wl * self.dt

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = ['wheel_r_joint', 'wheel_l_joint']
        msg.position     = [float(self.angle_r), float(self.angle_l)]
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()