import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class Localisation(Node):
    def __init__(self):
        super().__init__('localisation')
        
        self.namespace = self.get_namespace().rstrip('/')

        # PARÁMETROS DE POSICIÓN INICIAL
        # Se leen para saber dónde empieza el robot en el mapa global
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.init_x = self.get_parameter('init_pose_x').value
        self.init_y = self.get_parameter('init_pose_y').value

        # Constantes físicas del Puzzlebot
        self.r = 0.05 # Radio de rueda
        self.L = 0.19 # Distancia entre ruedas
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # --- SUSCRIPCIONES ---
        # Escucha las velocidades angulares de cada rueda publicadas por el simulador
        self.sub_wr = self.create_subscription(Float32, 'wr', self.wr_cb, 10)
        self.sub_wl = self.create_subscription(Float32, 'wl', self.wl_cb, 10)
        
        # --- PUBLICACIONES ---
        # Publica el mensaje estándar de Odometría (posición y velocidad estimada)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # Broadcaster para publicar la transformada (TF) de odom -> base_footprint
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.wr = 0.0
        self.wl = 0.0
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def wr_cb(self, msg):
        self.wr = msg.data

    def wl_cb(self, msg):
        self.wl = msg.data

    def timer_cb(self):
        self.get_logger().info(f'Velocidades Ruedas -> R: {self.wr:.2f}, L: {self.wl:.2f}')
        
        # CINEMÁTICA DIFERENCIAL (Dead Reckoning)
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L
        
        # Integración para obtener posición (X, Y) y orientación (Theta)
        self.x += v * np.cos(self.th) * self.dt
        self.y += v * np.sin(self.th) * self.dt
        self.th += w * self.dt

        # Construcción del mensaje de Odometría
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = f"{self.namespace}/odom"
        odom.child_frame_id = f"{self.namespace}/base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = np.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.th / 2.0)
        self.odom_pub.publish(odom)

        # PUBLICACIÓN DE LA TRANSFORMADA (TF)
        # Conecta el frame 'odom' con el robot, compensando la posición inicial
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = f"{self.namespace}/base_footprint"
        t.transform.translation.x = self.x + self.init_x
        t.transform.translation.y = self.y + self.init_y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()