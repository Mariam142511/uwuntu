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
        
        # Parámetros físicos
        self.r = 0.05
        self.L = 0.19
        
        # Estado (Dead Reckoning)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # Suscripciones a las ruedas 
        self.sub_wr = self.create_subscription(Float32, '/wr', self.wr_cb, 10)
        self.sub_wl = self.create_subscription(Float32, '/wl', self.wl_cb, 10)
        
        # Publicador de Odometría
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
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
        
        # Cinemática Diferencial (Dead Reckoning)
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L
        
        # Integración de Euler
        self.x += v * np.cos(self.th) * self.dt
        self.y += v * np.sin(self.th) * self.dt
        self.th += w * self.dt

        # Crear y publicar mensaje de Odometría
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Posición
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        
        # Orientación (Conversión de Euler Z a Cuaternión simple)
        odom.pose.pose.orientation.z = np.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.th / 2.0)
        
        self.odom_pub.publish(odom)

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