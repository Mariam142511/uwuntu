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

        # PARAMETROS DE POSICION INICIAL
        # Se leen para saber donde empieza el robot en el mapa global
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.init_x = self.get_parameter('init_pose_x').value
        self.init_y = self.get_parameter('init_pose_y').value

        # Constantes fisicas del Puzzlebot
        self.r = 0.05 # Radio de rueda
        self.L = 0.19 # Distancia entre ruedas
        
        # ESTADO Y PARÁMETROS DE INCERTIDUMBRE
        self.x = 0.0   # Posición estimada en X
        self.y = 0.0   # Posición estimada en Y
        self.th = 0.0  # Orientación estimada (Theta)
        
        # Constantes de error de los motores (Parámetros a tunear)
        self.kr = 0.02 
        self.kl = 0.02
        # Matriz de Covarianza de la Pose (Incertidumbre acumulada)
        self.P = np.zeros((3, 3))

        if "group1" in self.namespace:
            # El Líder es el "Ground Truth" (Referencia ideal)
            # Al ser 0, la matriz P nunca aumentará (incertidumbre cero).
            self.kr = 0.0 
            self.kl = 0.0
            self.P = np.zeros((3, 3))

        # --- SUSCRIPCIONES ---
        # Escucha las velocidades angulares de cada rueda publicadas por el simulador
        self.sub_wr = self.create_subscription(Float32, 'wr', self.wr_cb, 10)
        self.sub_wl = self.create_subscription(Float32, 'wl', self.wl_cb, 10)
        
        # --- PUBLICACIONES ---
        # Publica el mensaje estandar de Odometria (posicion y velocidad estimada)
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
        
        # CINEMATICA DIFERENCIAL (Dead Reckoning)
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L
        th_ant = self.th

        # Integracion para obtener posicion (X, Y) y orientacion (Theta)
        self.x += v * np.cos(self.th) * self.dt
        self.y += v * np.sin(self.th) * self.dt
        self.th += w * self.dt

        # Jacobiano con respecto al estado anterior (Fp)
        Fp = np.array([
            [1, 0, -v * np.sin(th_ant) * self.dt],
            [0, 1,  v * np.cos(th_ant) * self.dt],
            [0, 0, 1]
        ])

        # Jacobiano con respecto al ruido de los motores (F_delta)
        # Este mapea como el error en wr y wl afecta a x, y, th
        alpha = self.dt * self.r / 2.0
        beta = self.dt * self.r / self.L
        
        F_delta = np.array([
            [alpha * np.cos(th_ant), alpha * np.cos(th_ant)],
            [alpha * np.sin(th_ant), alpha * np.sin(th_ant)],
            [beta, -beta]
        ])

        # Matriz de covarianza del ruido de los motores (Q)
        # ΣΔ = [kr*|wr|  0; 0  kl*|wl|]
        Q = np.array([
            [self.kr * abs(self.wr), 0],
            [0, self.kl * abs(self.wl)]
        ])

        # Prediccion de la nueva Covarianza: P = Fp*P*Fp.T + F_delta*Q*F_delta.T
        self.P = Fp @ self.P @ Fp.T + F_delta @ Q @ F_delta.T

        # Construccion del mensaje de Odometria
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = f"{self.namespace}/odom"
        odom.child_frame_id = f"{self.namespace}/base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = np.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.th / 2.0)

        # LLENAR COVARIANZA EN EL MENSAJE (Matriz 6x6 aplanada a 36 elementos)
        # ROS usa 6x6 (x, y, z, roll, pitch, yaw). Solo llenamos x, y y yaw (indice 5)
        cov = np.zeros(36)
        cov[0] = self.P[0, 0] # Var(x)
        cov[1] = self.P[0, 1] # Cov(x,y)
        cov[5] = self.P[0, 2] # Cov(x,theta)
        cov[6] = self.P[1, 0] # Cov(y,x)
        cov[7] = self.P[1, 1] # Var(y)
        cov[11] = self.P[1, 2] # Cov(y,theta)
        cov[30] = self.P[2, 0] # Cov(theta,x)
        cov[31] = self.P[2, 1] # Cov(theta,y)
        cov[35] = self.P[2, 2] # Var(theta)
        
        odom.pose.covariance = cov.tolist()
        self.odom_pub.publish(odom)

        # PUBLICACION DE LA TRANSFORMADA (TF)
        # Conecta el frame 'odom' con el robot, compensando la posicion inicial
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = f"{self.namespace}/odom"
        t.child_frame_id = f"{self.namespace}/base_footprint"
        t.transform.translation.x = self.x 
        t.transform.translation.y = self.y
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