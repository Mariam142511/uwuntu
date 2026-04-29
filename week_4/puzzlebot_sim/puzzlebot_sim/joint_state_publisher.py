import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Float32 
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import transforms3d
import numpy as np

class PuzzlebotPublisher(Node):
    def __init__(self):
        super().__init__('puzzlebot_publisher')
        
        self.namespace = self.get_namespace().rstrip('/')

        # Variables para la animación visual de las ruedas
        self.angle_r = 0.0
        self.angle_l = 0.0
        self.dt = 0.05

        # --- SUSCRIPCIONES ---
        # Recibe los comandos de velocidad (v, w) del controlador
        self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.wr = 0.0
        self.wl = 0.0

        # --- PUBLICACIONES ---
        # Publica las velocidades de ruedas para que el nodo de Localización las procese
        self.pub_wr = self.create_publisher(Float32, 'wr', 10)
        self.pub_wl = self.create_publisher(Float32, 'wl', 10)
        # Publica el estado de los joints para mover las ruedas en RViz
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Declare the parameter with a default value
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_z', 1.0)
        self.declare_parameter('init_pose_yaw', np.pi/2)
        self.declare_parameter('odom_frame', 'odom')

        # Retrieve the parameter value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value.strip('/')

        #Drone Initial Pose
        self.intial_pos_x = self.get_parameter('init_pose_x').value
        self.intial_pos_y = self.get_parameter('init_pose_y').value
        self.intial_pos_z = self.get_parameter('init_pose_z').value
        self.intial_pos_yaw = self.get_parameter('init_pose_yaw').value

        # Broadcasters Estáticos
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Broadcasters Dinámicos
        self.odom_broadcaster = TransformBroadcaster(self)

        # 2. Configuración TF: base_footprint -> base_link (Estático)
        # Eleva el cuerpo del robot 5cm sobre el nivel del suelo
        t_base_link = TransformStamped()
        t_base_link.header.stamp = self.get_clock().now().to_msg()
        t_base_link.header.frame_id = f"{self.namespace}/base_footprint"
        t_base_link.child_frame_id = f"{self.namespace}/base_link"
        t_base_link.transform.translation.x = 0.0
        t_base_link.transform.translation.y = 0.0
        t_base_link.transform.translation.z = 0.05
        q = transforms3d.euler.euler2quat(0, 0, 0)
        t_base_link.transform.rotation.x = q[1]
        t_base_link.transform.rotation.y = q[2]
        t_base_link.transform.rotation.z = q[3]
        t_base_link.transform.rotation.w = q[0]

        # 3. Configuración TF: base_link -> caster (Estático)
        # Posiciona la rueda loca en la parte trasera del robot
        t_caster = TransformStamped()
        t_caster.header.stamp = self.get_clock().now().to_msg()
        t_caster.header.frame_id = f"{self.namespace}/base_link"
        t_caster.child_frame_id = f"{self.namespace}/caster"
        t_caster.transform.translation.x = -0.09
        t_caster.transform.translation.y = 0.0
        t_caster.transform.translation.z = -0.043
        q = transforms3d.euler.euler2quat(0, 0, 0)
        t_caster.transform.rotation.x = q[1]
        t_caster.transform.rotation.y = q[2]
        t_caster.transform.rotation.z = q[3]
        t_caster.transform.rotation.w = q[0]

        # Enviamos las transformaciones estáticas una sola vez
        self.static_broadcaster.sendTransform([t_base_link])
        self.static_broadcaster.sendTransform([t_caster])
        
        # Broadcasters para los componentes móviles
        self.tf_odom = TransformBroadcaster(self)
        self.tf_base_footprint = TransformBroadcaster(self)

        # Configuración del Timer: Ejecuta la función timer_cb 20 veces por segundo
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def cmd_cb(self, msg):
        # CINEMÁTICA INVERSA
        # Convierte velocidad lineal/angular en velocidad de cada rueda (wr, wl)
        v, w = msg.linear.x, msg.angular.z
        L, R = 0.19, 0.05
        self.wr = (2 * v + w * L) / (2 * R)
        self.wl = (2 * v - w * L) / (2 * R)

        # Publica hacia el nodo de localización
        self.pub_wr.publish(Float32(data=float(self.wr)))
        self.pub_wl.publish(Float32(data=float(self.wl)))
    
    #Timer Callback
    def timer_cb(self):

        # LÓGICA DEL ÁRBOL DE TF GLOBAL
        # Para evitar parpadeos, solo un robot (group1) publica la relación map -> odom
        if "group1" in self.namespace:
        # DINÁMICO: map -> odom ---
            self.odom_transform = TransformStamped()
            self.odom_transform.header.stamp = self.get_clock().now().to_msg()
            self.odom_transform.header.frame_id = 'map'
            self.odom_transform.child_frame_id = 'odom'
            self.odom_transform.transform.translation.x = self.intial_pos_x
            self.odom_transform.transform.translation.y = self.intial_pos_y
            self.odom_transform.transform.translation.z = 0.0
            q_static = transforms3d.euler.euler2quat(0, 0, self.intial_pos_yaw)
            self.odom_transform.transform.rotation.x = q_static[1]
            self.odom_transform.transform.rotation.y = q_static[2]
            self.odom_transform.transform.rotation.z = q_static[3]
            self.odom_transform.transform.rotation.w = q_static[0]

            self.tf_broadcaster.sendTransform(self.odom_transform)

        # ANIMACIÓN DE RUEDAS
        # Acumula el ángulo para que se vean girar en el modelo 3D
        self.angle_r += self.wr * self.dt
        self.angle_l += self.wl * self.dt

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['wheel_r_joint', 'wheel_l_joint']
        joint_msg.position = [float(self.angle_r), float(self.angle_l)]
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotPublisher()
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