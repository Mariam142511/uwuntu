import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
import transforms3d
import numpy as np
from std_msgs.msg import Float32 

class KinematicModel(Node):
    def __init__(self):
        super().__init__('kinematic_model')
        
        # PARTE 1 variables del estado ideal
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0
        self.wheel_angle = 0.0

        # parametros fisicos
        self.r = 0.05     # Radio de la llanta 
        self.L = 0.19     # Distancia entre llantas 

        # Publicadores y Suscriptores
        self.wr_pub = self.create_publisher(Float32, '/wr', 10)
        self.wl_pub = self.create_publisher(Float32, '/wl', 10)
        # Recibe comandos de velocidad (v, w)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        # Publica la pose calculada
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)

        # Broadcasters Estáticos
        #self.map_broadcaster = StaticTransformBroadcaster(self)
        self.base_link_broadcaster = StaticTransformBroadcaster(self)
        self.caster_broadcaster = StaticTransformBroadcaster(self)

        # Broadcasters Dinámicos
        self.odom_broadcaster = TransformBroadcaster(self)

        # Configuración TF: world -> map (Estático)
        #t_map = TransformStamped()
        #t_map.header.stamp = self.get_clock().now().to_msg()
        #t_map.child_frame_id = 'map'
        #t_map.transform.translation.y = 0.0
        #t_map.transform.translation.z = 0.0
        #q = transforms3d.euler.euler2quat(0, 0, 0)
        #t_map.transform.rotation.x = q[1]
        #t_map.transform.rotation.y = q[2]
        #t_map.transform.rotation.z = q[3]
        #t_map.transform.rotation.w = q[0]

        # 2. Configuración TF: base_footprint -> base_link (Estático)
        # Eleva el cuerpo del robot 5cm sobre el nivel del suelo
        t_base_link = TransformStamped()
        t_base_link.header.stamp = self.get_clock().now().to_msg()
        t_base_link.header.frame_id = 'base_footprint'
        t_base_link.child_frame_id = 'base_link'
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
        t_caster.header.frame_id = 'base_link'
        t_caster.child_frame_id = 'caster'
        t_caster.transform.translation.x = -0.09
        t_caster.transform.translation.y = 0.0
        t_caster.transform.translation.z = -0.043
        q = transforms3d.euler.euler2quat(0, 0, 0)
        t_caster.transform.rotation.x = q[1]
        t_caster.transform.rotation.y = q[2]
        t_caster.transform.rotation.z = q[3]
        t_caster.transform.rotation.w = q[0]

        # Enviamos las transformaciones estáticas una sola vez
        #self.map_broadcaster.sendTransform([t_map])
        self.base_link_broadcaster.sendTransform([t_base_link])
        self.caster_broadcaster.sendTransform([t_caster])

        # Inicialización de mensajes para las transformaciones dinámicas (que cambian en el timer)
        self.odom_transform = TransformStamped()
        self.base_footprint_transform = TransformStamped()
        self.wheel_r_transform = TransformStamped()
        self.wheel_l_transform = TransformStamped()
        
        # Broadcasters para los componentes móviles
        self.tf_odom = TransformBroadcaster(self)
        self.tf_base_footprint = TransformBroadcaster(self)
        self.tf_wheel_r = TransformBroadcaster(self)
        self.tf_wheel_l = TransformBroadcaster(self)

        # Configuración del Timer: Ejecuta la función timer_cb 20 veces por segundo
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def cmd_cb(self, msg):
        # Actualiza las velocidades deseadas desde el tópico /cmd_vel
        self.v = msg.linear.x
        self.w = msg.angular.z

    #Timer Callback
    def timer_cb(self):
        dt = 0.05
        
        # PARTE 1 logica de cinematica ideal (simulador) 
        self.x += self.v * np.cos(self.th) * dt
        self.y += self.v * np.sin(self.th) * dt
        self.th += self.w * dt
        self.wheel_angle += (self.v / 0.05) * self.dt

        # Cálculo de las velocidades de las ruedas (Cinemática Inversa)
        # v = r*(wr + wl)/2  y  w = r*(wr - wl)/L
        wr_val = (2.0 * self.v + self.w * self.L) / (2.0 * self.r) 
        wl_val = (2.0 * self.v - self.w * self.L) / (2.0 * self.r) 
        self.get_logger().info(f'Velocidades Ruedas -> R: {wr_val:.2f}, L: {wl_val:.2f}')

        # Crear los mensajes
        msg_wr = Float32()
        msg_wl = Float32()
        msg_wr.data = float(wr_val)
        msg_wl.data = float(wl_val)

        # Publicar
        self.wr_pub.publish(msg_wr)
        self.wl_pub.publish(msg_wl)
        
        # DINÁMICO: map -> odom ---
        # Simula el movimiento del robot siguiendo una trayectoria circular
        self.odom_transform.header.stamp = self.get_clock().now().to_msg()
        self.odom_transform.header.frame_id = 'map'
        self.odom_transform.child_frame_id = 'odom'
        self.odom_transform.transform.translation.x = 0.5
        self.odom_transform.transform.translation.y = 0.0
        self.odom_transform.transform.translation.z = 0.0
        
        q_static = transforms3d.euler.euler2quat(0, 0, 0)
        self.odom_transform.transform.rotation.x = q_static[1]
        self.odom_transform.transform.rotation.y = q_static[2]
        self.odom_transform.transform.rotation.z = q_static[3]
        self.odom_transform.transform.rotation.w = q_static[0]

        # DINÁMICO: odom -> base_footprint
        self.base_footprint_transform.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_transform.header.frame_id = 'odom'
        self.base_footprint_transform.child_frame_id = 'base_footprint'
        self.base_footprint_transform.transform.translation.x = self.x
        self.base_footprint_transform.transform.translation.y = self.y
        self.base_footprint_transform.transform.translation.z = 0.0
        self.timer
        q_yaw = transforms3d.euler.euler2quat(0, 0, self.th)
        self.base_footprint_transform.transform.rotation.x = q_yaw[1]
        self.base_footprint_transform.transform.rotation.y = q_yaw[2]
        self.base_footprint_transform.transform.rotation.z = q_yaw[3]
        self.base_footprint_transform.transform.rotation.w = q_yaw[0]

        #Rueda derecha
        self.wheel_r_transform.header.stamp = self.get_clock().now().to_msg()
        self.wheel_r_transform.header.frame_id = 'base_link'
        self.wheel_r_transform.child_frame_id = 'wheel_r'
        self.wheel_r_transform.transform.translation.x = 0.052
        self.wheel_r_transform.transform.translation.y = -0.095
        self.wheel_r_transform.transform.translation.z = -0.0025
        q_wheel = transforms3d.euler.euler2quat(0, self.wheel_angle, 0)
        self.wheel_r_transform.transform.rotation.x = q_wheel[1]
        self.wheel_r_transform.transform.rotation.y = q_wheel[2]
        self.wheel_r_transform.transform.rotation.z = q_wheel[3]
        self.wheel_r_transform.transform.rotation.w = q_wheel[0]

        #Rueda izquierda
        self.wheel_l_transform.header.stamp = self.get_clock().now().to_msg()
        self.wheel_l_transform.header.frame_id = 'base_link'
        self.wheel_l_transform.child_frame_id = 'wheel_l'
        self.wheel_l_transform.transform.translation.x = 0.052
        self.wheel_l_transform.transform.translation.y = 0.095
        self.wheel_l_transform.transform.translation.z = -0.0025
        q_wheel = transforms3d.euler.euler2quat(0, self.wheel_angle, 0)
        self.wheel_l_transform.transform.rotation.x = q_wheel[1]
        self.wheel_l_transform.transform.rotation.y = q_wheel[2]
        self.wheel_l_transform.transform.rotation.z = q_wheel[3]
        self.wheel_l_transform.transform.rotation.w = q_wheel[0]

        # Publicar todas las transformaciones
        self.tf_odom.sendTransform(self.odom_transform)
        self.tf_base_footprint.sendTransform(self.base_footprint_transform)
        self.tf_wheel_r.sendTransform(self.wheel_r_transform)
        self.tf_wheel_l.sendTransform(self.wheel_l_transform)

        # Publicacion de posestamped 
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation = self.base_footprint_transform.transform.rotation
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicModel()
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