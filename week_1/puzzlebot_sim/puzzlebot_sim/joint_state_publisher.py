import math

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
import transforms3d
import numpy as np


class PuzzlebotPublisher(Node):

    def __init__(self):
        super().__init__('puzzlebot_publisher')

        # Parámetros de movimiento
        self.radius = 0.3
        self.omega = 0.5

        # Definir TF y Markers
        self.define_TF()
        self.define_markers()

        # Broadcaster por cada TF
        self.tf_br_footprint    = TransformBroadcaster(self)
        self.tf_br_base_link    = StaticTransformBroadcaster(self)
        self.tf_br_wheel_r      = TransformBroadcaster(self)
        self.tf_br_wheel_l      = TransformBroadcaster(self)
        self.tf_br_caster       = StaticTransformBroadcaster(self)

        # Enviar transforms estáticos una sola vez
        self.tf_br_base_link.sendTransform(self.base_link_tf)
        self.tf_br_caster.sendTransform(self.caster_tf)

        # Publisher por cada marker
        self.chassis_marker_pub  = self.create_publisher(Marker, '/chassis_marker', 10)
        self.wheel_r_marker_pub  = self.create_publisher(Marker, '/wheel_r_marker', 10)
        self.wheel_l_marker_pub  = self.create_publisher(Marker, '/wheel_l_marker', 10)
        self.caster_marker_pub   = self.create_publisher(Marker, '/caster_marker', 10)

        # Timer
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def timer_cb(self):
        time = self.get_clock().now().nanoseconds / 1e9

        # Actualizar stamps de markers
        self.chassis.header.stamp  = self.get_clock().now().to_msg()
        self.wheel_r.header.stamp  = self.get_clock().now().to_msg()
        self.wheel_l.header.stamp  = self.get_clock().now().to_msg()
        self.caster.header.stamp   = self.get_clock().now().to_msg()

        # odom -> base_footprint  (movimiento circular)
        x   = self.radius * math.cos(self.omega * time)
        y   = self.radius * math.sin(self.omega * time)
        yaw = self.omega * time

        self.odom_tf.header.stamp = self.get_clock().now().to_msg()
        self.odom_tf.transform.translation.x = x
        self.odom_tf.transform.translation.y = y
        self.odom_tf.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0.0, 0.0, yaw)
        self.odom_tf.transform.rotation.x = q[1]
        self.odom_tf.transform.rotation.y = q[2]
        self.odom_tf.transform.rotation.z = q[3]
        self.odom_tf.transform.rotation.w = q[0]

        # ruedas giran sobre Y
        wheel_angle = self.omega * time * 3.0
        q_w = transforms3d.euler.euler2quat(0.0, wheel_angle, 0.0)

        self.wheel_r_tf.header.stamp = self.get_clock().now().to_msg()
        self.wheel_r_tf.transform.rotation.x = q_w[1]
        self.wheel_r_tf.transform.rotation.y = q_w[2]
        self.wheel_r_tf.transform.rotation.z = q_w[3]
        self.wheel_r_tf.transform.rotation.w = q_w[0]

        self.wheel_l_tf.header.stamp = self.get_clock().now().to_msg()
        self.wheel_l_tf.transform.rotation.x = q_w[1]
        self.wheel_l_tf.transform.rotation.y = q_w[2]
        self.wheel_l_tf.transform.rotation.z = q_w[3]
        self.wheel_l_tf.transform.rotation.w = q_w[0]

        # Enviar TF dinámicos
        self.tf_br_footprint.sendTransform(self.odom_tf)
        self.tf_br_wheel_r.sendTransform(self.wheel_r_tf)
        self.tf_br_wheel_l.sendTransform(self.wheel_l_tf)

        # Publicar markers
        self.chassis_marker_pub.publish(self.chassis)
        self.wheel_r_marker_pub.publish(self.wheel_r)
        self.wheel_l_marker_pub.publish(self.wheel_l)
        self.caster_marker_pub.publish(self.caster)

    def define_markers(self):

        # Chassis — sin rotación en el marker porque el TF base_link ya está rotado pi/2
        self.chassis = Marker()
        self.chassis.header.frame_id = 'base_link'
        self.chassis.header.stamp = self.get_clock().now().to_msg()
        self.chassis.id = 0
        self.chassis.type = Marker.MESH_RESOURCE
        self.chassis.mesh_resource = 'package://puzzlebot_sim/meshes/Puzzlebot_Jetson_Lidar_Edition_Base.stl'
        self.chassis.action = Marker.ADD
        self.chassis.pose.position.x = 0.0
        self.chassis.pose.position.y = 0.0
        self.chassis.pose.position.z = 0.0
        self.chassis.pose.orientation.x = 0.0
        self.chassis.pose.orientation.y = 0.0
        self.chassis.pose.orientation.z = 0.0
        self.chassis.pose.orientation.w = 1.0
        self.chassis.scale.x = 1.0
        self.chassis.scale.y = 1.0
        self.chassis.scale.z = 1.0
        self.chassis.color.r = 0.8
        self.chassis.color.g = 0.7
        self.chassis.color.b = 0.0
        self.chassis.color.a = 1.0

        # Rueda derecha — sin rotación en marker
        self.wheel_r = Marker()
        self.wheel_r.header.frame_id = 'wheel_r'
        self.wheel_r.header.stamp = self.get_clock().now().to_msg()
        self.wheel_r.id = 0
        self.wheel_r.type = Marker.MESH_RESOURCE
        self.wheel_r.mesh_resource = 'package://puzzlebot_sim/meshes/Puzzlebot_Wheel.stl'
        self.wheel_r.action = Marker.ADD
        self.wheel_r.pose.position.x = 0.0
        self.wheel_r.pose.position.y = 0.0
        self.wheel_r.pose.position.z = 0.0
        q_wr = transforms3d.euler.euler2quat(math.pi / 2, 0.0, 0.0)
        self.wheel_r.pose.orientation.x = q_wr[1]
        self.wheel_r.pose.orientation.y = q_wr[2]
        self.wheel_r.pose.orientation.z = q_wr[3]
        self.wheel_r.pose.orientation.w = q_wr[0]
        self.wheel_r.scale.x = 1.0
        self.wheel_r.scale.y = 1.0
        self.wheel_r.scale.z = 1.0
        self.wheel_r.color.r = 1.0
        self.wheel_r.color.g = 0.2
        self.wheel_r.color.b = 0.2
        self.wheel_r.color.a = 1.0

        # Rueda izquierda — sin rotación en marker
        self.wheel_l = Marker()
        self.wheel_l.header.frame_id = 'wheel_l'
        self.wheel_l.header.stamp = self.get_clock().now().to_msg()
        self.wheel_l.id = 0
        self.wheel_l.type = Marker.MESH_RESOURCE
        self.wheel_l.mesh_resource = 'package://puzzlebot_sim/meshes/Puzzlebot_Wheel.stl'
        self.wheel_l.action = Marker.ADD
        self.wheel_l.pose.position.x = 0.0
        self.wheel_l.pose.position.y = 0.0
        self.wheel_l.pose.position.z = 0.0
        q_wl = transforms3d.euler.euler2quat(math.pi / 2, 0.0, 0.0)
        self.wheel_l.pose.orientation.x = q_wl[1]
        self.wheel_l.pose.orientation.y = q_wl[2]
        self.wheel_l.pose.orientation.z = q_wl[3]
        self.wheel_l.pose.orientation.w = q_wl[0]
        self.wheel_l.scale.x = 1.0
        self.wheel_l.scale.y = 1.0
        self.wheel_l.scale.z = 1.0
        self.wheel_l.color.r = 0.2
        self.wheel_l.color.g = 0.2
        self.wheel_l.color.b = 1.0
        self.wheel_l.color.a = 1.0

        # Caster
        self.caster = Marker()
        self.caster.header.frame_id = 'caster'
        self.caster.header.stamp = self.get_clock().now().to_msg()
        self.caster.id = 0
        self.caster.type = Marker.MESH_RESOURCE
        self.caster.mesh_resource = 'package://puzzlebot_sim/meshes/Puzzlebot_Caster_Wheel.stl'
        self.caster.action = Marker.ADD
        self.caster.pose.position.x = 0.0
        self.caster.pose.position.y = 0.0
        self.caster.pose.position.z = 0.0
        self.caster.pose.orientation.x = 0.0
        self.caster.pose.orientation.y = 0.0
        self.caster.pose.orientation.z = 0.0
        self.caster.pose.orientation.w = 1.0
        self.caster.scale.x = 1.0
        self.caster.scale.y = 1.0
        self.caster.scale.z = 1.0
        self.caster.color.r = 0.5
        self.caster.color.g = 0.5
        self.caster.color.b = 0.5
        self.caster.color.a = 1.0

    def define_TF(self):

        # odom -> base_footprint
        self.odom_tf = TransformStamped()
        self.odom_tf.header.stamp = self.get_clock().now().to_msg()
        self.odom_tf.header.frame_id = 'odom'
        self.odom_tf.child_frame_id = 'base_footprint'
        self.odom_tf.transform.translation.x = self.radius
        self.odom_tf.transform.translation.y = 0.0
        self.odom_tf.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0.0, 0.0, 0.0)
        self.odom_tf.transform.rotation.x = q[1]
        self.odom_tf.transform.rotation.y = q[2]
        self.odom_tf.transform.rotation.z = q[3]
        self.odom_tf.transform.rotation.w = q[0]

        # base_footprint -> base_link  (rotado pi/2 en Z para alinear el STL)
        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.header.frame_id = 'base_footprint'
        self.base_link_tf.child_frame_id = 'base_link'
        self.base_link_tf.transform.translation.x = 0.0
        self.base_link_tf.transform.translation.y = 0.0
        self.base_link_tf.transform.translation.z = 0.05
        q = transforms3d.euler.euler2quat(0.0, 0.0, math.pi / 2)
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]

        # base_link -> wheel_r  (valores del PDF)
        self.wheel_r_tf = TransformStamped()
        self.wheel_r_tf.header.stamp = self.get_clock().now().to_msg()
        self.wheel_r_tf.header.frame_id = 'base_link'
        self.wheel_r_tf.child_frame_id = 'wheel_r'
        self.wheel_r_tf.transform.translation.x = 0.052
        self.wheel_r_tf.transform.translation.y = -0.095
        self.wheel_r_tf.transform.translation.z = -0.0025
        q = transforms3d.euler.euler2quat(0.0, 0.0, 0.0)
        self.wheel_r_tf.transform.rotation.x = q[1]
        self.wheel_r_tf.transform.rotation.y = q[2]
        self.wheel_r_tf.transform.rotation.z = q[3]
        self.wheel_r_tf.transform.rotation.w = q[0]

        # base_link -> wheel_l  (valores del PDF)
        self.wheel_l_tf = TransformStamped()
        self.wheel_l_tf.header.stamp = self.get_clock().now().to_msg()
        self.wheel_l_tf.header.frame_id = 'base_link'
        self.wheel_l_tf.child_frame_id = 'wheel_l'
        self.wheel_l_tf.transform.translation.x = 0.052
        self.wheel_l_tf.transform.translation.y = 0.095
        self.wheel_l_tf.transform.translation.z = -0.0025
        q = transforms3d.euler.euler2quat(0.0, 0.0, 0.0)
        self.wheel_l_tf.transform.rotation.x = q[1]
        self.wheel_l_tf.transform.rotation.y = q[2]
        self.wheel_l_tf.transform.rotation.z = q[3]
        self.wheel_l_tf.transform.rotation.w = q[0]

        # base_link -> caster  (valores del PDF)
        self.caster_tf = TransformStamped()
        self.caster_tf.header.stamp = self.get_clock().now().to_msg()
        self.caster_tf.header.frame_id = 'base_link'
        self.caster_tf.child_frame_id = 'caster'
        self.caster_tf.transform.translation.x = -0.095
        self.caster_tf.transform.translation.y = 0.0
        self.caster_tf.transform.translation.z = -0.03
        q = transforms3d.euler.euler2quat(0.0, 0.0, 0.0)
        self.caster_tf.transform.rotation.x = q[1]
        self.caster_tf.transform.rotation.y = q[2]
        self.caster_tf.transform.rotation.z = q[3]
        self.caster_tf.transform.rotation.w = q[0]


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