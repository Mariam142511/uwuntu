#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import numpy as np

# --- FÓRMULA CORREGIDA (Evita los backflips) ---
def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

class PuzzlebotKinematics(Node):
    def __init__(self):
        super().__init__('puzzlebot_kinematics')
        
        self.r = 0.048    # Radio de la rueda (5cm)
        self.L = 0.19    # Distancia entre ruedas (19cm)
        
        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Posición acumulada de las llantas (Para quitar los errores de RVIZ)
        self.wl_pos = 0.0
        self.wr_pos = 0.0
        
        self.v = 0.0
        self.w = 0.0
        
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.02, self.update_logic)
        self.last_time = self.get_clock().now()
        
        self.get_logger().info("Nodo de Cinematica Puzzlebot INICIADO (Sin Backflips)")

    def cmd_vel_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_logic(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. Odometría
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta += self.w * dt

        q = quaternion_from_euler(0.0, 0.0, self.theta)

        # 2. Transformada TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 3. Odometría (Mensaje)
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.pub_odom.publish(odom)

        # 4. Juntas de las ruedas (CORREGIDO PARA RVIZ)
        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ['wheel_left_joint', 'wheel_right_joint']
        
        wr = (self.v + (self.w * self.L / 2.0)) / self.r
        wl = (self.v - (self.w * self.L / 2.0)) / self.r
        
        # Acumulamos la posición (esto quita los errores rojos)
        self.wr_pos += wr * dt
        self.wl_pos += wl * dt
        
        js.position = [self.wl_pos, self.wr_pos]
        js.velocity = [wl, wr]
        
        self.pub_joints.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
