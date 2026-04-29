import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from std_msgs.msg import Bool

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.namespace = self.get_namespace().rstrip('/')

        # --- SUSCRIPCIONES ---
        # El nodo se suscribe a su propia odometría para saber su posición actual (X, Y, Theta)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        
        self.target_x = 0.0
        self.target_y = 0.0

        # LÓGICA LÍDER-SEGUIDOR:
        # Si el robot pertenece al grupo 2 (seguidor), se suscribe a la odometría del grupo 1 (líder)
        if "group2" in self.namespace:
            self.sub_target = self.create_subscription(Odometry, '/group1/odom', self.target_cb, 10)

        # --- PUBLICACIONES ---
        # Publica comandos de velocidad (v, w) para mover al robot
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publica un booleano cuando el líder llega a su destino final
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        # configuracion de trayectoria
        # Cambia estos puntos para hacer un triángulo o cuadrado
        # Cuadrado:
        self.waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        # Triangulo:
        #self.waypoints = [(1.0, 0.0), (0.5, 0.86), (0.0, 0.0)]
        self.current_idx = 0

        # Ganancias Proporcionales del controlador
        self.kv = 0.3  # Velocidad lineal
        self.ka = 0.8  # Velocidad angular
        
        self.x = 0.0 
        self.y = 0.0 
        self.th = 0.0
        
        # Timer que ejecuta el bucle de control a 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def target_cb(self, msg):
        # Actualiza la posición del Robot 1 como el nuevo objetivo
        self.target_x = msg.pose.pose.position.x
        self.target_y = msg.pose.pose.position.y

    def odom_cb(self, msg):
        
        # Callback para actualizar la posición actual del robot
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Conversión de Cuaternión a ángulo Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.th = np.arctan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        '''
        Se implementó un controlador proporcional en coordenadas polares 
        con una máquina de estados que prioriza la alineación angular alpha
        antes del avance lineal rho garantizando estabilidad y evitando 
        oscilaciones bruscas durante el seguimiento dinámico
        '''
        # Definir el objetivo (Target)
        if "group2" in self.namespace:
            # el robot2: Su objetivo es el robot1
            target_x = self.target_x
            target_y = self.target_y
            rho_threshold = 0.05  # Qué tan cerca quieres que llegue (5cm)
        else:
            # el robot1: Su objetivo es la lista de puntos
            if self.current_idx >= len(self.waypoints):
                self.pub.publish(Twist()) # Stop
                return
            
            target_x, target_y = self.waypoints[self.current_idx]
            rho_threshold = 0.08

        # Cálculo de Errores (Coordenadas Polares)
        dx = target_x - self.x
        dy = target_y - self.y
        rho = np.sqrt(dx**2 + dy**2) # Distancia al objetivo
        
        angle_to_goal = np.arctan2(dy, dx)
        alpha = angle_to_goal - self.th # Error de orientación
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha)) # Normalización

        msg = Twist()

        # MÁQUINA DE ESTADOS DEL CONTROLADOR
        if rho > rho_threshold:
            if abs(alpha) > 0.15:
                # Girar hacia el objetivo si el ángulo es grande
                msg.linear.x = 0.0
                msg.angular.z = self.ka * alpha
            else:
                # Avanzar y corregir ángulo suavemente
                v_gain = 0.5 if "group2" in self.namespace else self.kv
                msg.linear.x = v_gain * rho
                msg.angular.z = self.ka * alpha
        else:
            # solo el robot 1 llega al punto pasa al sigueinte watpont
            if "group1" in self.namespace:
                self.get_logger().info(f'Punto {self.current_idx} alcanzado')
                self.current_idx += 1
            
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()