import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from std_msgs.msg import Bool

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        # Suscripción a la pose calculada en la Parte 2
        self.sub = self.create_subscription(PoseStamped, '/pose', self.pose_cb, 10)
        
        # Publicación de comandos de velocidad
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Bandera de llegada: Avisa a otros nodos cuando la trayectoria completa ha terminado
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        # configuracion de trayectoria
        # Cambia estos puntos para hacer un triángulo o cuadrado
        # Cuadrado:
        self.waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        # Triangulo:
        #self.waypoints = [(1.0, 0.0), (0.5, 0.86), (0.0, 0.0)]
        self.current_idx = 0
        
        # Ganancias del controlador (ajustan qué tan rápido gira y avanza)
        self.kv = 0.3  # Velocidad lineal
        self.ka = 0.8  # Velocidad angular
        
        # Estado actual del robot
        self.x = 0.0 
        self.y = 0.0 
        self.th = 0.0
        
        # Timer de control a 20Hz (mismo que la simulación)
        self.timer = self.create_timer(0.05, self.control_loop)

    def pose_cb(self, msg):
        #Recibe la posición y convierte el cuaternión a ángulo Yaw
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        
        # Conversión de Cuaternión a Euler (Yaw)
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.th = np.arctan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """
        TIPO DE CONTROLADOR: Se eligió un controlador Proporcional en Coordenadas Polares
        ESTRATEGIA: Control cinemático basado en coordenadas polares (error de distancia rho 
        y error de orientación alpha).
        COMPORTAMIENTO: Implementa una máquina de estados para priorizar la alineación angular 
        antes del avance lineal, garantizando una trayectoria más estable.
        """
        #Lógica de Go-to-Goal
        if self.current_idx >= len(self.waypoints):
            msg_reached = Bool()
            msg_reached.data = True
            self.goal_reached_pub.publish(msg_reached)
            self.pub.publish(Twist()) # Stop
            return

        target_x, target_y = self.waypoints[self.current_idx]
        
        # Calcular errores hacia el objetivo
        dx = target_x - self.x
        dy = target_y - self.y
        rho = np.sqrt(dx**2 + dy**2) # Distancia (error de posición)
        
        angle_to_goal = np.arctan2(dy, dx)
        alpha = angle_to_goal - self.th # Error de ángulo
        
        # Normalizar ángulo entre -pi y pi para que no gire de más
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        msg = Twist()

        # Máquina de estados 
        if rho > 0.08: # Tolerancia de llegada (8 cm)
            # Si el ángulo es muy grande, girar en el lugar primero
            if abs(alpha) > 0.15:
                msg.linear.x = 0.0
                msg.angular.z = self.ka * alpha
            else:
                # Si está alineado, avanzar y corregir ángulo suavemente
                msg.linear.x = self.kv * rho
                msg.angular.z = self.ka * alpha
        else:
            # Llegó al punto actual, pasar al siguiente
            self.get_logger().info(f'Punto {self.current_idx} alcanzado: ({target_x}, {target_y})')
            self.current_idx += 1
            
        self.pub.publish(msg)

        """
        La implementación de una máquina de estados garantiza la estabilidad, evitando que el robot 
        oscile al intentar alzanzar un punto exacto
        """

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