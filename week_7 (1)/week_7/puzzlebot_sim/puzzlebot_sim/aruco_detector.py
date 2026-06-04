import math
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray


class ArucoDetector(Node):
    """
    Nodo de detección de marcadores ArUco desde la cámara del Puzzlebot.

    Recibe imágenes de la cámara de Gazebo, detecta marcadores ArUco,
    estima su pose 3D con estimatePoseSingleMarkers y publica los
    resultados como MarkerArray para el EKF de localización.

    Suscripciones:
      - /camera      (Image)      : imagen RGB de la cámara
      - /camera_info (CameraInfo) : parámetros intrínsecos (K, D)

    Publicaciones:
      - /aruco_markers         (MarkerArray) : poses de markers detectados
      - /aruco/image_annotated (Image)       : imagen con markers dibujados
    """

    def __init__(self) -> None:
        super().__init__('aruco_detector_node')

        # ── Parámetros del nodo ───────────────────────────────
        self.declare_parameter('image_topic',           '/camera')
        self.declare_parameter('camera_info_topic',     '/camera_info')
        self.declare_parameter('aruco_topic',           '/aruco_markers')
        self.declare_parameter('annotated_image_topic', '/aruco/image_annotated')
        self.declare_parameter('marker_size',            0.18)
        self.declare_parameter('dictionary',             'DICT_4X4_250')

        self.topic_imagen  = str(self.get_parameter('image_topic').value)
        self.topic_caminfo = str(self.get_parameter('camera_info_topic').value)
        self.topic_markers = str(self.get_parameter('aruco_topic').value)
        self.topic_anotada = str(self.get_parameter('annotated_image_topic').value)
        self.tam_marker    = float(self.get_parameter('marker_size').value)

        # ── Estado interno ────────────────────────────────────
        self.bridge       = CvBridge()
        self.K            = None   # Matriz intrínseca 3x3
        self.D            = None   # Coeficientes de distorsión
        self.camara_lista = False
        self.mutex        = threading.Lock()

        # ── Preparar detectores ArUco ─────────────────────────
        # Se prueban múltiples diccionarios en orden de preferencia
        nombres_dict = [
            str(self.get_parameter('dictionary').value),
            'DICT_4X4_250',
            'DICT_4X4_100',
            'DICT_4X4_50',
        ]

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.params = cv2.aruco.DetectorParameters()
        else:
            self.params = cv2.aruco.DetectorParameters_create()

        self.diccionarios = []
        for nombre in nombres_dict:
            dict_id = getattr(cv2.aruco, nombre, None)
            if dict_id is None:
                continue
            if hasattr(cv2.aruco, 'getPredefinedDictionary'):
                d = cv2.aruco.getPredefinedDictionary(dict_id)
            else:
                d = cv2.aruco.Dictionary_get(dict_id)
            self.diccionarios.append((nombre, d))

        # ── Publicadores ──────────────────────────────────────
        self.pub_markers = self.create_publisher(MarkerArray, self.topic_markers, 10)
        self.pub_imagen  = self.create_publisher(Image, self.topic_anotada,  10)

        # ── Suscriptores ──────────────────────────────────────
        self.create_subscription(CameraInfo, self.topic_caminfo, self.cb_caminfo, 10)
        self.create_subscription(Image,      self.topic_imagen,  self.cb_imagen,  5)

        self.get_logger().info(
            f'ArucoDetector listo | imagen={self.topic_imagen} '
            f'| publica={self.topic_markers} | marker_size={self.tam_marker}m')

    # ─────────────────────────────────────────────────────────
    # Callback: parámetros intrínsecos de la cámara
    # ─────────────────────────────────────────────────────────

    def cb_caminfo(self, msg: CameraInfo) -> None:
        """Guarda K y D al recibir el primer mensaje de camera_info."""
        with self.mutex:
            if self.camara_lista:
                return
            self.K = np.array(msg.k, dtype=float).reshape((3, 3))
            self.D = np.array(msg.d, dtype=float) if len(msg.d) > 0 \
                     else np.zeros(5, dtype=float)
            self.camara_lista = True
            self.get_logger().info(
                f'CameraInfo recibida | fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f}')

    # ─────────────────────────────────────────────────────────
    # Callback: imagen de la cámara
    # ─────────────────────────────────────────────────────────

    def cb_imagen(self, msg: Image) -> None:
        """
        Procesa cada frame:
          1. Detecta esquinas de markers ArUco
          2. Estima pose 3D con estimatePoseSingleMarkers
          3. Convierte rvec → cuaternión
          4. Publica MarkerArray e imagen anotada
        """
        with self.mutex:
            if not self.camara_lista:
                return
            K = self.K.copy()
            D = self.D.copy()

        # Convertir imagen ROS → OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge error: {e}')
            return

        gris    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        anotada = frame.copy()
        array   = MarkerArray()

        # Detectar markers probando diccionarios en orden
        esquinas, ids = None, None
        for _, diccionario in self.diccionarios:
            esq, ids_test, _ = cv2.aruco.detectMarkers(
                gris, diccionario, parameters=self.params)
            if ids_test is not None and len(ids_test) > 0:
                esquinas = esq
                ids      = ids_test
                break

        # Sin detecciones → publicar array vacío
        if ids is None or len(ids) == 0:
            self.pub_markers.publish(array)
            try:
                self.pub_imagen.publish(
                    self.bridge.cv2_to_imgmsg(anotada, encoding='bgr8'))
            except Exception:
                pass
            return

        # Estimar pose 3D de cada marker detectado
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            esquinas, self.tam_marker, K, D)

        sello = self.get_clock().now().to_msg()

        for i, mid in enumerate(ids.flatten()):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            # Convertir rvec → matriz R → cuaternión
            R, _ = cv2.Rodrigues(rvec)
            qw = math.sqrt(max(0.0, 1.0 + R[0,0] + R[1,1] + R[2,2])) / 2.0
            qx = (R[2,1] - R[1,2]) / (4.0 * qw + 1e-12)
            qy = (R[0,2] - R[2,0]) / (4.0 * qw + 1e-12)
            qz = (R[1,0] - R[0,1]) / (4.0 * qw + 1e-12)

            distancia = float(np.linalg.norm(tvec))

            # Conversión de ejes OpenCV → ROS
            # OpenCV: X→derecha, Y→abajo,  Z→frente
            # ROS:    X→frente,  Y→izquierda, Z→arriba
            m = Marker()
            m.header.stamp    = sello
            m.header.frame_id = msg.header.frame_id if msg.header.frame_id \
                                 else 'camera_link_optical'
            m.ns     = 'aruco'
            m.id     = int(mid)
            m.type   = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x =  float(tvec[2])   # Z_cv  → X_ros
            m.pose.position.y = -float(tvec[0])   # -X_cv → Y_ros
            m.pose.position.z = -float(tvec[1])   # -Y_cv → Z_ros
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw

            # La distancia se codifica en scale.x para que el EKF
            # pueda escalar la incertidumbre de medición dinámicamente
            m.scale.x = distancia
            m.scale.y = distancia
            m.scale.z = distancia
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 0.8
            m.color.b = 0.2

            array.markers.append(m)

            self.get_logger().info(
                f'id={int(mid)} | dist={distancia:.2f}m | '
                f'tvec=({tvec[0]:.2f},{tvec[1]:.2f},{tvec[2]:.2f})')

            # ── Anotar imagen ─────────────────────────────────
            pts = esquinas[i].astype(int).reshape((-1, 2))
            cv2.polylines(anotada, [pts], True, (0, 255, 255), 3, cv2.LINE_AA)
            cx = int(np.mean(pts[:, 0]))
            cy = int(np.mean(pts[:, 1]))
            cv2.circle(anotada, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(anotada, f'ID {int(mid)}',
                        (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(anotada, f'd={distancia:.2f}m',
                        (cx + 8, cy + 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2, cv2.LINE_AA)

        # Dibujar contornos y ejes 3D
        try:
            cv2.aruco.drawDetectedMarkers(anotada, esquinas, ids)
            for i in range(len(ids)):
                cv2.drawFrameAxes(anotada, K, D,
                                  rvecs[i], tvecs[i],
                                  self.tam_marker * 0.5)
        except Exception:
            pass

        # Publicar resultados
        self.pub_markers.publish(array)
        try:
            self.pub_imagen.publish(
                self.bridge.cv2_to_imgmsg(anotada, encoding='bgr8'))
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetector()
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