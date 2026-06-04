import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray


# ─────────────────────────────────────────────────────────────
# Utilidades de geometría 2D
# ─────────────────────────────────────────────────────────────

def normalizar_angulo(angulo: float) -> float:
    """Normaliza un ángulo al rango [-π, π]."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


def componer_poses(pose_a: np.ndarray, pose_b: np.ndarray) -> np.ndarray:
    """Compone dos poses 2D: T_ab = T_a ⊕ T_b."""
    theta_a = float(pose_a[2])
    c = math.cos(theta_a)
    s = math.sin(theta_a)
    x     = float(pose_a[0]) + c * float(pose_b[0]) - s * float(pose_b[1])
    y     = float(pose_a[1]) + s * float(pose_b[0]) + c * float(pose_b[1])
    theta = normalizar_angulo(theta_a + float(pose_b[2]))
    return np.array([x, y, theta], dtype=float)


def invertir_pose(pose: np.ndarray) -> np.ndarray:
    """Calcula la pose inversa: T^{-1}."""
    theta = float(pose[2])
    c = math.cos(theta)
    s = math.sin(theta)
    x = -(c * float(pose[0]) + s * float(pose[1]))
    y =  (s * float(pose[0]) - c * float(pose[1]))
    return np.array([x, y, normalizar_angulo(-theta)], dtype=float)


def cuaternion_a_yaw(orientacion) -> float:
    """Convierte un cuaternión a ángulo yaw (rotación en Z)."""
    siny = 2.0 * (orientacion.w * orientacion.z + orientacion.x * orientacion.y)
    cosy = 1.0 - 2.0 * (orientacion.y * orientacion.y + orientacion.z * orientacion.z)
    return math.atan2(siny, cosy)


# ─────────────────────────────────────────────────────────────
# Nodo principal: Localización con EKF
# ─────────────────────────────────────────────────────────────

class Localisation(Node):
    """
    Nodo de localización basado en Filtro de Kalman Extendido (EKF).

    Fusiona odometría de encoders de ruedas con observaciones de
    marcadores ArUco para estimar la pose del robot en el mundo.

    Entradas:
      - /VelocityEncR, /VelocityEncL : velocidades angulares de las ruedas
      - /aruco_markers               : MarkerArray con poses de marcadores

    Salidas:
      - /odom                        : pose estimada (Odometry)
      - TF odom → base_footprint     : árbol de transformadas dinámico
    """

    def __init__(self) -> None:
        super().__init__('localisation_node')

        # ── Parámetros del modelo cinemático ─────────────────
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base',   0.19)
        self.declare_parameter('sample_time',  0.02)

        # ── Ruido del modelo de movimiento (encoders) ─────────
        self.declare_parameter('sigma_v',  0.001)
        self.declare_parameter('sigma_w',  0.01)

        # ── Ruido del modelo de observación (ArUco) ───────────
        self.declare_parameter('sigma_obs_x',     0.08)
        self.declare_parameter('sigma_obs_y',     0.08)
        self.declare_parameter('sigma_obs_theta', 0.15)

        # ── Pose inicial del robot ────────────────────────────
        self.declare_parameter('x0',     -2.50)
        self.declare_parameter('y0',     -2.50)
        self.declare_parameter('theta0',  0.0)

        # ── Frames del árbol TF ───────────────────────────────
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # ── Topics de encoders ────────────────────────────────
        self.declare_parameter('wr_topic', '/VelocityEncR')
        self.declare_parameter('wl_topic', '/VelocityEncL')

        # ── Offset físico cámara → centro del robot ───────────
        self.declare_parameter('camera_base_x',     0.1241)
        self.declare_parameter('camera_base_y',     0.0)
        self.declare_parameter('camera_base_theta', 0.0)

        # ── Parámetros de la corrección ArUco ─────────────────
        self.declare_parameter('aruco_topic',               '/aruco_markers')
        self.declare_parameter('aruco_distance_gain',       0.5)
        self.declare_parameter('aruco_theta_distance_gain', 0.5)
        self.declare_parameter('mahal_threshold',           999999.0)

        # ── Mapa de marcadores ArUco (JSON) ───────────────────
        mapa_default = json.dumps([
            {'id': 0, 'x':  0.00, 'y': -3.50, 'theta': math.pi},
            {'id': 1, 'x':  3.00, 'y': -2.00, 'theta': 2.14},
            {'id': 2, 'x': -1.30, 'y':  1.30, 'theta': 0.0},
            {'id': 3, 'x': -3.00, 'y':  3.50, 'theta': 0.0},
            {'id': 4, 'x':  3.90, 'y':  2.90, 'theta': math.pi},
            {'id': 5, 'x': -1.40, 'y': -2.50, 'theta': math.pi},
        ])
        self.declare_parameter('marker_map_json', mapa_default)

        # ── Leer todos los parámetros ─────────────────────────
        self.radio        = float(self.get_parameter('wheel_radius').value)
        self.base         = float(self.get_parameter('wheel_base').value)
        self.dt           = float(self.get_parameter('sample_time').value)
        self.sigma_v      = float(self.get_parameter('sigma_v').value)
        self.sigma_w      = float(self.get_parameter('sigma_w').value)
        self.sigma_obs_x  = float(self.get_parameter('sigma_obs_x').value)
        self.sigma_obs_y  = float(self.get_parameter('sigma_obs_y').value)
        self.sigma_obs_th = float(self.get_parameter('sigma_obs_theta').value)
        self.frame_odom   = str(self.get_parameter('odom_frame').value)
        self.frame_base   = str(self.get_parameter('base_frame').value)
        self.topic_wr     = str(self.get_parameter('wr_topic').value)
        self.topic_wl     = str(self.get_parameter('wl_topic').value)
        self.topic_aruco  = str(self.get_parameter('aruco_topic').value)

        self.offset_camara = np.array([
            float(self.get_parameter('camera_base_x').value),
            float(self.get_parameter('camera_base_y').value),
            float(self.get_parameter('camera_base_theta').value),
        ], dtype=float)

        self.ganancia_dist_xy = float(self.get_parameter('aruco_distance_gain').value)
        self.ganancia_dist_th = float(self.get_parameter('aruco_theta_distance_gain').value)
        self.umbral_mahal     = float(self.get_parameter('mahal_threshold').value)

        # Parsear mapa de marcadores desde JSON
        self.mapa_aruco = self._parsear_mapa(
            str(self.get_parameter('marker_map_json').value))

        # ── Estado inicial del EKF ────────────────────────────
        self.mu = np.array([
            float(self.get_parameter('x0').value),
            float(self.get_parameter('y0').value),
            float(self.get_parameter('theta0').value),
        ], dtype=float)

        # Covarianza inicial pequeña (conocemos la pose inicial)
        self.sigma = np.diag([0.05, 0.05, 0.10]).astype(float)

        # Velocidades de las ruedas
        self.wr = 0.0
        self.wl = 0.0

        # Observación ArUco pendiente de procesar
        self.observacion_pendiente: list = []

        # ── Publicadores y suscriptores ───────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_odom       = self.create_publisher(Odometry, '/odom', 10)

        self.create_subscription(Float32,     self.topic_wr,    self.cb_wr,    10)
        self.create_subscription(Float32,     self.topic_wl,    self.cb_wl,    10)
        self.create_subscription(MarkerArray, self.topic_aruco, self.cb_aruco, 10)

        # Timer principal del EKF
        self.create_timer(self.dt, self.ciclo_ekf)

        self.get_logger().info(
            f'EKF listo | pose_init=({self.mu[0]:.2f},{self.mu[1]:.2f}) '
            f'| markers={len(self.mapa_aruco)} | topic={self.topic_aruco}')

    # ─────────────────────────────────────────────────────────
    # Callbacks de sensores
    # ─────────────────────────────────────────────────────────

    def cb_wr(self, msg: Float32) -> None:
        self.wr = float(msg.data)

    def cb_wl(self, msg: Float32) -> None:
        self.wl = float(msg.data)

    def cb_aruco(self, msg: MarkerArray) -> None:
        """Procesa el MarkerArray y guarda la observación más cercana."""
        candidatos = []
        for marker in getattr(msg, 'markers', []):
            mid = self._id_del_marker(marker)
            if mid is None or mid not in self.mapa_aruco:
                continue

            pose_camara = self._pose_del_marker(marker)
            if pose_camara is None:
                continue

            pose_robot = self._pose_robot_desde_marker(mid, pose_camara)
            if pose_robot is None:
                continue

            try:
                distancia = float(getattr(marker, 'scale').x)
            except Exception:
                distancia = float(np.linalg.norm(pose_camara))

            candidatos.append((pose_robot, distancia, mid))

        if not candidatos:
            self.observacion_pendiente = []
            return

        # Usar solo el marcador más cercano (mayor confianza)
        mejor_pose, mejor_dist, mejor_id = min(candidatos, key=lambda t: t[1])
        self.get_logger().info(
            f'ArUco id={mejor_id} | pose_robot=({mejor_pose[0]:.2f},'
            f'{mejor_pose[1]:.2f},{mejor_pose[2]:.2f}) | dist={mejor_dist:.2f}m')
        self.observacion_pendiente = [(np.array(mejor_pose, dtype=float), mejor_dist)]

    # ─────────────────────────────────────────────────────────
    # Ciclo principal del EKF
    # ─────────────────────────────────────────────────────────

    def ciclo_ekf(self) -> None:
        """Predicción + corrección + publicación a la frecuencia del timer."""
        self._prediccion()

        if self.observacion_pendiente:
            obs, dist = self.observacion_pendiente[0]
            self._correccion(obs, dist)
            self.observacion_pendiente = []

        self._publicar_estado()

    # ─────────────────────────────────────────────────────────
    # Paso de PREDICCIÓN del EKF
    # ─────────────────────────────────────────────────────────

    def _prediccion(self) -> None:
        """
        Propaga el estado con el modelo cinemático diferencial.
        Actualiza μ y Σ con el Jacobiano del modelo de movimiento.
        """
        v = 0.5 * self.radio * (self.wr + self.wl)
        w = self.radio * (self.wr - self.wl) / self.base

        theta = float(self.mu[2])
        c = math.cos(theta)
        s = math.sin(theta)

        # Actualizar pose estimada
        self.mu[0] += v * c * self.dt
        self.mu[1] += v * s * self.dt
        self.mu[2]  = normalizar_angulo(theta + w * self.dt)

        # Jacobiano respecto al estado
        F = np.array([
            [1.0, 0.0, -v * s * self.dt],
            [0.0, 1.0,  v * c * self.dt],
            [0.0, 0.0,  1.0],
        ], dtype=float)

        # Jacobiano respecto al ruido de proceso
        L = np.array([
            [c * self.dt, 0.0],
            [s * self.dt, 0.0],
            [0.0,         self.dt],
        ], dtype=float)

        # Covarianza del ruido de proceso
        Q = np.diag([self.sigma_v**2, self.sigma_w**2]).astype(float)

        # Propagación de covarianza
        self.sigma = F @ self.sigma @ F.T + L @ Q @ L.T
        self.sigma = 0.5 * (self.sigma + self.sigma.T)

    # ─────────────────────────────────────────────────────────
    # Paso de CORRECCIÓN del EKF
    # ─────────────────────────────────────────────────────────

    def _correccion(self, observacion: np.ndarray, distancia: float) -> None:
        """
        Corrige la estimación con la pose inferida desde un marcador ArUco.
        Aplica compuerta de Mahalanobis y escala el ruido con la distancia.
        """
        z = np.array([float(observacion[0]),
                      float(observacion[1]),
                      float(observacion[2])], dtype=float)

        # Innovación
        innovacion    = z - self.mu
        innovacion[2] = normalizar_angulo(float(innovacion[2]))

        # Escalar incertidumbre con la distancia al marcador
        factor_xy = max(1.0, 1.0 + self.ganancia_dist_xy * distancia)
        factor_th = max(1.0, 1.0 + self.ganancia_dist_th * distancia)

        R = np.diag([
            self.sigma_obs_x**2  * factor_xy,
            self.sigma_obs_y**2  * factor_xy,
            self.sigma_obs_th**2 * factor_th,
        ]).astype(float)

        # Matriz de innovación (H = I porque observamos el estado directamente)
        S = self.sigma + R

        # Compuerta de Mahalanobis
        try:
            dist_mahal = float(innovacion.T @ np.linalg.inv(S) @ innovacion)
        except Exception:
            dist_mahal = float('inf')

        if dist_mahal > self.umbral_mahal:
            self.get_logger().warn(
                f'[EKF] Corrección rechazada | Mahalanobis={dist_mahal:.2f} '
                f'> {self.umbral_mahal}')
            return

        # Ganancia de Kalman
        K = self.sigma @ np.linalg.inv(S)

        # Actualizar estado
        self.mu    = self.mu + K @ innovacion
        self.mu[2] = normalizar_angulo(float(self.mu[2]))

        # Actualizar covarianza (forma de Joseph)
        I  = np.eye(3, dtype=float)
        IK = I - K
        self.sigma = IK @ self.sigma @ IK.T + K @ R @ K.T
        self.sigma = 0.5 * (self.sigma + self.sigma.T)

        self.get_logger().info(
            f'[EKF] Corrección aplicada | '
            f'innovacion=({innovacion[0]:.3f},{innovacion[1]:.3f},'
            f'{innovacion[2]:.3f}) | Mahal={dist_mahal:.2f}')

    # ─────────────────────────────────────────────────────────
    # Publicación del estado
    # ─────────────────────────────────────────────────────────

    def _publicar_estado(self) -> None:
        """
        Publica la pose estimada como:
          - Odometry en /odom
          - TF odom → base_footprint  (árbol dinámico del robot)
        """
        x     = float(self.mu[0])
        y     = float(self.mu[1])
        theta = float(self.mu[2])
        qz    = math.sin(theta / 2.0)
        qw    = math.cos(theta / 2.0)
        sello = self.get_clock().now().to_msg()

        # ── Publicar TF dinámico: odom → base_footprint ──────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = sello
        tf_msg.header.frame_id = self.frame_odom
        tf_msg.child_frame_id  = self.frame_base
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z    = qz
        tf_msg.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        # ── Publicar Odometry ─────────────────────────────────
        odom = Odometry()
        odom.header.stamp            = sello
        odom.header.frame_id         = self.frame_odom
        odom.child_frame_id          = self.frame_base
        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocidades estimadas
        odom.twist.twist.linear.x  = 0.5 * self.radio * (self.wr + self.wl)
        odom.twist.twist.angular.z = self.radio * (self.wr - self.wl) / self.base

        # Covarianza 6x6 (solo llenamos los términos x, y, yaw)
        cov = [0.0] * 36
        cov[0]  = float(self.sigma[0, 0])
        cov[1]  = float(self.sigma[0, 1])
        cov[5]  = float(self.sigma[0, 2])
        cov[6]  = float(self.sigma[1, 0])
        cov[7]  = float(self.sigma[1, 1])
        cov[11] = float(self.sigma[1, 2])
        cov[30] = float(self.sigma[2, 0])
        cov[31] = float(self.sigma[2, 1])
        cov[35] = float(self.sigma[2, 2])
        odom.pose.covariance = cov

        self.pub_odom.publish(odom)

    # ─────────────────────────────────────────────────────────
    # Funciones auxiliares
    # ─────────────────────────────────────────────────────────

    def _parsear_mapa(self, json_str: str) -> dict:
        """Convierte el JSON del mapa de markers a {id: np.array([x,y,theta])}."""
        mapa = {}
        try:
            entradas = json.loads(json_str)
        except json.JSONDecodeError:
            self.get_logger().warn('marker_map_json inválido — mapa vacío')
            return mapa
        for e in entradas:
            try:
                mapa[int(e['id'])] = np.array(
                    [float(e['x']), float(e['y']), float(e['theta'])],
                    dtype=float)
            except (KeyError, TypeError, ValueError):
                continue
        return mapa

    def _id_del_marker(self, marker) -> int | None:
        """Extrae el ID del marcador del mensaje Marker."""
        for campo in ('id', 'marker_id'):
            val = getattr(marker, campo, None)
            if val is not None:
                try:
                    return int(val)
                except (TypeError, ValueError):
                    pass
        return None

    def _pose_del_marker(self, marker) -> np.ndarray | None:
        """Extrae la pose 2D (x, y, yaw) del mensaje Marker."""
        campo_pose = getattr(marker, 'pose', None)
        if campo_pose is None:
            return None
        if hasattr(campo_pose, 'pose'):
            campo_pose = campo_pose.pose
        if hasattr(campo_pose, 'position') and hasattr(campo_pose, 'orientation'):
            pos = campo_pose.position
            yaw = cuaternion_a_yaw(campo_pose.orientation)
            return np.array([float(pos.x), float(pos.y), yaw], dtype=float)
        return None

    def _pose_robot_desde_marker(self, marker_id, pose_camara):
        pose_mundo = self.mapa_aruco.get(marker_id)
        if pose_mundo is None:
            return None
        offset_inverso = invertir_pose(self.offset_camara)
        pose_robot = componer_poses(pose_mundo, 
                                    componer_poses(pose_camara, offset_inverso))
        # El theta calculado apunta hacia el marcador — corregir 180°
        pose_robot[2] = normalizar_angulo(pose_robot[2] + math.pi)
        return pose_robot


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Localisation()
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