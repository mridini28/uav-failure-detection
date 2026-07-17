import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String
from px4_msgs.msg import SensorGps

import joblib
import os

WINDOW_SIZE = 20
EPSILON = 1e-30

MODEL_PATH = os.path.expanduser('~/px4_ros_ws/ml/anomaly_model.joblib')


class MLAnomalyDetector(Node):

    def __init__(self):
        super().__init__('ml_anomaly_detector')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.model = joblib.load(MODEL_PATH)
        self.get_logger().info(f"Loaded model from {MODEL_PATH}")

        self.gps_lat_window = deque(maxlen=WINDOW_SIZE)
        self.gps_lon_window = deque(maxlen=WINDOW_SIZE)

        self.create_subscription(
            SensorGps, '/fault/vehicle_gps_position',
            self.gps_callback, qos_profile)

        self.anomaly_flag_publisher = self.create_publisher(
            Bool, '/uav_monitor/ml_anomaly_detected', 10)
        self.anomaly_score_publisher = self.create_publisher(
            String, '/uav_monitor/ml_anomaly_score', 10)

    def compute_variance(self, window):
        if len(window) < 2:
            return 0.0
        vals = list(window)
        mean = sum(vals) / len(vals)
        return sum((v - mean) ** 2 for v in vals) / len(vals)

    def gps_callback(self, msg):
        self.gps_lat_window.append(msg.latitude_deg)
        self.gps_lon_window.append(msg.longitude_deg)

        if len(self.gps_lat_window) < WINDOW_SIZE:
            return  # not enough history yet

        lat_var = self.compute_variance(self.gps_lat_window)
        lon_var = self.compute_variance(self.gps_lon_window)

        log_lat_var = np.log10(lat_var + EPSILON)
        log_lon_var = np.log10(lon_var + EPSILON)

        features = np.array([[log_lat_var, log_lon_var]])
        prediction = self.model.predict(features)[0]  # -1 = anomaly, 1 = normal
        score = self.model.decision_function(features)[0]

        is_anomaly = prediction == -1

        flag_msg = Bool()
        flag_msg.data = bool(is_anomaly)
        self.anomaly_flag_publisher.publish(flag_msg)

        score_msg = String()
        score_msg.data = f"score={score:.4f}, lat_var={lat_var:.2e}, lon_var={lon_var:.2e}"
        self.anomaly_score_publisher.publish(score_msg)

        if is_anomaly:
            self.get_logger().warn(f"ML ANOMALY DETECTED: {score_msg.data}")
        else:
            self.get_logger().info(f"nominal ({score_msg.data})")


def main(args=None):
    rclpy.init(args=args)
    node = MLAnomalyDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()