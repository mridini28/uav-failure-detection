import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String

from px4_msgs.msg import (
    VehicleOdometry,
    EstimatorStatusFlags,
    BatteryStatus,
    SensorGps,
)

# --- Thresholds (tune these as you test) ---
MIN_SATELLITES = 6
MIN_BATTERY = 0.15
MAX_POSITION_JUMP = 2.0  # meters, between consecutive 0.1s samples
STALE_TIMEOUT = {
    'odometry': 0.5,
    'estimator_status': 3.5,   # publishes ~1Hz, give it headroom
    'battery_status': 3.5,
    'gps_position': 0.5,
}

class AnomalyDetector(Node):

    def __init__(self):
        super().__init__('anomaly_detector')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odometry = None
        self.estimator_status = None
        self.battery_status = None
        self.gps_position = None
        self.last_position = None

        # Wall-clock timestamps of last received message per topic
        self.last_seen = {
            'odometry': None,
            'estimator_status': None,
            'battery_status': None,
            'gps_position': None,
        }

        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odometry_callback, qos_profile)
        self.create_subscription(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
            self.estimator_status_callback, qos_profile)
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1',
            self.battery_status_callback, qos_profile)
        self.create_subscription(
            SensorGps, '/fault/vehicle_gps_position',
            self.gps_position_callback, qos_profile)
        self.anomaly_flag_publisher = self.create_publisher(Bool, '/uav_monitor/anomaly_detected', 10)
        self.anomaly_detail_publisher = self.create_publisher(String, '/uav_monitor/anomaly_detail', 10)
        self.timer = self.create_timer(0.1, self.check_anomalies)

    def odometry_callback(self, msg):
        self.odometry = msg
        self.last_seen['odometry'] = time.monotonic()

    def estimator_status_callback(self, msg):
        self.estimator_status = msg
        self.last_seen['estimator_status'] = time.monotonic()

    def battery_status_callback(self, msg):
        self.battery_status = msg
        self.last_seen['battery_status'] = time.monotonic()

    def gps_position_callback(self, msg):
        self.gps_position = msg
        self.last_seen['gps_position'] = time.monotonic()

    def is_stale(self, key):
        last = self.last_seen[key]
        if last is None:
            return False  # never received yet, not "stale", just not started
        return (time.monotonic() - last) > STALE_TIMEOUT[key]

    def check_anomalies(self):
        anomalies = []

        # Staleness checks (topic stopped publishing = failure)
        if self.is_stale('gps_position'):
            anomalies.append("GPS_TOPIC_STALE")
        if self.is_stale('odometry'):
            anomalies.append("ODOMETRY_TOPIC_STALE")
        if self.is_stale('estimator_status'):
            anomalies.append("ESTIMATOR_TOPIC_STALE")
        if self.is_stale('battery_status'):
            anomalies.append("BATTERY_TOPIC_STALE")

        # Rule 1: GPS satellite drop (only meaningful if not stale)
        if self.gps_position is not None and not self.is_stale('gps_position'):
            if self.gps_position.satellites_used < MIN_SATELLITES:
                anomalies.append(
                    f"LOW_SATELLITES({self.gps_position.satellites_used})")

        # Rule 2: GNSS fault flag from EKF
        if self.estimator_status is not None and not self.is_stale('estimator_status'):
            if self.estimator_status.cs_gnss_fault:
                anomalies.append("GNSS_FAULT_FLAG")

        # Rule 3: Battery critical
        if self.battery_status is not None and not self.is_stale('battery_status'):
            if self.battery_status.remaining < MIN_BATTERY:
                anomalies.append(
                    f"LOW_BATTERY({self.battery_status.remaining:.2f})")

        # Rule 4: Position discontinuity
        if self.odometry is not None and not self.is_stale('odometry'):
            pos = self.odometry.position
            if self.last_position is not None:
                dx = pos[0] - self.last_position[0]
                dy = pos[1] - self.last_position[1]
                dz = pos[2] - self.last_position[2]
                jump = (dx**2 + dy**2 + dz**2) ** 0.5
                if jump > MAX_POSITION_JUMP:
                    anomalies.append(f"POSITION_JUMP({jump:.2f}m)")
            self.last_position = pos

        flag_msg = Bool()
        flag_msg.data = bool(anomalies)
        self.anomaly_flag_publisher.publish(flag_msg)

        detail_msg = String()
        detail_msg.data = ", ".join(anomalies) if anomalies else "nominal"
        self.anomaly_detail_publisher.publish(detail_msg)

        if anomalies:
            self.get_logger().warn(f"ANOMALY DETECTED: {', '.join(anomalies)}")
        else:
            self.get_logger().info("nominal")


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()