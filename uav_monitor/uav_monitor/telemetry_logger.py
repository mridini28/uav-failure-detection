import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    VehicleOdometry,
    SensorCombined,
    EstimatorStatusFlags,
    BatteryStatus,
    SensorGps,
)


class TelemetryLogger(Node):

    def __init__(self):
        super().__init__('telemetry_logger')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Latest received messages, cached
        self.odometry = None
        self.sensor_combined = None
        self.estimator_status = None
        self.battery_status = None
        self.gps_position = None

        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odometry_callback, qos_profile)
        self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self.sensor_combined_callback, qos_profile)
        self.create_subscription(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags',
            self.estimator_status_callback, qos_profile)
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1',
            self.battery_status_callback, qos_profile)
        self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position',
            self.gps_position_callback, qos_profile)

        # Print a unified snapshot at 10Hz
        self.timer = self.create_timer(0.1, self.log_snapshot)

    def odometry_callback(self, msg):
        self.odometry = msg

    def sensor_combined_callback(self, msg):
        self.sensor_combined = msg

    def estimator_status_callback(self, msg):
        self.estimator_status = msg

    def battery_status_callback(self, msg):
        self.battery_status = msg

    def gps_position_callback(self, msg):
        self.gps_position = msg

    def log_snapshot(self):
        # Only print once we have at least some data
        if self.odometry is None and self.gps_position is None:
            return

        parts = []

        if self.odometry is not None:
            pos = self.odometry.position
            parts.append(f"pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")

        if self.gps_position is not None:
            parts.append(f"sats={self.gps_position.satellites_used}")

        if self.battery_status is not None:
            parts.append(f"batt={self.battery_status.remaining:.2f}")

        if self.estimator_status is not None:
            parts.append(f"gnss_fault={self.estimator_status.cs_gnss_fault}")

        self.get_logger().info(" | ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()