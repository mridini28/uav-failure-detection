import csv
import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry, BatteryStatus, SensorGps

WINDOW_SIZE = 20  # ~2 seconds at 10Hz


class DataCollector(Node):

    def __init__(self):
        super().__init__('data_collector')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odometry = None
        self.battery_status = None
        self.gps_position = None

        # Rolling windows of recent GPS lat/lon (to detect "frozen" GPS)
        self.gps_lat_window = deque(maxlen=WINDOW_SIZE)
        self.gps_lon_window = deque(maxlen=WINDOW_SIZE)

        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odometry_callback, qos_profile)
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1',
            self.battery_status_callback, qos_profile)
        self.create_subscription(
            SensorGps, '/fault/vehicle_gps_position',
            self.gps_position_callback, qos_profile)

        self.declare_parameter('output_file', 'flight_data_normal.csv')
        output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.filepath = os.path.expanduser(f'~/px4_ros_ws/{output_file}')

        self.csv_file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp', 'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'satellites_used', 'battery_remaining',
            'gps_lat_variance', 'gps_lon_variance', 'gps_window_filled'
        ])

        self.get_logger().info(f"Logging telemetry to {self.filepath}")

        self.timer = self.create_timer(0.1, self.log_row)

    def odometry_callback(self, msg):
        self.odometry = msg

    def battery_status_callback(self, msg):
        self.battery_status = msg

    def gps_position_callback(self, msg):
        self.gps_position = msg
        self.gps_lat_window.append(msg.latitude_deg)
        self.gps_lon_window.append(msg.longitude_deg)

    def compute_variance(self, window):
        if len(window) < 2:
            return 0.0
        vals = list(window)
        mean = sum(vals) / len(vals)
        return sum((v - mean) ** 2 for v in vals) / len(vals)

    def log_row(self):
        if self.odometry is None:
            return

        pos = self.odometry.position
        vel = self.odometry.velocity
        sats = self.gps_position.satellites_used if self.gps_position else -1
        batt = self.battery_status.remaining if self.battery_status else -1.0

        lat_var = self.compute_variance(self.gps_lat_window)
        lon_var = self.compute_variance(self.gps_lon_window)
        window_filled = 1 if len(self.gps_lat_window) == WINDOW_SIZE else 0

        self.writer.writerow([
            time.time(), pos[0], pos[1], pos[2],
            vel[0], vel[1], vel[2],
            sats, batt,
            lat_var, lon_var, window_filled
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()