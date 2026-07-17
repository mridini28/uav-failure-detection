import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry


class ResponseNode(Node):

    def __init__(self):
        super().__init__('response_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.anomaly_active = False
        self.hold_position = None  # captured position to hold at, once anomaly triggers
        self.current_position = None
        self.offboard_engaged = False
        self.offboard_setpoint_counter = 0

        self.create_subscription(
            Bool, '/uav_monitor/anomaly_detected',
            self.anomaly_callback, 10)

        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odometry_callback, qos_profile)

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def anomaly_callback(self, msg):
        was_active = self.anomaly_active
        self.anomaly_active = msg.data

        # On the rising edge (anomaly just started), capture current position to hold
        if self.anomaly_active and not was_active:
            if self.current_position is not None:
                self.hold_position = [float(x) for x in self.current_position]
                self.get_logger().warn(
                    f"ANOMALY TRIGGERED - engaging hold at {self.hold_position}")

    def odometry_callback(self, msg):
        self.current_position = msg.position

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent (response node)')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_hold_setpoint(self):
        if self.hold_position is None:
            return
        msg = TrajectorySetpoint()
        msg.position = self.hold_position
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        if not self.anomaly_active:
            return  # do nothing while nominal - don't interfere with normal flight

        # Anomaly is active: publish offboard heartbeat + hold setpoint
        self.publish_offboard_control_mode()
        self.publish_hold_setpoint()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = ResponseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()