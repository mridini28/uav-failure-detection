import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor

from px4_msgs.msg import SensorGps


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter(
            'fault_mode', 'none',
            ParameterDescriptor(
                description="Fault mode: none, low_sats, frozen, dropout"))

        self.last_good_msg = None

        self.subscription = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position',
            self.gps_callback, qos_profile)

        self.publisher = self.create_publisher(
            SensorGps, '/fault/vehicle_gps_position', qos_profile)

        self.get_logger().info(
            "Fault injector running. Change mode with:\n"
            "  ros2 param set /fault_injector fault_mode <none|low_sats|frozen|dropout>")

    def gps_callback(self, msg):
        mode = self.get_parameter('fault_mode').get_parameter_value().string_value

        if mode == 'dropout':
            # Simply don't publish anything -> downstream sees a stale topic
            return

        if mode == 'frozen':
            # Republish the last known-good message instead of the new one
            if self.last_good_msg is not None:
                self.publisher.publish(self.last_good_msg)
            else:
                self.publisher.publish(msg)
                self.last_good_msg = msg
            return

        if mode == 'low_sats':
            corrupted = msg
            corrupted.satellites_used = 2
            self.publisher.publish(corrupted)
            self.last_good_msg = msg
            return

        # mode == 'none' (default): pass through unchanged
        self.publisher.publish(msg)
        self.last_good_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()