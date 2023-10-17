import rclpy
from rclpy.node import Node
import cantools
from rcl_interfaces.msg import ParameterDescriptor

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from ros2_qtcanbus_msg.msg import QCanBusFrame


class CandecodeNode(Node):
    def __init__(self):
        super().__init__("candecode_node")

        qos_history_desc = ParameterDescriptor(description='QoS history depth')
        warn_if_unknown_desc = ParameterDescriptor(description='Warn if unknown CAN ID received')
        dbc_file_desc = ParameterDescriptor(description='Path to DBC file to use for decoding')

        self.declare_parameter('qos_history', 100, qos_history_desc)
        self.declare_parameter('warn_if_unknown', False, warn_if_unknown_desc)
        self.declare_parameter('dbc_file', '/Users/tfoldi/Developer/mobility/canedge-influxdb-writer/dbc_files/Model3CAN.dbc', dbc_file_desc)

        dbc_file = self.get_parameter('dbc_file').get_parameter_value().string_value
        qos_history = self.get_parameter('qos_history').get_parameter_value().integer_value 
        self.warn_if_unknown = self.get_parameter('warn_if_unknown').get_parameter_value().bool_value
        
        self.db = cantools.database.load_file(dbc_file)

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            qos_history) 

        self.subscription = self.create_subscription(
            QCanBusFrame,
            'from_can_bus',
            self.listener_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.get_logger().info("Candecode initalized")

    def listener_callback(self, msg):
        self.get_logger().debug('received message: "%s/%s"' % (msg.id, hex(msg.id)))

        try:
            val = self.db.decode_message(msg.id, msg.data.copy(order='C'))

            status_msg = DiagnosticStatus()
            status_msg.level = DiagnosticStatus.OK
            status_msg.hardware_id = hex(msg.id)

            for key in val:
                key_value = KeyValue()
                key_value.key = key
                key_value.value = str(val[key])
                status_msg.values.append(key_value)

            status_msg.name = self.db.get_message_by_frame_id(msg.id).name

            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = msg.header.stamp
            diag_msg.header.frame_id = msg.header.frame_id
            diag_msg.status.append(status_msg)

            self.get_logger().debug(str(val))

            self.diagnostics_pub.publish(diag_msg)

        except KeyError:
            if self.warn_if_unknown:
                self.get_logger().warn('Unknown CAN ID: %s' % msg.id)
        except cantools.database.errors.DecodeError:
            self.get_logger().warn('Failed to decode CAN ID: %s/%s' % (msg.id, hex(msg.id)))


def main(args=None):
    rclpy.init(args=args)
    node = CandecodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()