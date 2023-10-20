# Copyright 2023, Tamas Foldi
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos_overriding_options import QoSOverridingOptions

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

        decode_choices_desc = ParameterDescriptor(
            description="Decode choices as strings"
        )
        warn_if_unknown_desc = ParameterDescriptor(
            description="Warn if unknown CAN ID received"
        )
        dbc_file_desc = ParameterDescriptor(
            description="Path to DBC file to use for decoding"
        )

        self.declare_parameter("decode_choices", False, decode_choices_desc)
        self.declare_parameter("warn_if_unknown", False, warn_if_unknown_desc)
        self.declare_parameter(
            "dbc_file",
            "signals.dbc",
            dbc_file_desc,
        )

        self.decode_choices = (
            self.get_parameter("decode_choices").get_parameter_value().bool_value
        )
        dbc_file = self.get_parameter("dbc_file").get_parameter_value().string_value
        self.warn_if_unknown = (
            self.get_parameter("warn_if_unknown").get_parameter_value().bool_value
        )

        self.db = cantools.database.load_file(dbc_file)

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.subscription = self.create_subscription(
            QCanBusFrame,
            "from_can_bus",
            self.listener_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info("Candecode initalized")

    def listener_callback(self, msg):
        self.get_logger().debug('received message: "%s/%s"' % (msg.id, hex(msg.id)))

        try:
            val = self.db.decode_message(
                msg.id, msg.data.copy(order="C"), decode_choices=self.decode_choices
            )

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
                self.get_logger().warn("Unknown CAN ID: %s" % msg.id)
        except cantools.database.errors.DecodeError:
            self.get_logger().warn(
                "Failed to decode CAN ID: %s/%s" % (msg.id, hex(msg.id))
            )


def main(args=None):
    rclpy.init(args=args)
    node = CandecodeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
