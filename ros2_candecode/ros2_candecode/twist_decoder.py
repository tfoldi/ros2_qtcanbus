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
from rclpy.qos_overriding_options import QoSOverridingOptions
from geometry_msgs.msg import TwistStamped


class TwistDecoder:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        self.node.declare_parameters(
            namespace="",
            parameters=[
                ("twist_mapping.canbus_message_id", -1),
                ("twist_mapping.frame_id", ""),
                ("twist_mapping.uom", "kph"),
                ("twist_mapping.field_mapping.linear.x", ""),
                ("twist_mapping.field_mapping.linear.y", ""),
                ("twist_mapping.field_mapping.linear.z", ""),
                ("twist_mapping.field_mapping.angular.x", ""),
                ("twist_mapping.field_mapping.angular.y", ""),
                ("twist_mapping.field_mapping.angular.z", ""),
            ],
        )
        self.canbus_message_id = node.get_parameter(
            "twist_mapping.canbus_message_id"
        ).value
        self.frame_id = node.get_parameter("twist_mapping.frame_id").value

        if self.canbus_message_id == -1:
            self.get_logger().debug(
                "No CAN frame ID specified for Twist message. No conversion will be performed."
            )
            return
        else:
            self.get_logger().info(
                "Twist started with the following CAN frame ID: %d"
                % self.canbus_message_id
            )

        self.field_mapping = {}
        self.field_mapping["linear_x"] = node.get_parameter(
            "twist_mapping.field_mapping.linear.x"
        ).value
        self.field_mapping["linear_y"] = node.get_parameter(
            "twist_mapping.field_mapping.linear.y"
        ).value
        self.field_mapping["linear_z"] = node.get_parameter(
            "twist_mapping.field_mapping.linear.z"
        ).value
        self.field_mapping["angular_x"] = node.get_parameter(
            "twist_mapping.field_mapping.angular.x"
        ).value
        self.field_mapping["angular_y"] = node.get_parameter(
            "twist_mapping.field_mapping.angular.y"
        ).value
        self.field_mapping["angular_z"] = node.get_parameter(
            "twist_mapping.field_mapping.angular.z"
        ).value

        self.uom = node.get_parameter("twist_mapping.uom").value

        self.twist_pub = self.node.create_publisher(
            TwistStamped,
            "/vel",
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.get_logger().info(
            "TwistDecoder started with the following message mappings: %s"
            % self.field_mapping
        )

    def is_twist_message(self, msg_id):
        return self.canbus_message_id == msg_id

    def decode_twist_message(self, canframe, msg):
        self.get_logger().debug("Decoding Twist message: %s" % canframe)

        twist_msg = TwistStamped()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.header.stamp = msg.header.stamp

        multiplier = 1.0
        if self.uom == "kph":
            multiplier = 1.0 / 3.6

        if self.field_mapping["linear_x"]:
            twist_msg.twist.linear.x = (
                canframe.get(self.field_mapping["linear_x"]) * multiplier
            )

        if self.field_mapping["linear_y"]:
            twist_msg.twist.linear.y = (
                canframe.get(self.field_mapping["linear_y"]) * multiplier
            )

        if self.field_mapping["linear_z"]:
            twist_msg.twist.linear.z = (
                canframe.get(self.field_mapping["linear_z"]) * multiplier
            )

        if self.field_mapping["angular_x"]:
            twist_msg.twist.angular.x = (
                canframe.get(self.field_mapping["angular_x"]) * multiplier
            )

        if self.field_mapping["angular_y"]:
            twist_msg.twist.angular.y = (
                canframe.get(self.field_mapping["angular_y"]) * multiplier
            )

        if self.field_mapping["angular_z"]:
            twist_msg.twist.angular.z = (
                canframe.get(self.field_mapping["angular_z"]) * multiplier
            )

        self.twist_pub.publish(twist_msg)
