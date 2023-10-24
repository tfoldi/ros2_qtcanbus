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
from sensor_msgs.msg import NavSatFix
import numpy as np


class GPSDecoder:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        self.node.declare_parameters(
            namespace="",
            parameters=[
                ("navsat_mapping.canbus_message_id", -1),
                ("navsat_mapping.frame_id", ""),
                ("navsat_mapping.field_mapping.latitude", ""),
                ("navsat_mapping.field_mapping.longitude", ""),
                ("navsat_mapping.field_mapping.altitude", ""),
                ("navsat_mapping.field_mapping.status", ""),
                ("navsat_mapping.field_mapping.accuracy", ""),
            ],
        )
        self.canbus_message_id = node.get_parameter(
            "navsat_mapping.canbus_message_id"
        ).value
        self.frame_id = node.get_parameter("navsat_mapping.frame_id").value

        if self.canbus_message_id == -1:
            self.get_logger().debug(
                "No CAN frame ID specified for NavSatFix message. No conversion will be performed."
            )
            return
        else:
            self.get_logger().info(
                "GPSDecoder started with the following CAN frame ID: %d"
                % self.canbus_message_id
            )

        self.field_mapping = {}
        self.field_mapping["latitude"] = node.get_parameter(
            "navsat_mapping.field_mapping.latitude"
        ).value
        self.field_mapping["longitude"] = node.get_parameter(
            "navsat_mapping.field_mapping.longitude"
        ).value
        self.field_mapping["altitude"] = node.get_parameter(
            "navsat_mapping.field_mapping.altitude"
        ).value
        self.field_mapping["status"] = node.get_parameter(
            "navsat_mapping.field_mapping.status"
        ).value
        self.field_mapping["accuracy"] = node.get_parameter(
            "navsat_mapping.field_mapping.accuracy"
        ).value

        self.navsat_fix_pub = self.node.create_publisher(
            NavSatFix,
            "/navsat/fix",
            # rclpy.qos.qos_profile_soensor_data,
            100,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        self.get_logger().info(
            "GPSDecoder started with the following message mappings: %s"
            % self.field_mapping
        )

    def is_navsat_message(self, msg_id):
        return self.canbus_message_id == msg_id

    def decode_gps_message(self, canframe, msg):
        self.get_logger().debug("Decoding GPS message: %s" % canframe)

        navsat_msg = NavSatFix()
        navsat_msg.header.frame_id = self.frame_id
        navsat_msg.header.stamp = msg.header.stamp

        if self.field_mapping["latitude"]:
            navsat_msg.latitude = canframe.get(self.field_mapping["latitude"])

        if self.field_mapping["longitude"]:
            navsat_msg.longitude = canframe.get(self.field_mapping["longitude"])

        if self.field_mapping["altitude"]:
            navsat_msg.altitude = canframe.get(self.field_mapping["altitude"])

        if self.field_mapping["status"]:
            navsat_msg.status.status = canframe.get(self.field_mapping["status"])
        else:
            navsat_msg.status.status = 0

        navsat_msg.status.service = 1  # GPS

        # Assuming gps_accuracy is the accuracy value extracted from the GPS signal
        if canframe.get(self.field_mapping["accuracy"]) is not None:
            gps_accuracy = canframe.get(self.field_mapping["accuracy"])
        else:
            gps_accuracy = 1.0

        # Variance is the square of the standard deviation (accuracy in this case)
        variance = gps_accuracy**2

        # Create a 3x3 diagonal covariance matrix with the variance value
        cov_matrix = np.eye(3) * variance

        # Convert the 3x3 covariance matrix to a 1D array of 9 elements
        position_covariance = cov_matrix.flatten()

        # Set the covariance type to COVARIANCE_TYPE_DIAGONAL_KNOWN (2)
        position_covariance_type = 2

        navsat_msg.position_covariance = position_covariance
        navsat_msg.position_covariance_type = position_covariance_type

        self.get_logger().debug("Publishing NavSatFix message: %s" % navsat_msg)

        self.navsat_fix_pub.publish(navsat_msg)
