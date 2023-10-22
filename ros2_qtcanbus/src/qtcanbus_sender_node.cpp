/*
 * Copyright (c) 2023, Tamas Foldi
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of mosquitto nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QObject>
#include <QString>
#include <QThread>
#include <QtSerialBus/QCanBus>
#include <QtSerialBus/QCanBusFrame>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include "ros2_qtcanbus_msgs/msg/q_can_bus_frame.hpp"

#include "qtcanbus_sender_node.h"

using namespace rclcpp;
using namespace std::chrono_literals;

QtCanbusSenderNode::QtCanbusSenderNode(QObject* parent)
    : QObject(parent), Node("qtcanbus_sender_node") {
  QString errorString;

  auto canbus_plugin_desc = rcl_interfaces::msg::ParameterDescriptor{};
  canbus_plugin_desc.description = "The CAN bus plugin to use";

  auto canbus_interface_desc = rcl_interfaces::msg::ParameterDescriptor{};
  canbus_interface_desc.description = "The CAN bus interface to use";

  this->declare_parameter("canbus_plugin", "socketcan", canbus_plugin_desc);
  this->declare_parameter("canbus_interface", "vcan0", canbus_interface_desc);

  auto canbus_plugin = this->get_parameter("canbus_plugin").as_string();
  auto canbus_interface = this->get_parameter("canbus_interface").as_string();

  RCLCPP_INFO_STREAM(this->get_logger(), "Connecting to CAN device "
                                             << canbus_plugin << ":"
                                             << canbus_interface.c_str());

  auto qos = rclcpp::QoS(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
  m_publisher = this->create_publisher<ros2_qtcanbus_msgs::msg::QCanBusFrame>(
      "from_can_bus", qos);

  RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Available CAN plugins: "
          << QCanBus::instance()->plugins().join(", ").toStdString());

  const QList<QCanBusDeviceInfo> devices =
      QCanBus::instance()->availableDevices(canbus_plugin.c_str(),
                                            &errorString);

  for (int i = 0; i < devices.count(); i++)
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "Found device for plugin: " << devices[i].name().toStdString());

  m_canDevice = QCanBus::instance()->createDevice(
      canbus_plugin.c_str(), canbus_interface.c_str(), &errorString);
  if (!m_canDevice) {
    RCLCPP_FATAL_STREAM(this->get_logger(),
                        "Error creating device: " << errorString.toStdString());
    throw std::runtime_error("Error creating device");
  }

  // Connect the frameReceived signal to our slot
  connect(m_canDevice, &QCanBusDevice::framesReceived, this,
          &QtCanbusSenderNode::readFrames);

  // Connect error signal. Reconnection should be handled here.
  connect(m_canDevice, &QCanBusDevice::errorOccurred,
          [this](QCanBusDevice::CanBusError error) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "CAN Bus Error: " << m_canDevice->errorString().toStdString()
                                  << " Error code: " << error);
          });

  // Start the CAN interface
  if (!m_canDevice->connectDevice()) {
    RCLCPP_FATAL_STREAM(
        this->get_logger(),
        "Connection error: " << m_canDevice->errorString().toStdString());
    delete m_canDevice;
    m_canDevice = nullptr;
    throw std::runtime_error("Connection error");
  }

  RCLCPP_INFO(this->get_logger(), "Connected to CAN device %s:%s",
              canbus_plugin.c_str(), canbus_interface.c_str());

  m_message.header.frame_id = "can";
}

void QtCanbusSenderNode::readFrames() {
  while (m_canDevice->framesAvailable()) {
    const QCanBusFrame frame = m_canDevice->readFrame();
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Received frame with ID "
                            << frame.frameId() << " and payload: "
                            << frame.payload().toHex().toStdString()
                            << " and size: " << frame.payload().size());

    m_message.header.stamp = this->now();

    m_message.id = frame.frameId();
    m_message.dlc = frame.payload().size();
    m_message.is_extended = frame.hasExtendedFrameFormat();
    m_message.is_error = frame.hasErrorStateIndicator();

    memcpy(m_message.data.data(), frame.payload().data(),
           std::min(frame.payload().size(), (int)m_message.data.size()));

    this->m_publisher->publish(m_message);
  }
}
