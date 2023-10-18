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

#pragma once

#include <QCanBusDevice>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QObject>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "ros2_qtcanbus_msg/msg/q_can_bus_frame.hpp"
#include "std_msgs/msg/int32.hpp"

class QtCanbusSenderNode : public QObject, public rclcpp::Node {
  Q_OBJECT

 public:
  QtCanbusSenderNode(QObject* parent = nullptr);
  virtual ~QtCanbusSenderNode() = default;

 private slots:
  void readFrames();

 private:
  QCanBusDevice* m_canDevice = nullptr;

  rclcpp::Publisher<ros2_qtcanbus_msg::msg::QCanBusFrame>::SharedPtr
      m_publisher;
  ros2_qtcanbus_msg::msg::QCanBusFrame m_message;
};