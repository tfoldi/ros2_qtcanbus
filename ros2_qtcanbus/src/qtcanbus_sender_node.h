#pragma once

#include <QCoreApplication>
#include <QTimer>
#include <QObject>
#include <QCanBusDevice>
#include <QElapsedTimer>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_qtcanbus_msg/msg/q_can_bus_frame.hpp"

class QtCanbusSenderNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    QtCanbusSenderNode(QObject* parent=nullptr);
    virtual ~QtCanbusSenderNode() = default;

private slots:
    void readFrames();

private:

    QCanBusDevice *m_canDevice = nullptr;

    rclcpp::Publisher<ros2_qtcanbus_msg::msg::QCanBusFrame>::SharedPtr m_publisher;
    ros2_qtcanbus_msg::msg::QCanBusFrame m_message;


};