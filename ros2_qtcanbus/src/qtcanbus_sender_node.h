#pragma once

#include <QCoreApplication>
#include <QTimer>
#include <QObject>
#include <QElapsedTimer>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class TestNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    TestNode(QObject* parent=nullptr);
    virtual ~TestNode() = default;

private slots:
    void onQtTimerCB();

private:

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisher;
    std_msgs::msg::Int32 m_message;


    QTimer *m_timer;
    QElapsedTimer m_monotonicTimer;
    rclcpp::TimerBase::SharedPtr m_periodic_timer;
};