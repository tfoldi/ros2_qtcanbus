#include <QDebug>
#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"

#include "qtcanbus_sender_node.h"

using namespace rclcpp;
using namespace std::chrono_literals;

TestNode::TestNode(QObject *parent):
    QObject (parent),
    Node("test_node")
{
    m_publisher = this->create_publisher<std_msgs::msg::Int32>("from_can_bus", 10);

    m_monotonicTimer.start();
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &TestNode::onQtTimerCB);
    m_timer->start(1000);


    m_periodic_timer = this->create_wall_timer(
          1s,
          [this]() {
            qDebug() << "@" << m_monotonicTimer.elapsed() << "ms : ros timer timeout from thread : " << QThread::currentThreadId();
    });
}

void TestNode::onQtTimerCB()
{
    m_message.data += 1;
    this->m_publisher->publish(m_message);
    qDebug() << "@" << m_monotonicTimer.elapsed() << "ms : QTimer timeout from thread : " << QThread::currentThreadId();
}