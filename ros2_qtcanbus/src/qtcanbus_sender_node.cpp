#include <algorithm>
#include <QObject>
#include <QString>
#include <QThread>
#include <QtSerialBus/QCanBus>
#include <QtSerialBus/QCanBusFrame>

#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include "ros2_qtcanbus_msg/msg/q_can_bus_frame.hpp"

#include "qtcanbus_sender_node.h"

using namespace rclcpp;
using namespace std::chrono_literals;

QtCanbusSenderNode::QtCanbusSenderNode(QObject *parent) : QObject(parent),
                                                          Node("qtcanbus_sender_node")
{
    QString errorString;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
    m_publisher = this->create_publisher<ros2_qtcanbus_msg::msg::QCanBusFrame>("from_can_bus", qos);

    RCLCPP_INFO_STREAM(this->get_logger(), "Available CAN plugins: " << QCanBus::instance()->plugins().join(", ").toStdString());

    const QList<QCanBusDeviceInfo> devices = QCanBus::instance()->availableDevices(QStringLiteral("clx000can"), &errorString);

    for (int i = 0; i < devices.count(); i++)
        RCLCPP_INFO_STREAM(this->get_logger(), "Found device: " << devices[i].name().toStdString());

    // TODO: from parameters
    m_canDevice = QCanBus::instance()->createDevice("clx000can", "cu.usbmodem123456781", &errorString);
    if (!m_canDevice)
    {
        RCLCPP_FATAL_STREAM(this->get_logger(), "Error creating device: " << errorString.toStdString());
        throw std::runtime_error("Error creating device");    
    }

    // Connect the frameReceived signal to our slot
    connect(m_canDevice, &QCanBusDevice::framesReceived, this, &QtCanbusSenderNode::readFrames);

    // Connect error signal. Reconnection should be handled here.
    connect(m_canDevice, &QCanBusDevice::errorOccurred, [this](QCanBusDevice::CanBusError error)
            { RCLCPP_ERROR_STREAM(this->get_logger(), "CAN Bus Error: "
                                                          << m_canDevice->errorString().toStdString()
                                                          << " Error code: " << error); });

    // Start the CAN interface
    if (!m_canDevice->connectDevice())
    {
        RCLCPP_FATAL_STREAM(this->get_logger(), "Connection error: " << m_canDevice->errorString().toStdString());
        delete m_canDevice;
        m_canDevice = nullptr;
        throw std::runtime_error("Connection error");
    }

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Connected to CAN device");

    m_message.header.frame_id = "can";
}

void QtCanbusSenderNode::readFrames()
{
    while (m_canDevice->framesAvailable())
    {
        const QCanBusFrame frame = m_canDevice->readFrame();
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Received frame with ID " << frame.frameId()
                  << " and payload: " << frame.payload().toHex().toStdString()
                  << " and size: " << frame.payload().size());

        m_message.header.stamp = this->now();

        m_message.id = frame.frameId();
        m_message.dlc = frame.payload().size(); 
        m_message.is_extended = frame.hasExtendedFrameFormat();
        m_message.is_error = frame.hasErrorStateIndicator();

        // for (int i = 0; i < std::min(frame.payload().size(),(int)m_message.data.size()) ; i++)
        // {
        //     m_message.data[i] = frame.payload().at(i);
        // }
        memcpy(m_message.data.data(), frame.payload().data(), std::min(frame.payload().size(),(int)m_message.data.size()) );

        this->m_publisher->publish(m_message);
    }
}
