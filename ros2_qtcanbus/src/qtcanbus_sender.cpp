#include <QCoreApplication>
#include <QTimer>
#include <QObject>

#include <rclcpp/rclcpp.hpp>

#include <libros2qt/qt_executor.h>
#include "qtcanbus_sender_node.h"

using namespace rclcpp;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    rclcpp::init(argc, argv);
    int res;

    try
    {
        auto server = std::make_shared<QtCanbusSenderNode>();

        QtExecutor executor;
        executor.add_node(server);

        executor.start();

        res = a.exec();
        RCLCPP_INFO(server->get_logger(), "Exited QT thread");
    }
    catch (std::exception &e)
    {
        res = 1;
    }

    rclcpp::shutdown();
    return res;
}