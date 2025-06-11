#include "qnode.h"

QNode::QNode() {
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("ros2_qt_demo");
    serial_rcv_pub = node->create_publisher<tars_msgs::msg::SerialReceive>("/serial_receive_debug", 10);
    timer = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&QNode::timerCallback, this));
    rcv_msg = tars_msgs::msg::SerialReceive();
    this->start();
}

void QNode::run() {
    rclcpp::spin(node);
    Q_EMIT rosShutdown();
    rclcpp::shutdown();
}

void QNode::timerCallback() {
    serial_rcv_pub->publish(rcv_msg);
}
