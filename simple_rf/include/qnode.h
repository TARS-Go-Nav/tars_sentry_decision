#pragma once

#include <QObject>
#include <QThread>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "tars_msgs/msg/serial_receive.hpp"

class QNode : public QThread {
    Q_OBJECT
public:
    QNode();
    tars_msgs::msg::SerialReceive rcv_msg;

protected:
    void run();

Q_SIGNALS:
	void rosShutdown();

private:
    void timerCallback();
    rclcpp::Publisher<tars_msgs::msg::SerialReceive>::SharedPtr serial_rcv_pub;
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<rclcpp::Node> node;

signals:
    void emitTopicData(QString);
};

