#ifndef REFBOX_INTERFACE_HH
#define REFBOX_INTERFACE_HH

#include "rclcpp/rclcpp.hpp"
#include <QtCore>
#include <QNetworkInterface>
#include <QUdpSocket>
#include <ctime>
#include <string>
#include "referee.pb.h"
#include "ctr_msgs/msg/state.hpp"

class Refbox_Interface : public rclcpp::Node {
    private:
        QHash<int, QList<qint8>> _robots;
        QHostAddress _ip;
        int _port;
        QUdpSocket *_socket;
        rclcpp::TimerBase::SharedPtr _timer;
        SSL_Referee _lastPacket;
        bool _processCommand;
        std::string _blueState;
        std::string _yellowState;
        qint8 _blueGk;
        qint8 _yellowGk;
        QHash<int, QHash<qint8, rclcpp::Publisher<ctr_msgs::msg::State>::SharedPtr>> _clientTable;

        rclcpp::CallbackGroup::SharedPtr _callback_group;

        void callback();
        void updateStates();
    public:
        Refbox_Interface(QHash<int, QList<qint8>> robots, QString ipAddress = "224.5.23.1", int port = 10003);
        ~Refbox_Interface();
};

#endif
