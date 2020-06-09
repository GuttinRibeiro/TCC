#ifndef GRSIM_ACTUATOR_SUB_HH
#define GRSIM_ACTUATOR_SUB_HH

#include "actuator_sub.hpp"
#include <QUdpSocket>

class Grsim_Actuator_Sub : public Actuator_Sub {
    private:
        // Internal variables:
        QUdpSocket _socket;

        // Implementing virtual methods:
        void send_cmd(ctr_msgs::msg::Command::SharedPtr msg);

    public:
        Grsim_Actuator_Sub(QHash<int, QList<qint8>> control_table, std::string nodeName, int nodeID, QHostAddress grsimAddress = QHostAddress(QHostAddress::LocalHost), int port = 20011);
};

#endif