#ifndef GRSIM_ACTUATOR_SUB_HH
#define GRSIM_ACTUATOR_SUB_HH

#include "actuator_sub.hpp"
#include <QUdpSocket>
#include <string>
#include <QMap>

typedef struct cmd {
  bool spinner;
  float kickspeedy;
  float kickspeedz;
} Command;

class Grsim_Actuator_Sub : public Actuator_Sub {
    private:
        // Internal variables:
        QUdpSocket _socket;
        Command _cmd_buffer;

        // Implementing virtual methods:
        void send_cmd(ctr_msgs::msg::Command::SharedPtr msg);
        void send_velocity(ctr_msgs::msg::Velocity::SharedPtr msg);

    public:
        Grsim_Actuator_Sub(std::string team, int id, std::string nodeName, int nodeID, QHostAddress grsimAddress = QHostAddress(QHostAddress::LocalHost), int port = 20011);
};

#endif
