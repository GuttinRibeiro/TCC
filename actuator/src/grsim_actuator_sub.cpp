#include "grsim_actuator_sub.hpp"
#include "grSim_Replacement.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Packet.pb.h"
#include <iostream>

Grsim_Actuator_Sub::Grsim_Actuator_Sub(QHash<int, QList<qint8>> control_table, std::string nodeName, int nodeID, QHostAddress grsimAddress, int port) : Actuator_Sub(control_table, nodeName, nodeID) {
    _socket.connectToHost(grsimAddress, port, QIODevice::WriteOnly);
}

void Grsim_Actuator_Sub::send_cmd(ctr_msgs::msg::Command::SharedPtr msg) {
    std::cout << "Command received!\n";
    grSim_Packet packet;

    // Set team and player number
    packet.mutable_commands()->set_isteamyellow(msg->team == "yellow");
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id((int)msg->id);

    // Robot command
    command->set_wheelsspeed(false);
    command->set_kickspeedx(msg->kickspeedy);

    if(msg->chipkick) {
        command->set_kickspeedz(msg->kickspeedz);
    } else {
        command->set_kickspeedz(0.0);
    }

    command->set_spinner(msg->spinner);
    command->set_velnormal(msg->vx);
    command->set_veltangent(msg->vy);
    command->set_velangular(msg->vang);

    // Send packet
    std::string s;
    packet.SerializeToString(&s);
    if(_socket.write(s.c_str(), s.length()) == -1) {
        std::cout << "[grSim Actuator] Failed to write to socket: " << _socket.errorString().toStdString() << "\n";
    }
}