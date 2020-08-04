#include "grsim_actuator_sub.hpp"
#include "grSim_Replacement.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Packet.pb.h"
#include <iostream>

Grsim_Actuator_Sub::Grsim_Actuator_Sub(std::string team, int id, std::string nodeName, int nodeID, QHostAddress grsimAddress, int port) : Actuator_Sub(team, id, nodeName, nodeID) {
    _socket.connectToHost(grsimAddress, port, QIODevice::WriteOnly);
}

void Grsim_Actuator_Sub::send_cmd(ctr_msgs::msg::Command::SharedPtr msg) {
    Command cmd;
    cmd.spinner = msg->spinner;
    cmd.kickspeedy = msg->kickspeedy;
    cmd.kickspeedz = msg->kickspeedz;
    auto pair = std::make_pair(msg->team, msg->id);
    _cmd_buffer.insert(pair, cmd);
}

void Grsim_Actuator_Sub::send_velocity(ctr_msgs::msg::Velocity::SharedPtr msg) {
    grSim_Packet packet;

    // Set team and player number
    packet.mutable_commands()->set_isteamyellow(msg->team == "yellow");
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id((int)msg->id);

    // Robot command
    command->set_wheelsspeed(false);

    // Command information
    auto key = std::make_pair(msg->team, msg->id);
    if(_cmd_buffer.contains(key)) {
      Command cmd = _cmd_buffer.value(key);
//        _cmd_buffer.remove(key);
      command->set_kickspeedx(cmd.kickspeedy);
      command->set_kickspeedz(cmd.kickspeedz);
      command->set_spinner(cmd.spinner);
    } else {
      command->set_kickspeedx(0.0);
      command->set_kickspeedz(0.0);
      command->set_spinner(false);
    }

    // Velocity information
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
