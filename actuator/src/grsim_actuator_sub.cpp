#include "grsim_actuator_sub.hpp"
#include "grSim_Replacement.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Packet.pb.h"
#include <iostream>

Grsim_Actuator_Sub::Grsim_Actuator_Sub(std::string team, int id, std::string nodeName, int nodeID, QHostAddress grsimAddress, int port) : Actuator_Sub(team, id, nodeName, nodeID) {
  _socket.connectToHost(grsimAddress, port, QIODevice::WriteOnly);
  _cmd_buffer.spinner = false;
  _cmd_buffer.kickspeedy = 0.0;
  _cmd_buffer.kickspeedz = 0.0;
}

void Grsim_Actuator_Sub::send_cmd(ctr_msgs::msg::Command::SharedPtr msg) {
  if(msg->hasholdballinformation) {
    _cmd_buffer.spinner = msg->holdball;
  }

  if(msg->haskickinformation) {
    _cmd_buffer.kickspeedy = msg->kickspeedy;
    _cmd_buffer.kickspeedz = msg->kickspeedz;
  }
  std::cout << "Command received!\n";
}

void Grsim_Actuator_Sub::send_velocity(ctr_msgs::msg::Velocity::SharedPtr msg) {
  grSim_Packet packet;

  // Set team and player number
  packet.mutable_commands()->set_isteamyellow(_team == "yellow");
  packet.mutable_commands()->set_timestamp(0.0);
  grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
  command->set_id((int)_id);

  // Robot command
  command->set_wheelsspeed(false);

  // Command information
  command->set_kickspeedx(_cmd_buffer.kickspeedy);
  command->set_kickspeedz(_cmd_buffer.kickspeedz);
  command->set_spinner(_cmd_buffer.spinner);

  // Velocity information
  command->set_velnormal(-msg->vx);
  command->set_veltangent(msg->vy);
  command->set_velangular(msg->vang);

  // Send packet
  std::string s;
  packet.SerializeToString(&s);
  if(_socket.write(s.c_str(), s.length()) == -1) {
      std::cout << "[grSim Actuator] Failed to write to socket: " << _socket.errorString().toStdString() << "\n";
  }
}
