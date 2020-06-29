#include "refbox_interface.hpp"
#include <chrono>
#include <memory>
#include "colors.h"

using namespace std::chrono_literals;

Refbox_Interface::Refbox_Interface(QHash<int, QList<qint8>> robots, QString ipAddress, int port) : Node("refbox_node") {
    _port = port;
    _ip = QHostAddress(ipAddress);
    _socket = new QUdpSocket();
    _processCommand = true;
    _blueGk = 0;
    _yellowGk = 0;

    if(_socket->bind(QHostAddress::AnyIPv4, _port, QUdpSocket::ShareAddress)==false) {
        std::cout << "[ERROR] SSLReferee: bind socket error (" << _socket->errorString().toStdString() << ")\n";
    }

    if(_socket->joinMulticastGroup(_ip)==false) {
        std::cout << "[ERROR] SSLReferee: failed to join multicast group (" << _socket->errorString().toStdString() << ")\n"; 
    }

    _callback_group_client = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    _robots = robots;
    QList<int> keys = _robots.keys();
    std::string team;
    // For each team:
    for(int i = 0; i < keys.size(); i++) {
        if(keys[i] == Colors::BLUE) {
            team = "blue_";
        } else {
            team = "yellow_";
        }

        // For each player:
        QList<qint8> ids = _robots.value(keys[i]);
        QHash<qint8, rclcpp::Client<ctr_msgs::srv::State>::SharedPtr> element;
        for(int j = 0; j < ids.size(); j++) {
            auto client = this->create_client<ctr_msgs::srv::State>("state_service/"+team+std::to_string(ids[j]),
                                                                    rmw_qos_profile_services_default, _callback_group_client);
            element.insert(ids[j], client);
        }
        _clientTable.insert(keys[i], element);
    }

    // Internal loop
    _callback_group_timer = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    _timer = this->create_wall_timer(20ms, std::bind(&Refbox_Interface::callback, this), _callback_group_timer);
    std::cout << "Refbox node created!\n";
}

Refbox_Interface::~Refbox_Interface() {
    // Leave multicast group
    _socket->leaveMulticastGroup(_ip);
    _socket->close();

    // Delete socket
    delete _socket;    
}

void Refbox_Interface::callback() {
    SSL_Referee refereePacket;
    char buffer[65535];
    int len = 0;

    while(_socket->hasPendingDatagrams()) {

        len = _socket->readDatagram(buffer, 65535);

        if(refereePacket.ParseFromArray(buffer, len)==false) {
            std::cout << "[ERROR] SSLReferee: protobuf referee packet parsing error!\n";
        }

        _processCommand = (refereePacket.command_counter() != _lastPacket.command_counter());
        _lastPacket.CopyFrom(refereePacket);
    }

    if(_processCommand) {
        updateStates();
    }
}

void Refbox_Interface::updateStates() {
    // Update gk's:
    if(_lastPacket.yellow().has_goalie()) {
        _yellowGk = _lastPacket.yellow().goalie();
    }

    if(_lastPacket.blue().has_goalie()) {
        _blueGk = _lastPacket.blue().goalie();
    }

    SSL_Referee_Command ref_command = _lastPacket.command();
    switch(ref_command) {
        case SSL_Referee_Command_TIMEOUT_YELLOW:
        case SSL_Referee_Command_TIMEOUT_BLUE:
        case SSL_Referee_Command_HALT:
            _blueState = "HALT";
            _yellowState = _blueState;
        break;
        
        case SSL_Referee_Command_GOAL_YELLOW:
        case SSL_Referee_Command_GOAL_BLUE:
        case SSL_Referee_Command_STOP: 
            _blueState = "GAMEOFF";
            _yellowState = _blueState;
        break;

        case SSL_Referee_Command_FORCE_START:
            _blueState = "FORCESTART";
            _yellowState = _blueState;
        break;

        case SSL_Referee_Command_NORMAL_START:
            _blueState = "NORMALSTART";
            _yellowState = _blueState;
        break;

        case SSL_Referee_Command_PREPARE_KICKOFF_YELLOW:  
            _blueState = "THEIRKICKOFF";
            _yellowState = "OURKICKOFF";
        break;    

        case SSL_Referee_Command_PREPARE_KICKOFF_BLUE:  
            _blueState = "OURKICKOFF";
            _yellowState = "THEIRKICKOFF";
        break;   

        case SSL_Referee_Command_PREPARE_PENALTY_YELLOW:
            _blueState = "THEIRPENALTY";
            _yellowState = "OURPENALTY";
        break;   
    
        case SSL_Referee_Command_PREPARE_PENALTY_BLUE:
            _blueState = "OURPENALTY";
            _yellowState = "THEIRPENALTY";
        break;   

        case SSL_Referee_Command_DIRECT_FREE_YELLOW:
            _blueState = "THEIRDIRECTKICK";
            _yellowState = "OURDIRECTKICK";
        break;

        case SSL_Referee_Command_DIRECT_FREE_BLUE:
            _blueState = "OURDIRECTKICK";
            _yellowState = "THEIRDIRECTKICK";
        break;

        case SSL_Referee_Command_INDIRECT_FREE_YELLOW:
            _blueState = "THEIRINDIRECTKICK";
            _yellowState = "OURINDIRECTKICK";
        break;

        case SSL_Referee_Command_INDIRECT_FREE_BLUE:
            _blueState = "OURINDIRECTKICK";
            _yellowState = "THEIRINDIRECTKICK";
        break;
    }

    QList<int> teamKeys = _clientTable.keys();
    for(int i = 0; i < teamKeys.size(); i++) {
        QList<qint8> ids = _clientTable.value(teamKeys[i]).keys();
        if(teamKeys[i] == Colors::BLUE) {
            for(int j = 0; j < ids.size(); j++) {
              std::cout << "test\n";
                auto request = std::make_shared<ctr_msgs::srv::State::Request>();
                request->state = _blueState;
                if(ids[j] == _blueGk) {
                    request->isgk = true;
                } else {
                    request->isgk = false;
                }

                // Wait for service:
                while(!_clientTable.value(teamKeys[i]).value(ids[j])->wait_for_service(1s)) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                }

                auto result = _clientTable.value(teamKeys[i]).value(ids[j])->async_send_request(request);
                auto status = result.wait_for(std::chrono::seconds(3));
                if(status == std::future_status::ready) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response received: %s", result.get()->feedback.c_str());
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to send status");
                }
            }
        } else {
            for(int j = 0; j < ids.size(); j++) {
                auto request = std::make_shared<ctr_msgs::srv::State::Request>();
                request->state = _yellowState;
                if(ids[j] == _yellowGk) {
                    request->isgk = true;
                } else {
                    request->isgk = false;
                }

                // Wait for service:
                while(!_clientTable.value(teamKeys[i]).value(ids[j])->wait_for_service(1s)) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                }

                auto result = _clientTable.value(teamKeys[i]).value(ids[j])->async_send_request(request);
                auto status = result.wait_for(std::chrono::seconds(3));
                if(status == std::future_status::ready) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response received is %s", result.get()->feedback.c_str());
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failed to send status");
                }
            }            
        }
    }
}
