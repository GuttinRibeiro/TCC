#include "refbox_interface.hpp"
#include <chrono>
#include <memory>
#include "colors.h"

using namespace std::chrono_literals;

static const rmw_qos_profile_t rmw_qos_custom_profile {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    100,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

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

    // Callback mutualmente exclusivas para evitar o envio de dados que não sejam os mais recentes
    _callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto pub_opt = rclcpp::PublisherOptions();
    pub_opt.callback_group = _callback_group;

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
        QHash<qint8, rclcpp::Publisher<ctr_msgs::msg::State>::SharedPtr> element;
        for(int j = 0; j < ids.size(); j++) {
            auto client = this->create_publisher<ctr_msgs::msg::State>("external_agent/state/"+team+std::to_string(ids[j]),
                                                                    rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_custom_profile.history, rmw_qos_custom_profile.depth), rmw_qos_custom_profile),
                                                                    pub_opt);
            element.insert(ids[j], client);
        }
        _clientTable.insert(keys[i], element);
    }

    // Internal loop
    _timer = this->create_wall_timer(20ms, std::bind(&Refbox_Interface::callback, this), _callback_group);
    std::cout << "Refbox node was created!\n";
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
                auto message = ctr_msgs::msg::State();
                message.state = _blueState;
                if(ids[j] == _blueGk) {
                    message.isgk = true;
                } else {
                    message.isgk = false;
                }

                _clientTable.value(teamKeys[i]).value(ids[j])->publish(message);
            }
        } else {
            for(int j = 0; j < ids.size(); j++) {
                auto message = ctr_msgs::msg::State();
                message.state = _yellowState;
                if(ids[j] == _yellowGk) {
                    message.isgk = true;
                } else {
                    message.isgk = false;
                }

                _clientTable.value(teamKeys[i]).value(ids[j])->publish(message);
            }            
        }
    }
}
