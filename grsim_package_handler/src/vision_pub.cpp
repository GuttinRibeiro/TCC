#include "vision_pub.hpp"
#include <chrono>
#include <memory>
#include <QList>
#include <iostream>
#define MAX_ROBOTS 6

using namespace std::chrono_literals;

enum Colors {
    BLUE,
    YELLOW
};


VisionNode::VisionNode(RoboCupSSLClient *client) : Node("grsim_node") {
    // Yellow robots
    for(int i = 0; i < MAX_ROBOTS; i++) {
        _publisherYellow.append(this->create_publisher<std_msgs::msg::String>("yellow_"+std::to_string(i), 10));
    }

    // Blue robots
    for(int i = 0; i < MAX_ROBOTS; i++) {
        _publisherBlue.append(this->create_publisher<std_msgs::msg::String>("blue_"+std::to_string(i), 10));
    }

    _client = client;
    // Nota: achei muito estranho isso de bindar com um timer. Tentei criar uma thread, mas
    // programa crasha. Checar como o client da visão funciona, provavelmente terei que jogar
    // um tempo minúsculo aqui.
    _timer = this->create_wall_timer(5ms, std::bind(&VisionNode::client_callback, this));
    _ball = Position();
}

void VisionNode::client_callback() {
    SSL_WrapperPacket packet;

    // Wait for package
    if(_client->receive(packet)){
        if(packet.has_detection()) {
            // Get data
            SSL_DetectionFrame currDetection = packet.detection();

            // Store detection packet for camera
            const int currCam = currDetection.camera_id();
            _detectionPackets.insert(currCam, currDetection);
        }

        // Meio que posso igonorar os pacotes do campo, já que assumo o campo como estático
        // e percebido pela visão embarcada, mas deixa aqui a atualização caso precise no 
        // futuro
        if(packet.has_geometry()) {
            // Store geometry data
            _geometryPacket = packet.geometry();
        }

        update();
    }

    // TODO: split info into topics

    // Create and publish a message to each topic
    //auto message = std_msgs::msg::String();
    //message.data = "Publishing!";
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //_publisher->publish(message);
}

void VisionNode::update() {
    // Parse camera data
    QList<std::pair<int,SSL_DetectionBall> > balls = parseCamerasBalls(_detectionPackets.values());
    QHash<int,std::pair<int,SSL_DetectionRobot> > robots = parseCamerasRobots(_detectionPackets.values());

    processBalls(balls);
    processRobots(robots);

    //[Debug]
    //Robot aux = _blueTeam.value(5);
    //std::cout << "Blue robot 5: " << aux.position().x() << ", " << aux.position().y() << "\n";
    //std::cout << "Blue robot 5: " << aux.orientation() << "\n";
}

void VisionNode::processBalls(const QList<std::pair<int,SSL_DetectionBall> > &balls) {

    // Merge multiple balls
    QList<std::pair<int,SSL_DetectionBall> >::const_iterator it;
    int numBalls = balls.size();
    float realX = 0.0;
    float realY = 0.0;
    //RCLCPP_INFO(this->get_logger(), "Number of balls: %d", balls.size());
    for(it=balls.constBegin(); it!=balls.constEnd(); it++) {
        const int camId = it->first;
        const SSL_DetectionBall ball = it->second;

        realX += ball.x()*MM2METER;
        realY += ball.y()*MM2METER;

        // Confidence
        if(ball.has_confidence()==false)
            continue;

        // [DEBUG] Print ball position
        //std::cout << "[Before merge] Ball pos: " << ball.x()*MM2METER << ", " << ball.y()*MM2METER << "\n";
    }

    Position ballAux(realX/numBalls, realY/numBalls, 0.0, false);
    _ball = ballAux;

    // [DEBUG] Print ball position
    //std::cout << "[After merge] Ball pos: " << _ball.x() << ", " << _ball.y() << "\n";
}

void VisionNode::processRobots(const QHash<int, std::pair<int,SSL_DetectionRobot> > &robots) {
    QList<std::pair<int, SSL_DetectionRobot>> yellowTeam = robots.values(Colors::YELLOW);
    QList<std::pair<int, SSL_DetectionRobot>> blueTeam = robots.values(Colors::BLUE);

    _yellowTeam = processTeam(yellowTeam);
    _blueTeam = processTeam(blueTeam);
}

QHash<qint8, Robot> VisionNode::processTeam(QList<std::pair<int, SSL_DetectionRobot>> team) {
    float realX = 0;
    float realY = 0;
    float realOri = 0;
    int numFrames = 0;
    int numOri = 0;
    QHash<qint8, Robot> ret;

    // For each id:
    for(unsigned i = 0; i < MAX_ROBOTS; i++) {
        QList<std::pair<int,SSL_DetectionRobot> >::iterator it;

        // Sum the coordinates of each frame for the same robot and apply a mean
        realX = 0.0;
        realY = 0.0;
        realOri = 0.0;
        numFrames = 0;
        numOri = 0;
        for(it=team.begin(); it!=team.end(); it++) {
            const SSL_DetectionRobot robot = it->second;

            // Discard tobots with wrong id
            if(robot.robot_id() != i || robot.has_confidence() == false) {
                continue;
            }

            if(robot.has_x() && robot.has_y()) {
                realX += robot.x()*MM2METER;
                realY += robot.y()*MM2METER;
                numFrames++;
                if(robot.has_orientation()) {
                    realOri += robot.orientation();
                    numOri++;
                }
            }
        }

        Position pos(realX/numFrames, realY/numFrames, 0.0, false);
        Robot aux(pos, -1, (char) i);
        if(numOri > 0) {
            aux.setOrientation(realOri/numFrames);
        }

        ret.insert((char) i, aux);
    }

    return ret;
}

QList<std::pair<int,SSL_DetectionBall> > VisionNode::parseCamerasBalls(const QList<SSL_DetectionFrame> &detectionFrames) const {
    QList<std::pair<int,SSL_DetectionBall> > retn;

    // Run on detection frames
    QList<SSL_DetectionFrame>::const_iterator it;
    for(it=detectionFrames.constBegin(); it!=detectionFrames.constEnd(); it++) {
        SSL_DetectionFrame frame = *it;

        // Add to retn list
        for(int i=0; i<frame.balls_size(); i++)
            retn.append(std::make_pair(frame.camera_id(), frame.balls(i)));
    }

    return retn;
}

QHash<int,std::pair<int,SSL_DetectionRobot> > VisionNode::parseCamerasRobots(const QList<SSL_DetectionFrame> &detectionFrames) const {
    QHash<int,std::pair<int,SSL_DetectionRobot> > retn;

    // Run on detection frames
    QList<SSL_DetectionFrame>::const_iterator it;
    for(it=detectionFrames.constBegin(); it!=detectionFrames.constEnd(); it++) {
        SSL_DetectionFrame frame = *it;

        // Add yellow to retn list
        for(int i=0; i<frame.robots_yellow_size(); i++)
            retn.insertMulti(Colors::YELLOW, std::make_pair(frame.camera_id(), frame.robots_yellow(i)));

        // Add blue to retn list
        for(int i=0; i<frame.robots_blue_size(); i++)
            retn.insertMulti(Colors::BLUE, std::make_pair(frame.camera_id(), frame.robots_blue(i)));
    }

    return retn;
}
