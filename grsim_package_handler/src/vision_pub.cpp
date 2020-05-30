#include "vision_pub.hpp"
#include "utils.hpp"
#include <chrono>
#include <memory>
#include <QList>
#include <iostream>
#include <cmath>
#define MAX_ROBOTS 6

using namespace std::chrono_literals;

enum Colors {
    BLUE,
    YELLOW
};


VisionNode::VisionNode(RoboCupSSLClient *client) : Node("grsim_node") {
    // Yellow robots
    for(qint8 i = 0; i < MAX_ROBOTS; i++) {
        _publisherYellow.insert(i, this->create_publisher<grsim_package_handler::msg::Visionpkg>("yellow_"+std::to_string(i), 10));
    }

    // Blue robots
    for(qint8 i = 0; i < MAX_ROBOTS; i++) {
        _publisherBlue.insert(i, this->create_publisher<grsim_package_handler::msg::Visionpkg>("blue_"+std::to_string(i), 10));
    }

    _client = client;
    // Nota: achei muito estranho isso de bindar com um timer. Tentei criar uma thread, mas
    // programa crasha. Checar como o client da visão funciona, provavelmente terei que jogar
    // um tempo minúsculo aqui.
    clock_gettime(CLOCK_REALTIME, &_start);
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
    split_packages();

    // Create and publish a message to each topic
    //auto message = std_msgs::msg::String();
    //message.data = "Publishing!";
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //_publisher->publish(message);
}


// Welcome to the hell!
void VisionNode::split_packages() {
    // Publish messages for blue robots: [ok]
    QList<qint8> ids = _blueTeam.keys();
    for(int i = 0; i < ids.size(); i++) {
        auto message = grsim_package_handler::msg::Visionpkg();
        Robot selected = _blueTeam.value(ids[i]);

        // Its own position:
        auto rob_msg = grsim_package_handler::msg::Robot();
        rob_msg.x = selected.position().x();
        rob_msg.y = selected.position().y();
        rob_msg.id = selected.id();
        rob_msg.team = "blue";
        message.robots.push_back(rob_msg);

        // Blue team:
        for(int j = 0; j < ids.size(); j++) {
            if(j != i) {
                Robot rob = _blueTeam.value(ids[j]);

                // If rob is within sensor range:
                if(Utils::isWithinInterval(selected.getSensor().minRange(), selected.getSensor().maxRange(), Utils::distance(selected.position(), rob.position()))) {
                    float angle = Utils::wrapToTwoPi(Utils::getAngle(rob.position(), selected.position()));
                    float orientation = Utils::wrapToTwoPi(selected.orientation());
                    float diff = Utils::angleDiff(angle, orientation);
                    if(Utils::isWithinInterval(selected.getSensor().minAngle(), selected.getSensor().maxAngle(), diff)) {
                        //[TODO] Check for obstruction:
                        auto rob_msg = grsim_package_handler::msg::Robot();
                        rob_msg.x = rob.position().x();
                        rob_msg.y = rob.position().y();
                        rob_msg.id = rob.id();
                        rob_msg.team = "blue";
                        message.robots.push_back(rob_msg);
                    }
                }
            }
        }

        // Yellow team:
        for(int j = 0; j < ids.size(); j++) {
            Robot rob = _yellowTeam.value(ids[j]);
            if(Utils::isWithinInterval(selected.getSensor().minRange(), selected.getSensor().maxRange(), Utils::distance(selected.position(), rob.position()))) {
                float angle = Utils::wrapToTwoPi(Utils::getAngle(rob.position(), selected.position()));
                float orientation = Utils::wrapToTwoPi(selected.orientation());
                float diff = Utils::angleDiff(angle, orientation);
                if(Utils::isWithinInterval(selected.getSensor().minAngle(), selected.getSensor().maxAngle(), diff)) {
                    //[TODO] Check for obstruction:
                    auto rob_msg = grsim_package_handler::msg::Robot();
                    rob_msg.x = rob.position().x();
                    rob_msg.y = rob.position().y();
                    rob_msg.id = rob.id();
                    rob_msg.team = "yellow";
                    message.robots.push_back(rob_msg);
                }
            }
        }

        // Ball:
        // If the ball is within sensor range:
        if(Utils::isWithinInterval(selected.getSensor().minRange(), selected.getSensor().maxRange(), Utils::distance(selected.position(), _ball))) {
            float angle = Utils::wrapToTwoPi(Utils::getAngle(_ball, selected.position()));
            float orientation = Utils::wrapToTwoPi(selected.orientation());
            float diff = Utils::angleDiff(angle, orientation);
            if(Utils::isWithinInterval(selected.getSensor().minAngle(), selected.getSensor().maxAngle(), diff)) {
                //[TODO] Check for obstruction:
                auto ball_msg = grsim_package_handler::msg::Ball();
                ball_msg.x = _ball.x();
                ball_msg.y = _ball.y();
                message.balls.push_back(ball_msg);
            }
        }

        // Publish the desired (I guess) message:
        clock_gettime(CLOCK_REALTIME, &_stop);
        message.timestamp = ((_stop.tv_sec*1E9+_stop.tv_nsec)-(_start.tv_sec*1E9+_start.tv_nsec))/1E9;
        _publisherBlue.value(ids[i])->publish(message);
    }

    //Publish messages for yellow robots: [ok]
    ids = _yellowTeam.keys();
    for(int i = 0; i < ids.size(); i++) {
        auto message = grsim_package_handler::msg::Visionpkg();
        Robot selected = _yellowTeam.value(ids[i]);

        // Its own position:
        auto rob_msg = grsim_package_handler::msg::Robot();
        rob_msg.x = selected.position().x();
        rob_msg.y = selected.position().y();
        rob_msg.id = selected.id();
        rob_msg.team = "yellow";
        message.robots.push_back(rob_msg);

        // Blue team:
        for(int j = 0; j < ids.size(); j++) {
            Robot rob = _blueTeam.value(ids[j]);
            // If rob is within sensor range:
            if(Utils::isWithinInterval(selected.getSensor().minRange(), selected.getSensor().maxRange(), Utils::distance(selected.position(), rob.position()))) {
                float angle = Utils::wrapToTwoPi(Utils::getAngle(rob.position(), selected.position()));
                float orientation = Utils::wrapToTwoPi(selected.orientation());
                float diff = Utils::angleDiff(angle, orientation);
                if(Utils::isWithinInterval(selected.getSensor().minAngle(), selected.getSensor().maxAngle(), diff)) {
                    //[TODO] Check for obstruction:
                    auto rob_msg = grsim_package_handler::msg::Robot();
                    rob_msg.x = rob.position().x();
                    rob_msg.y = rob.position().y();
                    rob_msg.id = rob.id();
                    rob_msg.team = "blue";
                    message.robots.push_back(rob_msg);
                }
            }
        }

        // Yellow team:
        for(int j = 0; j < ids.size(); j++) {
            if(j != i) {
                Robot rob = _yellowTeam.value(ids[j]);

                // If rob is within sensor range:
                if(Utils::isWithinInterval(selected.getSensor().minRange(), selected.getSensor().maxRange(), Utils::distance(selected.position(), rob.position()))) {
                    float angle = Utils::wrapToTwoPi(Utils::getAngle(rob.position(), selected.position()));
                    float orientation = Utils::wrapToTwoPi(selected.orientation());
                    float diff = Utils::angleDiff(angle, orientation);
                    if(Utils::isWithinInterval(selected.getSensor().minAngle(), selected.getSensor().maxAngle(), diff)) {
                        //[TODO] Check for obstruction:
                        auto rob_msg = grsim_package_handler::msg::Robot();
                        rob_msg.x = rob.position().x();
                        rob_msg.y = rob.position().y();
                        rob_msg.id = rob.id();
                        rob_msg.team = "yellow";
                        message.robots.push_back(rob_msg);
                }
            }
            }
        }

        // Ball:
        // If the ball is within sensor range:
        if(Utils::isWithinInterval(selected.getSensor().minRange(), selected.getSensor().maxRange(), Utils::distance(selected.position(), _ball))) {
            float angle = Utils::wrapToTwoPi(Utils::getAngle(_ball, selected.position()));
            float orientation = Utils::wrapToTwoPi(selected.orientation());
            float diff = Utils::angleDiff(angle, orientation);
            if(Utils::isWithinInterval(selected.getSensor().minAngle(), selected.getSensor().maxAngle(), diff)) {
                //[TODO] Check for obstruction:
                auto ball_msg = grsim_package_handler::msg::Ball();
                ball_msg.x = _ball.x();
                ball_msg.y = _ball.y();
                message.balls.push_back(ball_msg);
            }
        }

        // Publish the desired (I guess) message:
        _publisherYellow.value(ids[i])->publish(message);
    }    

}

void VisionNode::update() {
    // Parse camera data
    QList<std::pair<int,SSL_DetectionBall> > balls = parseCamerasBalls(_detectionPackets.values());
    QHash<int,std::pair<int,SSL_DetectionRobot> > robots = parseCamerasRobots(_detectionPackets.values());

    processBalls(balls);
    processRobots(robots);
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
    for(qint8 i = 0; i < MAX_ROBOTS; i++) {
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
        Robot aux(pos, -1, i);
        if(numOri > 0) {
            aux.setOrientation(realOri/numFrames);
        }
        Sensor sen("camera", 20.0, -M_PI/4, M_PI/4);
        aux.addSensor(sen);
        ret.insert(i, aux);
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
