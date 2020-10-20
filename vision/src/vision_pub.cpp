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
        _publisherYellow.insert(i, this->create_publisher<ctr_msgs::msg::Visionpkg>("vision/yellow_"+std::to_string(i), 10));
    }

    // Blue robots
    for(qint8 i = 0; i < MAX_ROBOTS; i++) {
        _publisherBlue.insert(i, this->create_publisher<ctr_msgs::msg::Visionpkg>("vision/blue_"+std::to_string(i), 10));
    }

    _client = client;
    clock_gettime(CLOCK_REALTIME, &_start);
    _timer = this->create_wall_timer(5ms, std::bind(&VisionNode::client_callback, this));
    _ball = Position();
}

void VisionNode::client_callback() {
    //clock_gettime(CLOCK_REALTIME, &_tstart);
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

    split_packages();
    //clock_gettime(CLOCK_REALTIME, &_tstop);
    //std::cout << "Tempo para dividir pacotes: " << ((_tstop.tv_sec*1E9+_tstop.tv_nsec)-(_tstart.tv_sec*1E9+_tstart.tv_nsec))/1E6 << " ms\n";
}

// Welcome to the hell!
void VisionNode::split_packages() {
    // Publish messages for blue robots: [ok]
    QList<qint8> ids = _blueTeam.keys();
    QList<Robot> visible_robots;
    for(int i = 0; i < ids.size(); i++) {
        visible_robots.clear();
        auto message = ctr_msgs::msg::Visionpkg();
        Robot selected = _blueTeam.value(ids[i]);

        // Its own position:
        auto rob_msg = ctr_msgs::msg::Robot();
        rob_msg.pos.x = selected.position().x();
        rob_msg.pos.y = selected.position().y();
        rob_msg.pos.z = 0.0;
        rob_msg.id = selected.id();
        rob_msg.pos.confidence = selected.position().confidence();
        rob_msg.orientation = Utils::wrapToTwoPi(selected.orientation());
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
                        visible_robots.append(rob);
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
                    visible_robots.append(rob);
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
                //Check for obstruction:
                if(checkVisibility(selected.position(), visible_robots, _ball, selected.radius())) {
                    auto ball_msg = ctr_msgs::msg::Position();
                    ball_msg.x = _ball.x();
                    ball_msg.y = _ball.y();
                    ball_msg.z = 0.0;
                    ball_msg.confidence = _ball.confidence();
                    message.balls.push_back(ball_msg);
                }
            }
        }

        // Check visible robots:
        for(int j = 0; j < visible_robots.size(); j++) {
            Robot target = visible_robots.takeAt(j);
            if(checkVisibility(selected.position(), visible_robots, target.position(), 2*selected.radius())) {
                auto rob_msg = ctr_msgs::msg::Robot();
                rob_msg.pos.x = target.position().x();
                rob_msg.pos.y = target.position().y();
                rob_msg.pos.z = 0.0;
                rob_msg.pos.confidence = target.position().confidence();
                rob_msg.id = target.id();
                rob_msg.team = target.team() == Colors::YELLOW ? "yellow" : "blue";
                message.robots.push_back(rob_msg);
            }
            visible_robots.insert(j, target);
        }

        // Publish the desired message:
        clock_gettime(CLOCK_REALTIME, &_stop);
        message.timestamp = ((_stop.tv_sec*1E9+_stop.tv_nsec)-(_start.tv_sec*1E9+_start.tv_nsec))/1E9;
        _publisherBlue.value(ids[i])->publish(message);
    }

    //Publish messages for yellow robots: [ok]
    ids = _yellowTeam.keys();
    for(int i = 0; i < ids.size(); i++) {
        visible_robots.clear();
        auto message = ctr_msgs::msg::Visionpkg();
        Robot selected = _yellowTeam.value(ids[i]);

        // Its own position:
        auto rob_msg = ctr_msgs::msg::Robot();
        rob_msg.pos.x = selected.position().x();
        rob_msg.pos.y = selected.position().y();
        rob_msg.pos.z = 0.0;
        rob_msg.pos.confidence = selected.position().confidence();
        rob_msg.id = selected.id();
        rob_msg.orientation = Utils::wrapToTwoPi(selected.orientation());
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
                    visible_robots.append(rob);
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
                        visible_robots.append(rob);
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
                //Check for obstruction:
                if(checkVisibility(selected.position(), visible_robots, _ball, selected.radius())) {
                    auto ball_msg = ctr_msgs::msg::Position();
                    ball_msg.x = _ball.x();
                    ball_msg.y = _ball.y();
                    ball_msg.z = 0.0;
                    ball_msg.confidence = _ball.confidence();
                    message.balls.push_back(ball_msg);
                }
            }
        }

        // Check visible robots:
        for(int j = 0; j < visible_robots.size(); j++) {
            Robot target = visible_robots.takeAt(j);
            if(checkVisibility(selected.position(), visible_robots, target.position(), 2*selected.radius())) {
                auto rob_msg = ctr_msgs::msg::Robot();
                rob_msg.pos.x = target.position().x();
                rob_msg.pos.y = target.position().y();
                rob_msg.pos.z = 0.0;
                rob_msg.pos.confidence = target.position().confidence();
                rob_msg.id = target.id();
                rob_msg.team = target.team() == Colors::YELLOW ? "yellow" : "blue";
                message.robots.push_back(rob_msg);
            }
            visible_robots.insert(j, target);
        }

        // Publish the desired message:
        clock_gettime(CLOCK_REALTIME, &_stop);
        message.timestamp = ((_stop.tv_sec*1E9+_stop.tv_nsec)-(_start.tv_sec*1E9+_start.tv_nsec))/1E9;
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
    float confidence = 0.0;
    //RCLCPP_INFO(this->get_logger(), "Number of balls: %d", balls.size());
    for(it=balls.constBegin(); it!=balls.constEnd(); it++) {
        const SSL_DetectionBall ball = it->second;

        // Confidence
        if(ball.has_confidence()==false) {
          numBalls -= 1;
          continue;
        } else {
          confidence += ball.confidence();
        }

        realX += ball.x()*MM2METER;
        realY += ball.y()*MM2METER;
    }

    Position ballAux(0.0, 0.0, 0.0, true);
    if(numBalls > 0) {
      ballAux.setX(realX/numBalls);
      ballAux.setY(realY/numBalls);
      ballAux.setZ(0.0);
      ballAux.setBool(false);
      ballAux.updateConfidence(confidence/numBalls);
    }
    _ball = ballAux;
}

void VisionNode::processRobots(const QHash<int, std::pair<int,SSL_DetectionRobot> > &robots) {
    QList<std::pair<int, SSL_DetectionRobot>> yellowTeam = robots.values(Colors::YELLOW);
    QList<std::pair<int, SSL_DetectionRobot>> blueTeam = robots.values(Colors::BLUE);

    _yellowTeam = processTeam(yellowTeam, Colors::YELLOW);
    _blueTeam = processTeam(blueTeam, Colors::BLUE);
}

QHash<qint8, Robot> VisionNode::processTeam(QList<std::pair<int, SSL_DetectionRobot>> team, int color) {
    float realX = 0;
    float realY = 0;
    float confidence = 0.0;
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
        confidence = 0;
        for(it=team.begin(); it!=team.end(); it++) {
            const SSL_DetectionRobot robot = it->second;

            // Discard robots with wrong id
            if(robot.robot_id() != i || robot.has_confidence() == false) {  
              continue;
            }

            if(robot.has_x() && robot.has_y()) {
                realX += robot.x()*MM2METER;
                realY += robot.y()*MM2METER;
                confidence += robot.confidence();
                numFrames++;
                if(robot.has_orientation()) {
                    realOri += robot.orientation();
                    numOri++;
                }
            }
        }

        Position pos(realX/numFrames, realY/numFrames, 0.0, false, confidence/numFrames);
        Robot aux(pos, color, i);
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

bool VisionNode::checkVisibility(Position reference, QList<Robot> robots, Position target, float minDistance) {
    for(int i = 0; i < robots.size(); i++) {
        // Check if the robot can be between the reference and the target:
        if(Utils::distance(target, reference) >= Utils::distance(robots[i].position(), reference)) {
            // Calculate a line from the reference to the target and check if the distance
            // from the robot to the referred line is less than minDist:
            if(Utils::distanceToLine(reference, target, robots[i].position()) < minDistance) {
                return false;
            }
        }
    }

    return true;
}
