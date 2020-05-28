#ifndef VISION_PUB
#define VISION_PUB

#include <QHash>
#include "robocup_ssl_client.h"
#include "rclcpp/rclcpp.hpp"
// TODO: include custom message
#include "grsim_package_handler/msg/ball.hpp"
#include "grsim_package_handler/msg/robot.hpp"
#include "grsim_package_handler/msg/visionpkg.hpp"
#include "position.hpp"
#include "robot.hpp"

#define MM2METER (1/1000.0)

class VisionNode : public rclcpp::Node {
    private:
        RoboCupSSLClient *_client;
        rclcpp::TimerBase::SharedPtr _timer;
        QHash<int, rclcpp::Publisher<grsim_package_handler::msg::Visionpkg>::SharedPtr> _publisherYellow;
        QHash<int, rclcpp::Publisher<grsim_package_handler::msg::Visionpkg>::SharedPtr> _publisherBlue;
        QHash<int, SSL_DetectionFrame> _detectionPackets;
        SSL_GeometryData _geometryPacket;
        Position _ball;
        QHash<qint8, Robot> _yellowTeam;
        QHash<qint8, Robot> _blueTeam;

        // Internal functions
        void client_callback();
        void update();
        void split_packages();
        QList<std::pair<int,SSL_DetectionBall> > parseCamerasBalls(const QList<SSL_DetectionFrame> &detectionFrames) const;
        QHash<int,std::pair<int,SSL_DetectionRobot> > parseCamerasRobots(const QList<SSL_DetectionFrame> &detectionFrames) const;
        void processBalls(const QList<std::pair<int,SSL_DetectionBall> > &balls);
        void processRobots(const QHash<int, std::pair<int,SSL_DetectionRobot> > &robots);
        QHash<qint8, Robot> processTeam(QList<std::pair<int, SSL_DetectionRobot>> team);

    public:
        VisionNode(RoboCupSSLClient *client);

};

#endif