#ifndef VISION_PUB
#define VISION_PUB

#include <QHash>
#include <ctime>
#include "robocup_ssl_client.h"
#include "rclcpp/rclcpp.hpp"
#include "vision/msg/ball.hpp"
#include "vision/msg/robot.hpp"
#include "vision/msg/visionpkg.hpp"
#include "position.hpp"
#include "robot.hpp"

#define MM2METER (1/1000.0)

class VisionNode : public rclcpp::Node {
    private:
        RoboCupSSLClient *_client;
        rclcpp::TimerBase::SharedPtr _timer;
        QHash<qint8, rclcpp::Publisher<vision::msg::Visionpkg>::SharedPtr> _publisherYellow;
        QHash<qint8, rclcpp::Publisher<vision::msg::Visionpkg>::SharedPtr> _publisherBlue;
        QHash<int, SSL_DetectionFrame> _detectionPackets;
        SSL_GeometryData _geometryPacket;
        Position _ball;
        QHash<qint8, Robot> _yellowTeam;
        QHash<qint8, Robot> _blueTeam;
        timespec _start, _stop;
        timespec _tstart, _tstop;

        // Internal functions
        void client_callback();
        void update();
        void split_packages();
        QList<std::pair<int,SSL_DetectionBall> > parseCamerasBalls(const QList<SSL_DetectionFrame> &detectionFrames) const;
        QHash<int,std::pair<int,SSL_DetectionRobot> > parseCamerasRobots(const QList<SSL_DetectionFrame> &detectionFrames) const;
        void processBalls(const QList<std::pair<int,SSL_DetectionBall> > &balls);
        void processRobots(const QHash<int, std::pair<int,SSL_DetectionRobot> > &robots);
        QHash<qint8, Robot> processTeam(QList<std::pair<int, SSL_DetectionRobot>> team, int color);
        bool checkVisibility(Position reference, QList<Robot> robots, Position target, float minDistance);

    public:
        VisionNode(RoboCupSSLClient *client);

};

#endif