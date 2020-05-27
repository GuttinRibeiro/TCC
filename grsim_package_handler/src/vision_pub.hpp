#ifndef VISION_PUB
#define VISION_PUB

#include <QHash>
#include "robocup_ssl_client.h"
#include "rclcpp/rclcpp.hpp"
// TODO: include custom message
#include "std_msgs/msg/string.hpp"
#include "position.hpp"

#define MM2METER (1/1000.0)

class VisionNode : public rclcpp::Node {
    private:
        RoboCupSSLClient *_client;
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
        QHash<int, SSL_DetectionFrame> _detectionPackets;
        SSL_GeometryData _geometryPacket;
        Position _ball;

        // Internal functions
        void client_callback();
        void update();
        QList<std::pair<int,SSL_DetectionBall> > parseCamerasBalls(const QList<SSL_DetectionFrame> &detectionFrames) const;
        QHash<int,std::pair<int,SSL_DetectionRobot> > parseCamerasRobots(const QList<SSL_DetectionFrame> &detectionFrames) const;
        void processBalls(const QList<std::pair<int,SSL_DetectionBall> > &balls);
        
    public:
        VisionNode(RoboCupSSLClient *client);

};

#endif