#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class MonocularCompressedSlamNode : public rclcpp::Node
{
public:
    MonocularCompressedSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularCompressedSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::CompressedImage;
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    void GrabImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;
};

#endif
