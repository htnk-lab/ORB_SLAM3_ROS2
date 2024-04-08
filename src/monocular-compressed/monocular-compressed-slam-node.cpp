#include "monocular-compressed-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularCompressedSlamNode::MonocularCompressedSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "image/compressed",
        10,
        std::bind(&MonocularCompressedSlamNode::GrabImage, this, std::placeholders::_1));
    m_pose_publisher = this->create_publisher<PoseStamped>("slam/pose", 10);
    std::cout << "slam changed" << std::endl;
}

MonocularCompressedSlamNode::~MonocularCompressedSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularCompressedSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    // m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(this->now()));
    auto Twc = m_SLAM->GetCamTwc();
    if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0)) // avoid publishing NaN
        return;
    if (Twc.angleX() == 0.0 && Twc.angleY() == 0.0 && Twc.angleZ() == 0.0 && Twc.translation()[0] == 0.0 && Twc.translation()[1] == 0.0 && Twc.translation()[2] == 0.0)
        return;
    std::cout << (Twc.angleX() == 0.0f);
    PoseStamped pose = PoseStamped();
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = "world";
    pose.pose = Utility::sophusToPoseMsg(Twc);
    m_pose_publisher->publish(pose);
}
