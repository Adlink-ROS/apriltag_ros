#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>

// apriltag
#include <apriltag.h>
#include "common/homography.h"
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
//#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
//#include <apriltag_msgs/msg/april_tag_detection.hpp>


class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

    ~AprilTagNode() override;
    

private:
    typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Mat3;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;
    std::string tag_family;
    double tag_edge_size;
    int max_hamming;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    bool remove_duplicates_ = true;
    bool z_up ;
    // function pointer for tag family creation / destruction
    static const std::map<std::string, apriltag_family_t *(*)(void)> tag_create;
    const static std::map<std::string, void (*)(apriltag_family_t*)> tag_destroy;

    image_transport::CameraSubscriber sub_cam;
    //const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, 
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    void getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const double size) const;
    void addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d >& objectPoints) const;
    void addImagePoints (apriltag_detection_t *detection, std::vector<cv::Point2d >& imagePoints) const;
    Eigen::Matrix4d getRelativeTransform(
        std::vector<cv::Point3d > objectPoints,
        std::vector<cv::Point2d > imagePoints,
        double fx, double fy, double cx, double cy) const;

    geometry_msgs::msg::TransformStamped makeTagPose(
        const Eigen::Matrix4d& transform,
        const Eigen::Quaternion<double> rot_quaternion,
        const std_msgs::msg::Header& header);

    static int idComparison (const void* first, const void* second);
    void removeDuplicates(zarray_t* detections_ );
    
};
