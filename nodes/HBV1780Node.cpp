#include <ros/ros.h>

#include <HBV1780Camera.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>

#define SHOW_IMG true
#define PUB_MSG true

static int counter = 0;

int main(int ac, char **av)
{
    ros::init(ac, av, "hbv_1780_node");
    ros::Time::init();
    ros::NodeHandle nh("~");

    std::string left_camera_config_file = "";
    if (!nh.getParam("left_camera_config_file", left_camera_config_file))
    {
        ROS_FATAL_STREAM("Config file to the left camera not given!");
    }
    std::string right_camera_config_file = "";
    if (!nh.getParam("right_camera_config_file", right_camera_config_file))
    {
        ROS_FATAL_STREAM("Config file to the right camera not given!");
    }

    left_camera_config_file = std::string(realpath(left_camera_config_file.c_str(), NULL));
    right_camera_config_file = std::string(realpath(right_camera_config_file.c_str(), NULL));

    cv::FileStorage fsLeft(left_camera_config_file, cv::FileStorage::READ);
    cv::Mat leftCamMat;
    fsLeft["camera_matrix"] >> leftCamMat;
    cv::Mat leftDistCoeff;
    fsLeft["distortion_coefficients"] >> leftDistCoeff;

    cv::FileStorage fsRight(left_camera_config_file, cv::FileStorage::READ);
    cv::Mat rightCamMat;
    fsRight["camera_matrix"] >> rightCamMat;
    cv::Mat rightDistCoeff;
    fsRight["distortion_coefficients"] >> rightDistCoeff;

    ros::Publisher leftImgPub = nh.advertise<sensor_msgs::Image>("left/image_raw", 10);
    ros::Publisher rightImgPub = nh.advertise<sensor_msgs::Image>("right/image_raw", 10);

    ros::Publisher leftImgRectPub = nh.advertise<sensor_msgs::Image>("left/image_rect", 10);
    ros::Publisher rightImgRectPub = nh.advertise<sensor_msgs::Image>("right/image_rect", 10);

    static HBV1780::HBV1780Camera cameraHandle;
    static cv::Mat left;
    static cv::Mat right;
    static cv::Mat whole;

    if (!cameraHandle.initialized())
    {
        ROS_FATAL_STREAM("Camera not successfully initialized!");
        return 1;
    }

#if SHOW_IMG
    cv::namedWindow("left");
    cv::namedWindow("right");
    // cv::namedWindow("whole");
#endif

    ros::Time lastTime = ros::Time::now();
    ros::Rate rate(1000);
    while (ros::ok())
    {
        // rate.sleep();
        cameraHandle.getFrame(left, right, whole);

        cv::Mat undistortedLeft;
        cv::undistort(left, undistortedLeft, leftCamMat, leftDistCoeff);

        cv::Mat undistortedRight;
        cv::undistort(right, undistortedRight, rightCamMat, rightDistCoeff);

        float dt = (ros::Time::now() - lastTime).toSec();
        float fps = 1 / dt;
        ROS_INFO_STREAM("fps: " << std::to_string(fps));
        lastTime = ros::Time::now();

#if SHOW_IMG
        cv::imshow("left", undistortedLeft);
        cv::imshow("right", undistortedRight);
        // cv::imshow("whole", whole);
        char k = cv::waitKey(1);
        if (k == 'q')
            break;
#endif

#if PUB_MSG
        cv_bridge::CvImage leftCvImg, rightCvImg;
        sensor_msgs::Image leftImgMsg, rightImgMsg;

        // -- prepare common image header --
        std_msgs::Header header;         // empty header
        header.seq = counter;            // user defined counter
        header.stamp = ros::Time::now(); // time

        // -- convert to commonly used RGB8 --
        cv::cvtColor(left, left, cv::COLOR_BGR2RGB);
        cv::cvtColor(right, right, cv::COLOR_BGR2RGB);

        // -- create cv_bridge objects --
        leftCvImg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, left);
        rightCvImg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, right);

        // -- convert to sensor_msgs::Image message --
        leftCvImg.toImageMsg(leftImgMsg);   // from cv_bridge to sensor_msgs::Image
        rightCvImg.toImageMsg(rightImgMsg); // from cv_bridge to sensor_msgs::Image

        // -- publish the messages --
        leftImgPub.publish(leftImgMsg);
        rightImgPub.publish(rightImgMsg);
#endif
    }
}