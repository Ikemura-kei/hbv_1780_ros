/**
 * @file HBV1780Node.cpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief This file defines a node to read a camera and publish (if configured so) raw and rectified images.
 * @version 0.1
 * @date 2023-06-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <ros/ros.h>

#include <HBV1780Camera.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <chrono>

// -- configuration variables --
#define SHOW_IMG false // whether to enable cv::imshow() for direct visualization, not recommended since we publish the images, use rviz instead.
#define PUB_MSG true // whether to publish images to topics.
#define PERFORMANCE_EVAL false // whether to log the execution times of various sub-processes (like image undistort).

void cvImg2SensorImageMsg(cv::Mat &cvImg, sensor_msgs::Image &sensorImageMsg, int counter)
{
    // -- prepare header --
    std_msgs::Header header;
    header.seq = counter;
    header.stamp = ros::Time::now();

    // -- prepare bridge variable --
    cv_bridge::CvImage bridgeVar(header, sensor_msgs::image_encodings::BGR8, cvImg);

    // -- convert to msg --
    bridgeVar.toImageMsg(sensorImageMsg);
}

#if PERFORMANCE_EVAL
class ExeEvaluator
{
public:
    ExeEvaluator(std::string procedureName) : procedureName(procedureName) {}

    void tic()
    {
        start = std::chrono::high_resolution_clock::now();
    }

    void tac()
    {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        ROS_INFO_STREAM("[hbv1780]: Execution duration for " << procedureName << " took: " << duration.count() / 1000.0f << " ms.");
    }

private:
    std::string procedureName;
    std::chrono::_V2::system_clock::time_point start;
};

static ExeEvaluator entireLoopEvaluator("<entire loop>");
static ExeEvaluator undistorEvaluator("<undistort>");
static ExeEvaluator getFrameEvaluator("<get frame>");
static ExeEvaluator publishEvaluator("<image publish>");
#endif

/*
 * Node parameters:
 *      - /<node_name>/left_camera_config_file (string): path to the calibration parameters of the left camera
 *      - /<node_name>/right_camera_config_file (string): path to the calibration parameters of the right camera
*/

int main(int ac, char **av)
{
    // -- initialize node and retrieve parameters --
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

    // -- read calibration parameters --
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

    // -- setup publishers --
    ros::Publisher leftImgPub = nh.advertise<sensor_msgs::Image>("left/image_raw", 10);
    ros::Publisher rightImgPub = nh.advertise<sensor_msgs::Image>("right/image_raw", 10);

    ros::Publisher leftImgRectPub = nh.advertise<sensor_msgs::Image>("left/image_rect", 10);
    ros::Publisher rightImgRectPub = nh.advertise<sensor_msgs::Image>("right/image_rect", 10);

    // -- static variables --
    static HBV1780::HBV1780Camera cameraHandle; // initialization is done within the constructor, so be careful not to make this a variable defined outside of main
    static cv::Mat left;
    static cv::Mat right;
    static cv::Mat whole;
    static int counter = 0;

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

    ros::Rate rate(1000);
    while (ros::ok())
    {
#if PERFORMANCE_EVAL
        entireLoopEvaluator.tic();
#endif
        rate.sleep();

#if PERFORMANCE_EVAL
        getFrameEvaluator.tic();
#endif
        bool ret = cameraHandle.getFrame(left, right, whole);
#if PERFORMANCE_EVAL
        getFrameEvaluator.tac();
#endif

        if (!ret)
        {
            ROS_WARN_STREAM("Image retrieval failed!");
            continue;
        }

        cv::Mat undistortedLeft, undistortedRight;

#if PERFORMANCE_EVAL
        undistorEvaluator.tic();
#endif
        cv::undistort(left, undistortedLeft, leftCamMat, leftDistCoeff);
        cv::undistort(right, undistortedRight, rightCamMat, rightDistCoeff);
#if PERFORMANCE_EVAL
        undistorEvaluator.tac();
#endif

#if SHOW_IMG
        cv::imshow("left", undistortedLeft);
        cv::imshow("right", undistortedRight);
        char k = cv::waitKey(1);
        if (k == 'q')
            break;
#endif

#if PUB_MSG
        sensor_msgs::Image leftRawImgMsg, rightRawImgMsg, leftRectImgMsg, rightRectImgMsg;

#if PERFORMANCE_EVAL
        publishEvaluator.tic();
#endif
        // -- convert to sensor_msgs::Image message --
        cvImg2SensorImageMsg(undistortedLeft, leftRectImgMsg, counter);
        cvImg2SensorImageMsg(undistortedRight, rightRectImgMsg, counter);
        cvImg2SensorImageMsg(left, leftRawImgMsg, counter);
        cvImg2SensorImageMsg(right, rightRawImgMsg, counter);

        // -- publish the messages --
        leftImgPub.publish(leftRawImgMsg);
        rightImgPub.publish(rightRawImgMsg);
        leftImgRectPub.publish(leftRectImgMsg);
        rightImgRectPub.publish(rightRectImgMsg);
#if PERFORMANCE_EVAL
        publishEvaluator.tac();
#endif

#endif

#if PERFORMANCE_EVAL
        entireLoopEvaluator.tac();
        std::cout << std::endl;
#endif
    }
}