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
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <chrono>

// -- configuration variables --
#define SHOW_IMG false         // whether to enable cv::imshow() for direct visualization, not recommended since we publish the images, use rviz instead.
#define PUB_MSG true           // whether to publish images to topics.
#define PERFORMANCE_EVAL false // whether to log the execution times of various sub-processes (like image undistort).

std::string exec(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

void cvImg2SensorImageMsg(cv::Mat &cvImg, sensor_msgs::Image &sensorImageMsg, int counter, ros::Time time)
{
    // -- prepare header --
    std_msgs::Header header;
    header.seq = counter;
    header.stamp = time;

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
 *      - /<node_name>/right_camera_config_file (string): path to the calibration parameters of the right camera4
 *      - /<node_name>/device (string): camera device, such as /dev/video2, which is the default value
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

    std::string device = "/dev/video2";
    if (!nh.getParam("device", device))
    {
        ROS_WARN_STREAM("Video device not set, using default: " << device);
    }

    // -- get the device indexes that correspond to each bus id --
    std::string s = exec("v4l2-ctl --list-devices");
    std::string delimiter = "\n";
    size_t pos = 0;
    int counter_ = 0;
    std::string token;
    ROS_INFO_STREAM("--> Original device id: " << device);
    while ((pos = s.find(delimiter)) != std::string::npos)
    {
        token = s.substr(0, pos);
        // ROS_ERROR_STREAM(token);
        if (counter_ % 5 == 0)
        {
            // -- this is the line containing bus id --
            if (token.find(device) != std::string::npos)
            {
                size_t nextPos = s.find(delimiter, pos + 1);
                std::string id = s.substr(pos + 1, nextPos - pos - 1);
                std::string followed = s.substr(pos + 1, nextPos - pos - 2); // to cope with two-digit camera ids
                if (isdigit(followed[followed.size()-1]))
                    device = std::string("/dev/video") + followed[followed.size()-1] + id.back();
                else
                    device = std::string("/dev/video") + id.back();
                break;
            }
        }
        s.erase(0, pos + delimiter.length());

        counter_ += 1;
    }
    ROS_INFO_STREAM("--> New device id: " << device);

    bool pubWhole = false;
    if (!nh.param<bool>("pub_whole", pubWhole, false))
    {
        ROS_WARN_STREAM("Publish whole image argument not set, by default is " << std::boolalpha << pubWhole);
    }
    ROS_INFO_STREAM("Publish whole image argument is " << std::boolalpha << pubWhole);

    // -- read calibration parameters --
    left_camera_config_file = std::string(realpath(left_camera_config_file.c_str(), NULL));
    right_camera_config_file = std::string(realpath(right_camera_config_file.c_str(), NULL));
    ROS_INFO_STREAM("--> Left config: " << left_camera_config_file);
    ROS_INFO_STREAM("--> Right config: " << right_camera_config_file);
    cv::FileStorage fsLeft(left_camera_config_file, cv::FileStorage::READ);
    cv::Mat leftCamMat;
    fsLeft["camera_matrix"] >> leftCamMat;
    cv::Mat leftDistCoeff;
    fsLeft["distortion_coefficients"] >> leftDistCoeff;
    cv::Mat leftT;
    fsLeft["T"] >> leftT;
    cv::Mat leftR;
    fsLeft["R"] >> leftR;
    // std::cout << leftT << leftR << std::endl;

    cv::FileStorage fsRight(right_camera_config_file, cv::FileStorage::READ);
    cv::Mat rightCamMat;
    fsRight["camera_matrix"] >> rightCamMat;
    cv::Mat rightDistCoeff;
    fsRight["distortion_coefficients"] >> rightDistCoeff;
    cv::Mat rightT;
    fsRight["T"] >> rightT;
    cv::Mat rightR;
    fsRight["R"] >> rightR;
    // std::cout << rightT << rightR << std::endl;

    // -- initialize camera info msg --
    sensor_msgs::CameraInfo cameraInfo;
    cameraInfo.header.seq = 0;
    cameraInfo.header.frame_id = "";
    cameraInfo.distortion_model = "plumb_bob";
    cameraInfo.width = (int)fsLeft["image_width"];
    cameraInfo.height = (int)fsLeft["image_height"];
    cameraInfo.D.resize(5);
    for (int i = 0; i < 5; i++)
        cameraInfo.D[i] = leftDistCoeff.at<double>(i);
    for (int i = 0; i < 9; i++)
        cameraInfo.K[i] = leftCamMat.at<double>(i);
    for (int i = 0; i < 9; i++)
        cameraInfo.R[i] = 0;
    cameraInfo.R[0] = cameraInfo.R[4] = cameraInfo.R[8] = 1.0;

    // -- P is a 3X4 projection matrix --
    cameraInfo.P[0] = cameraInfo.K[0];
    cameraInfo.P[1] = cameraInfo.K[1];
    cameraInfo.P[2] = cameraInfo.K[2];
    cameraInfo.P[3] = 0;

    cameraInfo.P[4] = cameraInfo.K[3];
    cameraInfo.P[5] = cameraInfo.K[4];
    cameraInfo.P[6] = cameraInfo.K[5];
    cameraInfo.P[7] = 0;

    cameraInfo.P[8] = 0;
    cameraInfo.P[9] = 0;
    cameraInfo.P[10] = 1;
    cameraInfo.P[11] = 0;

    // -- initialize rectification maps --
    cv::Size imageSize = cv::Size(640, 480);
    cv::Mat R1, R2, P1, P2, Q, newCamMat;
    cv::stereoRectify(leftCamMat, leftDistCoeff, rightCamMat, rightDistCoeff, imageSize, rightR, rightT, R1, R2, P1, P2, Q);
    cv::Mat leftMap1, leftMap2;
    cv::initUndistortRectifyMap(leftCamMat, leftDistCoeff, leftR, newCamMat, imageSize, CV_32FC1, leftMap1, leftMap2);
    cv::Mat rightMap1, rightMap2;
    cv::initUndistortRectifyMap(rightCamMat, rightDistCoeff, rightR, newCamMat, imageSize, CV_32FC1, rightMap1, rightMap2);
    std::cout << Q << std::endl;

    // -- setup publishers --
    ros::Publisher leftImgPub = nh.advertise<sensor_msgs::Image>("left/image_raw", 10);
    ros::Publisher rightImgPub = nh.advertise<sensor_msgs::Image>("right/image_raw", 10);
    ros::Publisher wholeImgPub = nh.advertise<sensor_msgs::Image>("whole", 10);

    ros::Publisher leftImgRectPub = nh.advertise<sensor_msgs::Image>("left/image_rect", 10);
    ros::Publisher rightImgRectPub = nh.advertise<sensor_msgs::Image>("right/image_rect", 10);

    ros::Publisher cameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

    // -- static variables --
    static HBV1780::HBV1780Camera cameraHandle(device); // initialization is done within the constructor, so be careful not to make this a variable defined outside of main
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

        cv::Mat rectLeft, rectRight;

#if PERFORMANCE_EVAL
        undistorEvaluator.tic();
#endif
        cv::remap(left, rectLeft, leftMap1, leftMap2, cv::INTER_LINEAR);
        cv::remap(right, rectRight, rightMap1, rightMap2, cv::INTER_LINEAR);
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
        sensor_msgs::Image leftRawImgMsg, rightRawImgMsg, leftRectImgMsg, rightRectImgMsg, wholeMsg;

#if PERFORMANCE_EVAL
        publishEvaluator.tic();
#endif
        // -- convert to sensor_msgs::Image message --
        ros::Time now = ros::Time::now();
        cvImg2SensorImageMsg(rectLeft, leftRectImgMsg, counter, now);
        cvImg2SensorImageMsg(rectRight, rightRectImgMsg, counter, now);
        cvImg2SensorImageMsg(left, leftRawImgMsg, counter, now);
        cvImg2SensorImageMsg(right, rightRawImgMsg, counter, now);
        if (pubWhole)
            cvImg2SensorImageMsg(whole, wholeMsg, counter, now);

        // -- publish the messages --
        leftImgPub.publish(leftRawImgMsg);
        rightImgPub.publish(rightRawImgMsg);
        leftImgRectPub.publish(leftRectImgMsg);
        rightImgRectPub.publish(rightRectImgMsg);
        if (pubWhole)
            wholeImgPub.publish(wholeMsg);
        cameraInfo.header.stamp = ros::Time::now();
        cameraInfo.header.seq += 1;
        cameraInfoPub.publish(cameraInfo);

        counter++;
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