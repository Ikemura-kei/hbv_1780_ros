#include <ros/ros.h>

#include <HBV1780Camera.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#define SHOW_IMG true
#define PUB_MSG true

static int counter = 0;

int main(int ac, char **av)
{
    ros::init(ac, av, "hbv_1780_node");
    ros::Time::init();
    ros::NodeHandle nh("~");

    ros::Publisher leftImgPub = nh.advertise<sensor_msgs::Image>("left/image_raw", 10);
    ros::Publisher rightImgPub = nh.advertise<sensor_msgs::Image>("right/image_raw", 10);

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
    cv::namedWindow("whole");
#endif

    ros::Time lastTime = ros::Time::now();
    ros::Rate rate(1000);
    while (ros::ok())
    {
        // rate.sleep();
        cameraHandle.getFrame(left, right, whole);

        float dt = (ros::Time::now() - lastTime).toSec();
        float fps = 1 / dt;
        ROS_INFO_STREAM("fps: " << std::to_string(fps));
        lastTime = ros::Time::now();

#if SHOW_IMG
        cv::imshow("left", left);
        cv::imshow("right", right);
        cv::imshow("whole", whole);
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