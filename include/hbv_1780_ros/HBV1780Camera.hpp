#pragma once
#include <opencv2/opencv.hpp>

namespace HBV1780
{
    class HBV1780Camera
    {
    public:
        HBV1780Camera();

        bool getFrame(cv::Mat &left, cv::Mat &right, cv::Mat &whole);

        bool initialized();

    private:
        int IMG_HALF_WIDTH;
        int IMG_HEIGHT;
        int IMG_WIDTH;
        cv::Size imageSize;
        cv::VideoCapture *camPtr;
        bool isInitialized = false;
        cv::Mat frame;
    };
}