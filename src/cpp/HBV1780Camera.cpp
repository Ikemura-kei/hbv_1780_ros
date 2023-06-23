#include <HBV1780Constants.hpp>
#include <HBV1780Camera.hpp>

#include <opencv2/opencv.hpp>

#include <chrono>
using namespace std::chrono;

namespace HBV1780
{
    HBV1780Camera::HBV1780Camera()
    {
        this->IMG_HALF_WIDTH = HBV_1780_SET_WIDTH_HALF;
        this->IMG_HEIGHT = HBV_1780_SET_HEIGHT;
        this->IMG_WIDTH = HBV_1780_SET_WIDTH;

        static cv::VideoCapture cap(2);
        this->camPtr = &cap;
        this->camPtr->set(cv::CAP_PROP_FRAME_WIDTH, this->IMG_WIDTH);
        this->camPtr->set(cv::CAP_PROP_FRAME_HEIGHT, this->IMG_HEIGHT);
        this->imageSize = cv::Size(this->IMG_HALF_WIDTH, this->IMG_HEIGHT);

        // -- test read --
        cv::Mat testFrame;
        *(this->camPtr) >> testFrame;

        if (testFrame.empty())
        {
            // no valid frame read
            this->isInitialized = false;
            this->camPtr = nullptr;
            return;
        }

        this->isInitialized = true;
    }

    bool HBV1780Camera::getFrame(cv::Mat &left, cv::Mat &right, cv::Mat &whole)
    {
        if (!this->isInitialized)
            return false;

        auto start = high_resolution_clock::now();
        *(this->camPtr) >> this->frame;
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        std::cout << duration.count() / 1000.0 << " ms" << std::endl;

        if (this->frame.empty())
        {
            return false;
        }

        // std::cout << this->imageSize << std::endl;

        cv::Rect leftROI(cv::Point(0, 0), this->imageSize);
        cv::Rect rightROI(cv::Point(this->IMG_HALF_WIDTH, 0), this->imageSize);

        left = this->frame(leftROI);
        right = this->frame(rightROI);
        whole = this->frame;

        return true;
    }

    bool HBV1780Camera::initialized()
    {
        return this->isInitialized;
    }
}