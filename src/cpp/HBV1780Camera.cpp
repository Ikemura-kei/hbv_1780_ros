/**
 * @file HBV1780Camera.cpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief This file defines the HBV1780Camera class for interacting with the camera.
 * @version 0.1
 * @date 2023-06-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <HBV1780Constants.hpp>
#include <HBV1780Camera.hpp>

#include <opencv2/opencv.hpp>
#include <regex>

#include <chrono>
using namespace std::chrono;

namespace HBV1780
{
    HBV1780Camera::HBV1780Camera(std::string device)
    {
        this->IMG_HALF_WIDTH = HBV_1780_SET_WIDTH_HALF;
        this->IMG_HEIGHT = HBV_1780_SET_HEIGHT;
        this->IMG_WIDTH = HBV_1780_SET_WIDTH;

        // -- use gstreamer backend, otherwise the frame rate is about 8Hz, probably because OpenCV's defualt implementation is too bad --
        std::string gstreamerStr = "v4l2src device=DEVICE_PLACEHOLDER ! image/jpeg, width=WIDTH_PLACEHOLDER, height=HEIGHT_PLACEHOLDER, framerate=30/1 ! jpegdec ! videoconvert ! appsink";
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("DEVICE_PLACEHOLDER"), device);
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("WIDTH_PLACEHOLDER"), std::to_string(this->IMG_WIDTH));
        gstreamerStr = std::regex_replace(gstreamerStr, std::regex("HEIGHT_PLACEHOLDER"), std::to_string(this->IMG_HEIGHT));
        // std::cout << gstreamerStr << std::endl;
        static cv::VideoCapture cap(gstreamerStr, cv::CAP_GSTREAMER);
        this->camPtr = &cap;
        this->imageSize = cv::Size(this->IMG_HALF_WIDTH, this->IMG_HEIGHT);

        // -- test read --
        cv::Mat testFrame;
        *(this->camPtr) >> testFrame;

        if (testFrame.empty())
        {
            // no valid frame read, initialization failed.
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

        *(this->camPtr) >> this->frame;

        if (this->frame.empty())
            return false;

        // -- the returned raw frame is a combined image of left and right, so we crop --
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