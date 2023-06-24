/**
 * @file HBV1780Camera.hpp
 * @author IKEMURA, Kei (ikemurakei2001@gmail.com)
 * @brief This file declares the HBV1780Camera class for interacting with the camera.
 * @version 0.1
 * @date 2023-06-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include <opencv2/opencv.hpp>

namespace HBV1780
{
    class HBV1780Camera
    {
    public:
        HBV1780Camera(std::string device);
/**
 * @brief Get the current frame.
 * 
 * @param left output argument, stores the left image.
 * @param right output argument, stores the right image.
 * @param whole output argument, stores the combined image, just in case people want, normally won't.
 * @return true when image retrieval succeeded.
 * @return false otherwise, like when image returned is empty or camera initialization was failed previously.
 */
        bool getFrame(cv::Mat &left, cv::Mat &right, cv::Mat &whole);
/**
 * @brief Checks if the camera was successfully initialized.
 * 
 * @return true if initialized successfully.
 * @return false otherwise.
 */
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