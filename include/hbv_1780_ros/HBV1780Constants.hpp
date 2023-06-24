/**
 * @file HBV1780Constants.hpp
 * @author IKEMURA, Kei (ikemurakei2001@gmail.com)
 * @brief This file contains some constant definitions about the camera
 * @version 0.1
 * @date 2023-06-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

namespace HBV1780
{
    static const int HBV_1780_SET_WIDTH = 1280;                        // combined image width, since the stereo pair is concatenated horizontally, each image is of width by half.
    static const int HBV_1780_SET_HEIGHT = 480;                        // image height.
    static const int HBV_1780_SET_WIDTH_HALF = HBV_1780_SET_WIDTH / 2; // the width of individual image.
}