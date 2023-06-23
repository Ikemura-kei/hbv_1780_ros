#pragma once

namespace HBV1780
{
    static const int HBV_1780_SET_WIDTH = 1280;                        // combined image width, since the stereo pair is concatenated horizontally, each image is of width by half.
    static const int HBV_1780_SET_HEIGHT = 480;                        // image height.
    static const int HBV_1780_SET_WIDTH_HALF = HBV_1780_SET_WIDTH / 2; // the width of individual image.
}