#include "VideoReceiver.hpp"
#include <iostream>

VideoReceiver::VideoReceiver(const std::string& uri)
{
    cap_.open(uri, cv::CAP_FFMPEG);
    if (!cap_.isOpened()) {
        std::cerr << "[VideoReceiver] Failed to open stream: " << uri << "\n";
    }
}

bool VideoReceiver::isOpened() const
{
    return cap_.isOpened();
}

bool VideoReceiver::grabFrame(cv::Mat& outFrame)
{
    cap_ >> outFrame;
    return !outFrame.empty();
}
