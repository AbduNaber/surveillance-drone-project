#include "VideoReceiver.hpp"
#include <iostream>

VideoReceiver::VideoReceiver(const std::string& uri)
{

    // Set a 5-second timeout (5,000,000 microseconds)
    cap_.open(uri + "?timeout=5000000", cv::CAP_FFMPEG);
    
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
    std::cout << "[VideoReceiver] Grabbed a frame\n";
    bool res = !outFrame.empty();
    if (res == false) {
        std::cerr << "[VideoReceiver] Failed to grab frame\n";
       
    }

    return res;
}
