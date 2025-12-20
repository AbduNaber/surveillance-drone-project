#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class VideoReceiver {
public:
    explicit VideoReceiver(const std::string& uri);
    bool isOpened() const;
    bool grabFrame(cv::Mat& outFrame);

private:
    cv::VideoCapture cap_;
};
