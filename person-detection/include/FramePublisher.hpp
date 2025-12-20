#pragma once
#include <zmq.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class FramePublisher {
public:
    FramePublisher(const std::string& endpoint);
    void sendFrame(const cv::Mat& frame);

private:
    zmq::context_t context_;
    zmq::socket_t socket_;
};
