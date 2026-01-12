#include "FramePublisher.hpp"
#include <iostream>

FramePublisher::FramePublisher(const std::string& endpoint)
    : context_(1), publisher_(context_, zmq::socket_type::pub)
{
    try {
        publisher_.bind(endpoint);
        std::cout << "[FramePublisher] Bound to " << endpoint << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "[FramePublisher] Failed to bind to " << endpoint << ": " << e.what() << std::endl;
    }
}

FramePublisher::~FramePublisher() {
    // ZMQ objects clean up themselves
}

void FramePublisher::sendFrame(const cv::Mat& frame)
{
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    
    // Encode to JPEG
    cv::imencode(".jpg", frame, buffer, params);

    try {
        zmq::message_t message(buffer.data(), buffer.size());
        publisher_.send(message, zmq::send_flags::none);
    } catch (const zmq::error_t& e) {
        std::cerr << "[FramePublisher] Send failed: " << e.what() << std::endl;
    }
}
