#include "FramePublisher.hpp"
#include <cstring>

FramePublisher::FramePublisher(const std::string& endpoint)
    : context_(1), socket_(context_, ZMQ_PUSH)
{
    socket_.bind(endpoint);
}

void FramePublisher::sendFrame(const cv::Mat& frame)
{
    const size_t size = frame.total() * frame.elemSize();
    zmq::message_t msg(size);
    std::memcpy(msg.data(), frame.data, size);
    socket_.send(msg, zmq::send_flags::none);
}
