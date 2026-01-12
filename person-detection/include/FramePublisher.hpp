#include <opencv2/opencv.hpp>
#include <string>
#include <zmq.hpp>

class FramePublisher {
public:
    FramePublisher(const std::string& endpoint);
    ~FramePublisher();
    void sendFrame(const cv::Mat& frame);

private:
    zmq::context_t context_;
    zmq::socket_t publisher_;
};
