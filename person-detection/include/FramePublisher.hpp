#include <opencv2/opencv.hpp>
#include <string>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

class FramePublisher {
public:
    FramePublisher(const std::string& destination_ip, int port);
    ~FramePublisher();
    void sendFrame(const cv::Mat& frame);

private:
    int sockfd_;
    struct sockaddr_in dest_addr_;
};
