#include "FramePublisher.hpp"
#include <cstring>

FramePublisher::FramePublisher(const std::string& destination_ip, int port)
{
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return;
    }

    memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(port);
    if (inet_pton(AF_INET, destination_ip.c_str(), &dest_addr_.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
    }
}

FramePublisher::~FramePublisher() {
    if (sockfd_ >= 0) {
        close(sockfd_);
    }
}

void FramePublisher::sendFrame(const cv::Mat& frame)
{
    std::vector<uchar> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    
    // Encode to JPEG to reduce size
    cv::imencode(".jpg", frame, buffer, params);

    if (buffer.size() > 65507) {
        std::cerr << "[FramePublisher] Frame too large for UDP: " << buffer.size() << " bytes. Dropping.\n";
        return;
    }

    ssize_t sent_bytes = sendto(sockfd_, buffer.data(), buffer.size(), 0, 
                               (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
    
    if (sent_bytes < 0) {
        perror("[FramePublisher] sendto failed");
    } else {
        // Optional: Comment out to reduce spam
        // std::cout << "[FramePublisher] Sent JPEG frame: " << buffer.size() << " bytes\n";
    }
}
