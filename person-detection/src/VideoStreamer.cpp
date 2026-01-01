#include "VideoStreamer.hpp"
#include <opencv2/opencv.hpp>

VideoStreamer::VideoStreamer()
    : receiver_("udp://@0.0.0.0:11111"),
      publisher_("tcp://*:5555"),
      running_(true)
{
}

void VideoStreamer::run() {
    if (!receiver_.isOpened()) {
        std::cerr << "[Streamer] VideoReceiver failed to open. Exiting.\n";
        return;
    }

    cv::Mat frame;
    int continuous_errors = 0;

    while (running_) {
        std::cout << "[Streamer] Grabbing frame..." << std::endl;
        if (!receiver_.grabFrame(frame)) {
            continuous_errors++;
            // Don't log every single frame error, just wait for a valid I-frame
            if (continuous_errors % 30 == 0) {
                std::cerr << "[Streamer] Waiting for valid keyframe..." << std::endl;
            }
            std::cout << "[Streamer] Frame grab error count: " << continuous_errors << std::endl;
            continue; 
        }

        continuous_errors = 0; // Reset once we get a good frame
        publisher_.sendFrame(frame);

        cv::imshow("C++ Video", frame);
        if (cv::waitKey(1) == 27) running_ = false;
    }
}