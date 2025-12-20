#include "VideoStreamer.hpp"
#include <opencv2/opencv.hpp>

VideoStreamer::VideoStreamer()
    : receiver_("udp://0.0.0.0:11111"),
      publisher_("tcp://*:5555"),
      running_(true)
{
}

void VideoStreamer::run()
{
    if (!receiver_.isOpened())
        return;

    cv::Mat frame;

    while (running_) {
        if (!receiver_.grabFrame(frame))
            continue;

        publisher_.sendFrame(frame);

        cv::imshow("C++ Video", frame);
        if (cv::waitKey(1) == 27)
            running_ = false;
    }
}
