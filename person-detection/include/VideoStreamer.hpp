#pragma once
#include "VideoReceiver.hpp"
#include "FramePublisher.hpp"

class VideoStreamer {
public:
    VideoStreamer();
    void run();

private:
    VideoReceiver receiver_;
    FramePublisher publisher_;
    bool running_;
};
