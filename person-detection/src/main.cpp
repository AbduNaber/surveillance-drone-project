#include "VideoStreamer.hpp"
#include "locationTracker.cpp"


// -------- Signal handling (Ctrl+C) --------
static LocationTracker* g_tracker = nullptr;


void signalHandler(int) {
    if (g_tracker) {
        g_tracker->stop();
    }
    std::exit(0);
}


int main()
{
    VideoStreamer app;
    //LocationTracker tracker("tcp://localhost:6000");
    //g_tracker = &tracker;


    //std::thread streamer_thread(&VideoStreamer::run, &app);
    app.run();
    //tracker.run();
    return 0;
}
