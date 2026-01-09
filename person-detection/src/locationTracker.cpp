#include <zmq.hpp>
#include <string>
#include <iostream>
#include <atomic>
#include <csignal>
#include <thread>
#include <nlohmann/json.hpp> // For JSON parsing

using json = nlohmann::json;
class LocationTracker {
public:
    LocationTracker(const std::string& endpoint)
        : context_(1),
          socket_(context_, ZMQ_SUB),
          endpoint_(endpoint),
          running_(true)
    {
        socket_.connect(endpoint_);
        socket_.set(zmq::sockopt::subscribe, "");
        std::cout << "[LocationTracker] Connected to " << endpoint_ << std::endl;
    }

    ~LocationTracker() {
        stop();
    }


    void updateTrackerState(int track_id, double timestamp) {

        //TODO: update tracker for UI mapping

    }

    void run() {
        while (running_) {
            zmq::message_t message;
            auto result = socket_.recv(message, zmq::recv_flags::none);
            if (!result) {
                continue;
            }

            std::string data(
                static_cast<char*>(message.data()),
                message.size()
            );

            handleMessage(data);
        }
    }

    void stop() {
        if (running_) {
            running_ = false;
            socket_.close();
            context_.close();
            std::cout << "[LocationTracker] Shutdown complete" << std::endl;
        }
    }

private:
    void handleMessage(const std::string& msg) {
        std::cout << "[Notification] " << msg << std::endl;

        // [Notification] {"event": "new_person", "track_id": 27, "bbox": [727, 308, 969, 701], "timestamp": 1767623737.485749}
        // Here you could parse the JSON message and extract relevant information

        // Example parsing logic (requires JSON library like nlohmann/json):
        try {
            auto json = nlohmann::json::parse(msg);
            std::string event = json["event"];
            if (event == "new_person") {
                int track_id = json["track_id"];
                std::vector<int> bbox = json["bbox"];
                double timestamp = json["timestamp"];
                
                std::cout << "[LocationTracker] New person detected: ID=" << track_id
                          << ", BBox=" << bbox[0] << "," << bbox[1] << "," << bbox[2] << "," << bbox[3]
                          << ", Timestamp=" << timestamp << std::endl;
                updateTrackerState(track_id, timestamp);
                
            }
        } catch (const std::exception& e) {
            std::cerr << "[LocationTracker] Error parsing JSON: " << e.what() << std::endl;
        }
    }

private:
    zmq::context_t context_;
    zmq::socket_t socket_;
    std::string endpoint_;
    std::atomic<bool> running_;
    std::vector<int> tracked_ids_;
};



