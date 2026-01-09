# ============================
# Compiler & flags
# ============================
CXX       := g++
CXXFLAGS  := -std=c++17 -O2 -Wall -Wextra

# ============================
# Paths
# ============================
INCLUDE_DIR := include
SRC_DIR     := src
BUILD_DIR   := build

# ============================
# Target
# ============================
TARGET := video_streamer

# ============================
# Source files
# ============================
SRCS := \
	$(SRC_DIR)/main.cpp \
	$(SRC_DIR)/VideoReceiver.cpp \
	$(SRC_DIR)/FramePublisher.cpp \
	$(SRC_DIR)/VideoStreamer.cpp \
	$(SRC_DIR)/locationTracker.cpp

OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)

# ============================
# Libraries
# ============================
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4)
OPENCV_LIBS   := $(shell pkg-config --libs opencv4)
ZMQ_LIBS      := -lzmq

# ============================
# Rules
# ============================
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(OPENCV_LIBS) $(ZMQ_LIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -I$(INCLUDE_DIR) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

.PHONY: all clean
