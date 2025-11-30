# Makefile for svg_pathfinder - VSCode GDB friendly

CXX := g++
SRC := svg_pathfinder.cpp
SRC2 := test_svgpp.cpp
TARGET := svg_pathfinder



# Default flags
CXXFLAGS := -std=c++17 -Wall -Wextra
LDFLAGS := -ltinyxml2

.PHONY: all build debug run run-debug clean v2

# Default target
all: v2

v2: $(SRC2)
	$(CXX) $(CXXFLAGS) $(SRC2) -o $(TARGET) 

# Release build
build: $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

# Debug build suitable for GDB in VSCode
debug: CXXFLAGS += -g -O0 
debug: $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)_debug $(LDFLAGS)

# Run release binary
run:  v2#build
	./$(TARGET)

# Run debug binary with SVG_DEBUG
run-debug: debug
	SVG_DEBUG=1 ./$(TARGET)_debug

# Clean all binaries and generated SVG
clean:
	rm -f $(TARGET) $(TARGET)_debug map_with_path.svg
