# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2

# Executable name
TARGET = grid_map

# Source files
SRCS = main.cpp gridMap.cpp gridMapContext.cpp

# Object directory and files
OBJDIR = obj
OBJS = $(patsubst %.cpp,$(OBJDIR)/%.o,$(SRCS))

# Default target
all: $(TARGET)

# Link step
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile step for each .cpp into $(OBJDIR)
$(OBJDIR)/%.o: %.cpp common.hpp
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -rf $(OBJDIR) $(TARGET)

# Run executable
run: $(TARGET)
	./$(TARGET)
