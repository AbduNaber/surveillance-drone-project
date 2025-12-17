# Compiler and flags
CXX = g++
# -Iinclude: Derleyicinin header dosyalarını 'include' klasöründe aramasını sağlar
# -O3: Yüksek optimizasyon (büyük haritalar için gerekli)
CXXFLAGS = -std=c++17 -Wall -O3 -Iinclude 
LDFLAGS = -lyaml-cpp

# Executable name
TARGET = grid_map

# Directories
SRCDIR = src
INCDIR = include
OBJDIR = obj

# Source files (src klasörü altındaki dosyalar)
SRCS = $(SRCDIR)/main.cpp \
       $(SRCDIR)/gridMap.cpp \
       $(SRCDIR)/gridMapContext.cpp \
       $(SRCDIR)/pathFinder.cpp

# Header files (include klasörü altındaki dosyalar)
# Bu dosyalar değiştiğinde proje yeniden derlenir
HEADERS = $(INCDIR)/common.hpp \
          $(INCDIR)/gridMap.hpp \
          $(INCDIR)/gridMapContext.hpp \
          $(INCDIR)/pathFinder.hpp

# Object files
# src/dosya.cpp -> obj/dosya.o dönüşümü yapar
OBJS = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SRCS))

# Default target
all: $(TARGET)

# Link step
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS)  -o $@ $^ $(LDFLAGS)

# Compile step
# obj/dosya.o oluşturmak için src/dosya.cpp kullanılır
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp $(HEADERS)
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -rf $(OBJDIR) $(TARGET) grid_output.txt

# Run executable
run: $(TARGET)
	./$(TARGET)