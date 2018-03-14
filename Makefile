CXX = arm-linux-gnueabihf-g++
#CXX = g++
CXXFLAGS = -g -H -Wall -Wextra -std=c++11
LDFLAGS = 


# name of executable to be produced
EXECUTABLE = Radiation-Control

# AOCL specific settings for OpenCL with cross-compiler
AOCL_COMPILE_CONFIG=$(shell aocl compile-config --arm)
AOCL_LINK_CONFIG=$(shell aocl link-config --arm)

SOURCE_DIR = src
INCLUDE_DIR = $(SOURCE_DIR)/inc
BUILD_DIR = build
BIN_DIR = $(BUILD_DIR)/bin

SOURCE_FILES = $(wildcard $(SOURCE_DIR)/*.cpp)
INCLUDE_FILES = $(wildcard $(INCLUDE_DIR)/*.h)
EXECUTABLE_FILES = $(EXECUTABLE:%=$(BIN_DIR)/%)
OBJECT_FILES = $(SOURCE_FILES:%.cpp=$(BUILD_DIR)/%.o)


build: $(EXECUTABLE_FILES)

clean:
	rm -r -f $(BUILD_DIR)

.PHONY: build clean

$(EXECUTABLE_FILES):	$(OBJECT_FILES)
	@echo Linking $<
	@mkdir $(BIN_DIR)
	@$(CXX) $(LDFLAGS) -o $@ $^ $(AOCL_LINK_CONFIG)
	@echo "Build successful!"

$(OBJECT_FILES):	$(BUILD_DIR)/%.o:	%.cpp
	@echo Compiling $<
	@mkdir -p $(@D)
	@$(CXX) -c $(CXXFLAGS) -I$(INCLUDE_DIR) -o $@ $< $(AOCL_COMPILE_CONFIG)
