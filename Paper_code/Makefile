TARGET      := panolocalization

BUILD_DIR   := ./build
SRC_DIR     := ./src
INC_DIR     := ./inc
OBJ_DIR     := $(BUILD_DIR)/obj
BIN_DIR     := $(BUILD_DIR)/bin

SRCS        := $(wildcard $(SRC_DIR)/detect/*.cpp)   \
               $(wildcard $(SRC_DIR)/warp/*.cpp) \
	 	 	   $(wildcard $(SRC_DIR)/localization/*.cpp) \
			   $(wildcard $(SRC_DIR)/log/*.cpp) \
               $(wildcard $(SRC_DIR)/*.cpp)

OBJS        := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

CXX         := g++
CXXFLAGS    := -std=c++11
INCLUDE     := -I$(INC_DIR) $(shell pkg-config --cflags opencv4)
LDFLAGS     := -L/usr/lib $(shell pkg-config --libs opencv4)

.PHONY: all build clean release install

all: release

$(BIN_DIR)/$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<

build:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(OBJ_DIR)

release: build $(BIN_DIR)/$(TARGET) install

clean:
	-@rm -rvf $(OBJ_DIR)/*
	-@rm -rvf $(BIN_DIR)/*
	-@rm -vf ./$(TARGET)

install:
	@mv $(BIN_DIR)/$(TARGET) .
