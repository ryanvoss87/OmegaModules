# VCV Rack 2 Plugin Makefile
# Set RACK_DIR to the location of the Rack SDK (default: two levels up)
RACK_DIR ?= ../..

FLAGS += -O3 -ffast-math -fno-math-errno -fno-trapping-math
FLAGS += -DNDEBUG
FLAGS += -Wall -Wextra

LDFLAGS +=

# Collect all .cpp source files
SOURCES += $(wildcard src/*.cpp)

# Files to include in the distributable ZIP
DISTRIBUTABLES += res
DISTRIBUTABLES += $(wildcard LICENSE*)
DISTRIBUTABLES += $(wildcard presets)

# Include the VCV Rack plugin Makefile framework
include $(RACK_DIR)/plugin.mk
