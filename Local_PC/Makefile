# ====== Unified Makefile ======
# Usage:
#   make -f Makefile.unified
#   make -f Makefile.unified PLATFORM=linux
#   make -f Makefile.unified PLATFORM=raspberry
#   make -f Makefile.unified PLATFORM=windows
#   make -f Makefile.unified gui
#
# PLATFORM is the target you want to build for. Host auto-detection is only
# used as the default, so cross-building can still be selected explicitly.

# ====== Host / Platform Detection ======
ifeq ($(OS),Windows_NT)
  DETECTED_PLATFORM := windows
else
  UNAME_M := $(shell uname -m 2>/dev/null)
  ifneq (,$(filter aarch64 arm64 armv7l armv6l,$(UNAME_M)))
    DETECTED_PLATFORM := raspberry
  else
    DETECTED_PLATFORM := linux
  endif
endif

PLATFORM ?= $(DETECTED_PLATFORM)
AXIS     ?= AXIS_5

# ====== Common Sources ======
LOCAL_SRC := source/tc_6axis.cpp \
             source/axis_controller.cpp \
             source/DriverConnection.cpp

RPI_SRC   := source/tc.cpp \
             source/DriverConnection.cpp

GUI_SRC   := source/GUI/tc_6axis_gui.cpp \
             source/GUI/main_window.cpp \
             source/GUI/gui_log_redirect.cpp \
             source/GUI/usb_camera.cpp \
             source/GUI/weld_detector.cpp \
             source/axis_controller.cpp \
             source/DriverConnection.cpp

# ====== Tool Defaults ======
CROSS_COMPILE ?= aarch64-linux-gnu-
PKG_CONFIG    ?= pkg-config

BASE_CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Isource -Isource/GUI
DEBUG_FLAGS   := -std=c++17 -g -O0 -Wall -Wextra -Isource -Isource/GUI

# ====== Platform Config ======
ifeq ($(PLATFORM),raspberry)
  ifeq ($(origin CXX),default)
    CXX = $(CROSS_COMPILE)g++
  endif
  TARGET   = test_rpi
  GUI_TARGET = gui_rpi
  SRC      = $(RPI_SRC)
  LIBDIR  ?= lib_rasp
  CXXFLAGS ?= $(BASE_CXXFLAGS) -I$(LIBDIR) -D$(AXIS)
  LDFLAGS  ?= -L$(LIBDIR) -Wl,-rpath,\$$ORIGIN/lib_rasp
  LDLIBS   ?= $(LIBDIR)/libEziMOTIONPlusR.so.1.0.3
else ifeq ($(PLATFORM),windows)
  CXX      ?= g++
  TARGET   = test.exe
  GUI_TARGET = gui.exe
  SRC      = $(LOCAL_SRC)
  LIBDIR  ?= lib
  CXXFLAGS ?= $(BASE_CXXFLAGS) -I$(LIBDIR) -D$(AXIS)
  LDFLAGS  ?= -L$(LIBDIR)
  LDLIBS   ?= -lEziMOTIONPlusR
else ifeq ($(PLATFORM),linux)
  CXX      ?= g++
  TARGET   = test
  GUI_TARGET = gui
  SRC      = $(LOCAL_SRC)
  LIBDIR  ?= lib
  CXXFLAGS ?= $(BASE_CXXFLAGS) -I$(LIBDIR) -D$(AXIS)
  LDFLAGS  ?= -L$(LIBDIR) -Wl,-rpath,\$$ORIGIN/lib
  LDLIBS   ?= -lEziMOTIONPlusR
else
  $(error Unknown PLATFORM '$(PLATFORM)'. Use PLATFORM=linux, raspberry, or windows)
endif

QT_CXXFLAGS     := $(shell $(PKG_CONFIG) --cflags Qt5Widgets 2>/dev/null)
QT_LDLIBS       := $(shell $(PKG_CONFIG) --libs Qt5Widgets 2>/dev/null)
OPENCV_CXXFLAGS := $(shell $(PKG_CONFIG) --cflags opencv4 2>/dev/null || $(PKG_CONFIG) --cflags opencv 2>/dev/null)
OPENCV_LDLIBS   := $(shell $(PKG_CONFIG) --libs opencv4 2>/dev/null || $(PKG_CONFIG) --libs opencv 2>/dev/null)

OBJDIR     := build/unified/$(PLATFORM)/$(AXIS)
GUI_OBJDIR := build/unified/$(PLATFORM)/$(AXIS)_gui

OBJ     := $(SRC:%.cpp=$(OBJDIR)/%.o)
GUI_OBJ := $(GUI_SRC:%.cpp=$(GUI_OBJDIR)/%.o)

# ====== Rules ======
.PHONY: all clean run debug gui print-config

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(GUI_OBJDIR):
	mkdir -p $(GUI_OBJDIR)

$(OBJDIR)/%.o: %.cpp | $(OBJDIR)
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

gui: $(GUI_TARGET)

$(GUI_TARGET): $(GUI_OBJ)
	$(CXX) -o $@ $^ $(LDFLAGS) -no-pie $(LDLIBS) $(QT_LDLIBS) $(OPENCV_LDLIBS)

$(GUI_OBJDIR)/%.o: %.cpp | $(GUI_OBJDIR)
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(QT_CXXFLAGS) $(OPENCV_CXXFLAGS) -fPIC -c $< -o $@

debug: CXXFLAGS := $(DEBUG_FLAGS) -I$(LIBDIR) -D$(AXIS)
debug: clean all

print-config:
	@printf '%s\n' 'DETECTED_PLATFORM=$(DETECTED_PLATFORM)'
	@printf '%s\n' 'PLATFORM=$(PLATFORM)'
	@printf '%s\n' 'AXIS=$(AXIS)'
	@printf '%s\n' 'CXX=$(CXX)'
	@printf '%s\n' 'TARGET=$(TARGET)'
	@printf '%s\n' 'GUI_TARGET=$(GUI_TARGET)'
	@printf '%s\n' 'LIBDIR=$(LIBDIR)'
	@printf '%s\n' 'CXXFLAGS=$(CXXFLAGS)'
	@printf '%s\n' 'LDFLAGS=$(LDFLAGS)'
	@printf '%s\n' 'LDLIBS=$(LDLIBS)'

clean:
	$(RM) -r build/unified/$(PLATFORM)
	$(RM) $(TARGET) $(GUI_TARGET)
