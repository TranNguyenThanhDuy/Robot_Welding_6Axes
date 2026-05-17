# ====== Unified Makefile ======
# Usage:
#   make
#   make PF=linux AXIS=6
#   make PF=rasp AXIS=6
#   make PF=windows AXIS=6
#   make gui PF=rasp AXIS=6
#
# PF is the target platform. Host auto-detection is only used as the default,
# so cross-building can still be selected explicitly.

# ====== Host / Platform Detection ======
ifeq ($(OS),Windows_NT)
  DETECTED_PF := windows
else
  UNAME_M := $(shell uname -m 2>/dev/null)
  ifneq (,$(filter aarch64 arm64 armv7l armv6l,$(UNAME_M)))
    DETECTED_PF := rasp
  else
    DETECTED_PF := linux
  endif
endif

# Backward compatibility: PLATFORM=raspberry still works, but PF is preferred.
PF ?= $(or $(PLATFORM),$(DETECTED_PF))
AXIS ?= 5

ifneq (,$(filter $(PF),rasp raspberry rpi pi))
  PF_CANON := rasp
else ifneq (,$(filter $(PF),win windows))
  PF_CANON := windows
else ifeq ($(PF),linux)
  PF_CANON := linux
else
  $(error Unknown PF '$(PF)'. Use PF=linux, PF=rasp, or PF=windows)
endif

ifneq (,$(filter AXIS_%,$(AXIS)))
  AXIS_DEFINE := $(AXIS)
  AXIS_NUM := $(patsubst AXIS_%,%,$(AXIS))
else
  AXIS_DEFINE := AXIS_$(AXIS)
  AXIS_NUM := $(AXIS)
endif

# ====== Common Sources ======
LOCAL_SRC := source/tc_6axis.cpp \
             source/axis_controller.cpp \
             source/DriverConnection.cpp

RPI_SRC   := source/tc.cpp \
             source/DriverConnection.cpp

GUI_SRC   := source/GUI/tc_6axis_gui.cpp \
             source/GUI/app_config.cpp \
             source/GUI/main_window.cpp \
             source/GUI/gui_log_redirect.cpp \
             source/GUI/usb_camera.cpp \
             source/GUI/weld_detector.cpp \
             source/DriverConnection.cpp \
             source/17_5/axis_controller_trajectory.cpp \
             source/17_5/trajectory_file.cpp \
             source/17_5/trajectory_planner.cpp

# ====== Tool Defaults ======
CROSS_COMPILE ?= aarch64-linux-gnu-
PKG_CONFIG    ?= pkg-config

BASE_CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Isource -Isource/GUI
DEBUG_FLAGS   := -std=c++17 -g -O0 -Wall -Wextra -Isource -Isource/GUI

# ====== Platform Config ======
ifeq ($(PF_CANON),rasp)
  ifeq ($(origin CXX),default)
    ifneq (,$(filter aarch64 arm64 armv7l armv6l,$(UNAME_M)))
      CXX = g++
    else
      CXX = $(CROSS_COMPILE)g++
    endif
  endif
  TARGET   = test_rpi
  GUI_TARGET = gui_rpi
  SRC      = $(RPI_SRC)
  LIBDIR  ?= lib_rasp
  CXXFLAGS ?= $(BASE_CXXFLAGS) -I$(LIBDIR) -D$(AXIS_DEFINE)
  LDFLAGS  ?= -L$(LIBDIR) -Wl,-rpath,\$$ORIGIN/lib_rasp
  LDLIBS   ?= $(LIBDIR)/libEziMOTIONPlusR.so.1.0.3
else ifeq ($(PF_CANON),windows)
  CXX      ?= g++
  TARGET   = test.exe
  GUI_TARGET = gui.exe
  SRC      = $(LOCAL_SRC)
  LIBDIR  ?= lib
  CXXFLAGS ?= $(BASE_CXXFLAGS) -I$(LIBDIR) -D$(AXIS_DEFINE)
  LDFLAGS  ?= -L$(LIBDIR)
  LDLIBS   ?= -lEziMOTIONPlusR
else ifeq ($(PF_CANON),linux)
  CXX      ?= g++
  TARGET   = test
  GUI_TARGET = planner
  SRC      = $(LOCAL_SRC)
  LIBDIR  ?= lib
  CXXFLAGS ?= $(BASE_CXXFLAGS) -I$(LIBDIR) -D$(AXIS_DEFINE)
  LDFLAGS  ?= -L$(LIBDIR) -Wl,-rpath,\$$ORIGIN/lib
  LDLIBS   ?= -lEziMOTIONPlusR
else
  $(error Unknown platform '$(PF_CANON)'. Use PF=linux, PF=rasp, or PF=windows)
endif

QT_CXXFLAGS     := $(shell $(PKG_CONFIG) --cflags Qt5Widgets 2>/dev/null)
QT_LDLIBS       := $(shell $(PKG_CONFIG) --libs Qt5Widgets 2>/dev/null)
OPENCV_CXXFLAGS := $(shell $(PKG_CONFIG) --cflags opencv4 2>/dev/null || $(PKG_CONFIG) --cflags opencv 2>/dev/null)
OPENCV_LDLIBS   := $(shell $(PKG_CONFIG) --libs opencv4 2>/dev/null || $(PKG_CONFIG) --libs opencv 2>/dev/null)

OBJDIR     := build/unified/$(PF_CANON)/$(AXIS_DEFINE)
GUI_OBJDIR := build/unified/$(PF_CANON)/$(AXIS_DEFINE)_gui

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

debug: CXXFLAGS := $(DEBUG_FLAGS) -I$(LIBDIR) -D$(AXIS_DEFINE)
debug: clean all

print-config:
	@printf '%s\n' 'DETECTED_PF=$(DETECTED_PF)'
	@printf '%s\n' 'PF=$(PF)'
	@printf '%s\n' 'PF_CANON=$(PF_CANON)'
	@printf '%s\n' 'AXIS=$(AXIS)'
	@printf '%s\n' 'AXIS_NUM=$(AXIS_NUM)'
	@printf '%s\n' 'AXIS_DEFINE=$(AXIS_DEFINE)'
	@printf '%s\n' 'CXX=$(CXX)'
	@printf '%s\n' 'TARGET=$(TARGET)'
	@printf '%s\n' 'GUI_TARGET=$(GUI_TARGET)'
	@printf '%s\n' 'LIBDIR=$(LIBDIR)'
	@printf '%s\n' 'CXXFLAGS=$(CXXFLAGS)'
	@printf '%s\n' 'LDFLAGS=$(LDFLAGS)'
	@printf '%s\n' 'LDLIBS=$(LDLIBS)'

clean:
	$(RM) -r build/unified/$(PF_CANON)
	$(RM) $(TARGET) $(GUI_TARGET)
