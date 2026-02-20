# ====== Config ======
CXX      := g++
TARGET   := test
GUI_TARGET := gui

SRC      := source/tc_6axis.cpp source/axis_controller.cpp source/DriverConnection.cpp
GUI_SRC  := source/tc_6axis_gui.cpp source/axis_controller.cpp source/DriverConnection.cpp

# Flags có thể chỉnh
AXIS     ?= AXIS_5
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Isource -Ilib -D$(AXIS)
# Thư mục chứa lib (mặc định: current dir ".")
LIBDIR   ?= lib
LDFLAGS  ?= -L$(LIBDIR) -Wl,-rpath,'$$ORIGIN/lib'
LDLIBS   ?= -lEziMOTIONPlusR

QT_CXXFLAGS := $(shell pkg-config --cflags Qt5Widgets)
QT_LDLIBS   := $(shell pkg-config --libs Qt5Widgets)

OBJDIR := build/$(AXIS)
GUI_OBJDIR := build/$(AXIS)_gui
OBJ := $(SRC:%.cpp=$(OBJDIR)/%.o)
GUI_OBJ := $(GUI_SRC:%.cpp=$(GUI_OBJDIR)/%.o)

# ====== Rules ======
.PHONY: all clean run debug gui

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
	$(CXX) -o $@ $^ $(LDFLAGS) -no-pie $(LDLIBS) $(QT_LDLIBS)

$(GUI_OBJDIR)/%.o: %.cpp | $(GUI_OBJDIR)
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(QT_CXXFLAGS) -fPIC -c $< -o $@

debug: CXXFLAGS := -std=c++17 -g -O0 -Wall -Wextra -Isource -Ilib
debug: clean all

clean:
	$(RM) -r build
	$(RM) $(TARGET) $(GUI_TARGET)
