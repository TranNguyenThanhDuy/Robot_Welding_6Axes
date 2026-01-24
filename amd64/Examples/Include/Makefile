# ====== Config ======
CXX      := g++
TARGET   := test
GUI_TARGET := tc_6axis_gui

SRC      := tc_6axis.cpp axis_controller.cpp DriverConnection.cpp
GUI_SRC  := tc_6axis_gui.cpp axis_controller.cpp DriverConnection.cpp

# Flags có thể chỉnh
AXIS     ?= AXIS_6
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -I. -D$(AXIS)
# Thư mục chứa lib (mặc định: current dir ".")
LIBDIR   ?= .
LDFLAGS  ?= -L$(LIBDIR) -Wl,-rpath,'$$ORIGIN'
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
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

gui: $(GUI_TARGET)

$(GUI_TARGET): $(GUI_OBJ)
	$(CXX) -o $@ $^ $(LDFLAGS) -no-pie $(LDLIBS) $(QT_LDLIBS)

$(GUI_OBJDIR)/%.o: %.cpp | $(GUI_OBJDIR)
	$(CXX) $(CXXFLAGS) $(QT_CXXFLAGS) -fPIC -c $< -o $@

debug: CXXFLAGS := -std=c++17 -g -O0 -Wall -Wextra -I.
debug: clean all

clean:
	$(RM) -r build
	$(RM) $(TARGET) $(GUI_TARGET)
