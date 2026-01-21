# ====== Config ======
CXX      := g++
TARGET   := test

SRC      := tc_6axis.cpp DriverConnection.cpp
OBJ      := $(SRC:.cpp=.o)

# Flags có thể chỉnh
AXIS     ?= AXIS_2
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -I. -D$(AXIS)
# Thư mục chứa lib (mặc định: current dir ".")
LIBDIR   ?= .
LDFLAGS  ?= -L$(LIBDIR) -Wl,-rpath,'$$ORIGIN'
LDLIBS   ?= -lEziMOTIONPlusR

# ====== Rules ======
.PHONY: all clean run debug

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

debug: CXXFLAGS := -std=c++17 -g -O0 -Wall -Wextra -I.
debug: clean all

clean:
	$(RM) $(OBJ) $(TARGET)
