CC = g++
CXXFLAGS = -g -o
LDFLAGS = -lEziMOTIONPlusR
OBJS = ConnectionExam ParameterTestExam InputPinControlExam OutputPinControlExam JogMovementExam JogMovementExExam MoveAbsIncPosExam MoveAbsIncPosExExam PositionOverrideExam VelocityOverrideExam OriginSearchExam SetPositionTableExam PushStopExam PushNonStopExam GetInputExam LatchExam SetOutputExam TriggerExam IOLevelExam MoveLinearAbsIncPosExam

all : $(OBJS)

ConnectionExam : 01.ConnectionExam/ConnectionExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

ParameterTestExam : 02.ParameterTestExam/ParameterTestExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

InputPinControlExam: 03.InputPinControlExam/InputPinControlExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

OutputPinControlExam : 04.OutputPinControlExam/OutputPinControlExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

JogMovementExam : 05.JogMovementExam/JogMovementExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

JogMovementExExam : 06.JogMovementExExam/JogMovementExExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

MoveAbsIncPosExam : 07.MoveAbsIncPosExam/MoveAbsIncPosExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

MoveAbsIncPosExExam : 08.MoveAbsIncPosExExam/MoveAbsIncPosExExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

PositionOverrideExam : 09.PositionOverrideExam/PositionOverrideExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

VelocityOverrideExam : 10.VelocityOverrideExam/VelocityOverrideExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

OriginSearchExam : 11.OriginSearchExam/OriginSearchExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

SetPositionTableExam : 12.SetPositionTableExam/SetPositionTableExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

PushStopExam : 13.PushStopExam/PushStopExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

PushNonStopExam : 14.PushNonStopExam/PushNonStopExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

GetInputExam : 15.GetInputExam/GetInputExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

LatchExam : 16.LatchExam/LatchExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

SetOutputExam : 17.SetOutputExam/SetOutputExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

TriggerExam : 18.TriggerExam/TriggerExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

IOLevelExam : 19.IOLevelExam/IOLevelExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

MoveLinearAbsIncPosExam : 20.MoveLinearAbsIncPosExam/MoveLinearAbsIncPosExam.cpp
	$(CC) $(CXXFLAGS) $@ $^ $(LDFLAGS)

clean :
	rm -f $(OBJS)
	rm -rf Include
