//====================================================================================================================================================
/// Copyright 2024 Lake Orion Robotics FIRST Team 302
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

// FRC includes
// #define INCLUDE_DATA_TRACE

// Team 302 includes
#include <utils/logging/DataTraceSocket.h>

// Third Party Includes

class DataTrace : public DataTraceSocket
{
public:
    static DataTrace *GetInstance();
    DataTrace();
    ~DataTrace() = default;

    void sendClimberData(double angle, double power);
    void sendElevatorData(double ElevatorHeight);
    void sendLauncherData(double WheelSetTop, double WheelSetBottom, double Angle, double topWheelCurrent, double bottomWheelCurrent, double theState, double xbutton);
    void sendNoteSensorData(double FrontIntake, double BackIntake, double Feeder, double Launcher, double PlacerIn, double PlacerMid, double PlacerOut);
    void sendNoteMotorData(double FrontIntake, double BackIntake, double Transfer, double Placer, double Feeder, double Elevator, double IntakeDifference, double NoteInIntake);

private:
    static DataTrace *m_instance;
};