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

// FRC includes

// Team 302 includes
#include <utils/logging/DataTrace.h>

// Third Party Includes
#include <stdio.h>

DataTrace *DataTrace::m_instance = nullptr;
DataTrace *DataTrace::GetInstance()
{
    if (DataTrace::m_instance == nullptr)
    {
        DataTrace::m_instance = new DataTrace();
    }
    return DataTrace::m_instance;
}

DataTrace::DataTrace()
{
}

void DataTrace::sendClimberData(double angle, double power)
{
    sprintf(sendBuffer, "$$Climber:%.3f:%.2f:%.2f##", m_timer.Get().to<double>(), angle, power);
    SendData();
}

void DataTrace::sendElevatorData(double ElevatorHeight)
{
    sprintf(sendBuffer, "$$Placer:%.3f:%.2f##", m_timer.Get().to<double>(), ElevatorHeight);
    SendData();
}

void DataTrace::sendLauncherData(double WheelSetTop, double WheelSetBottom, double Angle, double topWheelCurrent, double bottomWheelCurrent, double theState, double xbutton)
{
    sprintf(sendBuffer, "$$Launcher:%.3f:%.2f:%.2f:%.2f:%.2f:%.2f:%.2f:%.2f##", m_timer.Get().to<double>(), WheelSetTop, WheelSetBottom, Angle, topWheelCurrent, bottomWheelCurrent, theState, xbutton);
    SendData();
}

void DataTrace::sendNoteSensorData(double FrontIntake, double BackIntake, double Feeder, double Launcher, double PlacerIn, double PlacerMid, double PlacerOut)
{
    sprintf(sendBuffer, "$$NoteSensor:%.3f:%.2f:%.2f:%.2f:%.2f:%.2f:%.2f:%.2f##", m_timer.Get().to<double>(), FrontIntake, BackIntake, Feeder, Launcher, PlacerIn, PlacerMid, PlacerOut);
    SendData();
}
void DataTrace::sendNoteMotorData(double FrontIntake, double BackIntake, double Transfer, double Placer, double Feeder, double Elevator, double IntakeDifference, double NoteInIntake)
{
    sprintf(sendBuffer, "$$NoteMotor:%.3f:%.2f:%.2f:%.2f:%.2f:%.2f:%.2f:%.2f:%.2f##", m_timer.Get().to<double>(), FrontIntake, BackIntake, Transfer, Placer, Feeder, Elevator, IntakeDifference, NoteInIntake);
    SendData();
}

// int test(void)
// {
//     send(client_fd, hello, strlen(hello), 0);
//     printf("Hello message sent\n");
//     valread = read(client_fd, buffer, 1024);
//     printf("%s\n", buffer);

//     // closing the connected socket
//     close(client_fd);
//     return 0;
// }