
//====================================================================================================================================================
// Copyright 2024 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once
#include <driveteamfeedback/LEDStates.h>
#include <robotstate/IRobotStateChangeSubscriber.h>

class DriverFeedback : public IRobotStateChangeSubscriber
{
public:
    void UpdateFeedback();

    static DriverFeedback *GetInstance();

    void UpdateLEDStates();

    void UpdateCompressorState();

    void Update(RobotStateChanges::StateChange change, int value) override;

private:
    void UpdateRumble();
    void UpdateDiagnosticLEDs();
    void CheckControllers();
    void DisplayPressure() const;
    void DisplayDesiredGamePiece();
    void ResetRequests(void);
    DriverFeedback();
    ~DriverFeedback() = default;

    bool m_AutonomousEnabled = false;
    bool m_TeleopEnabled = false;

    DragonLeds::Colors oldState = DragonLeds::WHITE;
    DragonLeds::Colors currentState = DragonLeds::BLACK;

    enum DriverFeedbackStates
    {
        NONE
    };

    LEDStates *m_LEDStates = LEDStates::GetInstance();
    int m_controllerCounter = 0;
    bool m_rumbleLauncher = false;
    bool m_rumblePlacer = false;
    bool m_rumbleIntake = false;
    int m_rumbleLoopCounter = 0;
    int m_firstloop = true;

    static DriverFeedback *m_instance;
    RobotStateChanges::ScoringMode m_scoringMode = RobotStateChanges::ScoringMode::Launcher;
    RobotStateChanges::ClimbMode m_climbMode = RobotStateChanges::ClimbMode::ClimbModeOff;
};
