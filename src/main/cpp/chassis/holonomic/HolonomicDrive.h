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

// C++ Libraries

// Team 302 includes
#include "teleopcontrol/TeleopControl.h"
#include "State.h"
#include <chassis/swerve/driveStates/DragonTrajectoryGenerator.h>
#include <utils/DragonField.h>
#include <chassis/swerve/driveStates/VisionDrive.h>
#include <robotstate/IRobotStateChangeSubscriber.h>

class IChassis;
class SwerveChassis;

class HolonomicDrive : public State, public IRobotStateChangeSubscriber
{
public:
    HolonomicDrive();
    ~HolonomicDrive() = default;

    void Init() override;
    void Run() override;
    void Exit() override;
    bool AtTarget() override;

    void Update(RobotStateChanges::StateChange change, int state) override;

private:
    std::pair<ChassisOptionEnums::RELATIVE_POSITION, ChassisOptionEnums::RELATIVE_POSITION> GetAutoAlignDestination();

    bool IsAutoAligning();

    IChassis *m_chassis;
    SwerveChassis *m_swerve;
    DragonTrajectoryGenerator *m_trajectoryGenerator;
    ChassisOptionEnums::DriveStateType m_previousDriveState;
    frc::Trajectory m_generatedTrajectory;
    DragonField *m_field;
    const double m_slowModeMultiplier = 0.5;
    const double m_autoAlignAngleTolerance = 5.0;
    bool m_hasResetPosition = false;
    bool m_inVisionDrive = false;
    bool m_CheckTipping = false;
    bool m_latch = false;
    bool m_findingFloorGamePiece = false;

    RobotStateChanges::GamePiece m_desiredGamePiece;
};