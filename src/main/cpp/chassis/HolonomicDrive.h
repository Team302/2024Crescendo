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

#include "frc/geometry/Pose2d.h"

// Team 302 includes
#include "chassis/ChassisMovement.h"
#include "State.h"

class SwerveChassis;

class HolonomicDrive : public State
{
public:
    HolonomicDrive();
    ~HolonomicDrive() = default;

    void Init() override;
    void Run() override;
    void Exit() override;
    bool AtTarget() override;

private:
    void InitChassisMovement();
    void InitSpeeds(double forwardScale, double strafeScale, double rotateScale);
    void ResetPose();
    void AlignGamePiece();
    void HoldPosition();
    void TurnForward();
    void TurnBackward();
    void SlowMode();
    void CheckTipping(bool tippingSelected);
    void CheckRobotOriented(bool robotOrientedSelected);
    void AlignToSpeaker();
    void AlignToAmp();
    void AlignToStage();
    void TurnToPassAngle();

    void DriveToGamePiece(double forward, double strafe, double rot);

    SwerveChassis *m_swerve;
    ChassisOptionEnums::DriveStateType m_previousDriveState;
    const double m_slowModeMultiplier = 0.5;
    bool m_CheckTipping = false;
    bool m_checkTippingLatch = false;
    ChassisMovement m_moveInfo;

    bool m_robotOrientedLatch = false;
    bool m_robotOrientedDrive = false;
};