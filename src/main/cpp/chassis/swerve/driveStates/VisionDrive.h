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

// FRC Includes
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/Timer.h>

// Team302 Includes
#include <chassis/swerve/driveStates/RobotDrive.h>
#include <chassis/swerve/driveStates/FieldDrive.h>
#include <DragonVision/DragonVision.h>
#include <robotstate/IRobotStateChangeSubscriber.h>
#include "chassis/swerve/driveStates/VisionDrive.h"

class VisionDrive : public RobotDrive
{
public:
    enum ALIGNMENT_METHOD
    {
        ROTATE,
        STRAFE
    };

    VisionDrive(RobotDrive *robotDrive);

    std::array<frc::SwerveModuleState, 4> UpdateSwerveModuleStates(
        ChassisMovement &chassisMovement) override;

    void Init(
        ChassisMovement &chassisMovement) override;

    void ResetVisionDrive();

    bool isAligned(DragonCamera::PIPELINE pipelineMode);

    void setAlignmentMethod(ALIGNMENT_METHOD alignmentMethod) { m_alignmentMethod = alignmentMethod; }
    void setVisionPipeline(DragonCamera::PIPELINE pipeline) { m_pipelineMode = pipeline; }
    void setInAutonMode(bool inAuton) { m_inAutonMode = inAuton; }

private:
    enum VISION_STATE
    {
        NORMAL_DRIVE,
        LOOKING_FOR_VISION_TARGET,
        FOUND_VISION_TARGET,
        DRIVE_TO_TARGET,
        ALIGN_RAW_VISION,
        ALIGNED
    };

    ALIGNMENT_METHOD m_alignmentMethod;
    DragonCamera::PIPELINE m_pipelineMode;
    bool m_inAutonMode;

    void AlignRawVision(ChassisMovement &chassisMovement);

    bool AtTargetX(VisionData visionData);
    bool AtTargetY(VisionData vivionData);
    bool AtTargetAngle(VisionData visionData, units::angle::radian_t *angleError);

    RobotDrive *m_robotDrive;

    double m_kP_X = 0.1;
    double m_kP_Y = 0.075;

    int m_aprilTagID = -1;
    units::length::inch_t m_yDistanceToTarget = units::length::inch_t(0.0);
    units::length::inch_t m_xDistanceToTarget = units::length::inch_t(0.0);

    units::length::inch_t yErrorIntegral;

    // const double m_autoAlignKP = 0.075; //original
    const double m_autoAlignKP_Y = 0.05;  // used for driving to target
    const double m_autoAlignKP_X = 0.035; // used for driving to target
    const double m_visionKP_X = 1.5;      // used for vision based alignment
    double m_visionKP_Y = 2.5;            // used for vision based alignment
    double m_visionKI_Y = 0.00;           // used for vision based alignment

    // Linear movement settings for both X and Y directions
    const double m_minimumSpeed_mps = 1.25;
    const double m_maximumSpeed_mps = 2.0;
    const double m_linearTolerance_in = 2;

    // Linear movement settings for X direction
    const double m_inhibitXspeedAboveYError_in = 2.5;
    const double m_centerOfRobotToBumperEdge_in = 16.0;
    const double m_gamePieceToBumperOffset = 0.0;
    bool m_xErrorUnderThreshold = false;
    const double m_xErrorThreshold = 3.0;

    // Angular movement settings
    const double m_minimumOmega_radps = 0.7;
    const double m_maximumOmega_radps = 1.0;
    const double m_AngularTolerance_rad = std::numbers::pi * 4.0 / 180.0;
    const units::angle::degree_t m_inhibitXspeedAboveAngularError = units::angle::degree_t(5.0);
    const units::angle::degree_t m_stopXSpeedAboveAngleError = units::angle::degree_t(10.0);
    double m_visionKP_Angle = 2;

    // Other stuff
    const double m_autoAlignYTolerance = 2.5;  // tolerance in inches
    const double m_autoAlignXTolerance = 10.0; // tolerance in inches
    const double m_driveXTolerance = 19.5;     // tolerance in inches

    const double m_robotFrameXDistCorrection = 32.0; // Corrects for physical barrier to april tag, can never get closer than 30 inches
    const double m_robotFrameGapToTag = 7.0;         // This 6 inches  is so we don't scrape while driving in y direction

    const double m_AprilTagTargetDistance = 32.0 / 2.0 + 5; // to stop 5" off the wood
    const double m_highConeDistance = 45.0;
    const double m_lowConeDistance = 40.0;

    units::length::inch_t m_yTargetPos = units::length::inch_t(0.0);
    units::length::inch_t m_xTargetPos = units::length::inch_t(0.0);

    SwerveChassis *m_chassis;

    DragonVision *m_vision;

    frc::Timer *m_lostGamePieceTimer;
    const units::second_t m_lostGamePieceTimeout = units::second_t(0.25);

    bool m_haveGamePiece;
    bool m_moveInXDir;

    double getOffsetToTarget(ChassisOptionEnums::RELATIVE_POSITION targetGrid, ChassisOptionEnums::RELATIVE_POSITION targetNode, uint8_t AprilTagId);
    units::velocity::meters_per_second_t limitVelocityToBetweenMinAndMax(units::velocity::meters_per_second_t speed);
    units::angular_velocity::radians_per_second_t limitAngularVelocityToBetweenMinAndMax(units::angular_velocity::radians_per_second_t angularSpeed);
};