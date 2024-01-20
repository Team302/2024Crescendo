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

#include <cmath>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

// Team302 Includes
#include <chassis/swerve/driveStates/VisionDrive.h>
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include <utils/FMSData.h>
#include <robotstate/RobotState.h>
#include <utils/FMSData.h>
#include "teleopcontrol/TeleopControl.h"

/// DEBUGGING
#include "utils/logging/Logger.h"

VisionDrive::VisionDrive(RobotDrive *robotDrive) : RobotDrive(),
                                                   IRobotStateChangeSubscriber(),
                                                   // m_visionVYPID(1.0, 0.05, 0.0), // kP, kI, kD
                                                   // m_visionVXPID(1.0, 0.05, 0.0), // kP, kI, kD
                                                   m_alignmentMethod(ALIGNMENT_METHOD::ROTATE),
                                                   m_pipelineMode(DragonCamera::APRIL_TAG),
                                                   m_inAutonMode(false),
                                                   m_robotDrive(robotDrive),
                                                   m_chassis(nullptr),
                                                   m_vision(DragonVision::GetDragonVision()),
                                                   m_lostGamePieceTimer(new frc::Timer()),
                                                   m_haveGamePiece(false),
                                                   m_moveInXDir(false)
{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::HoldingGamePiece);
}

std::array<frc::SwerveModuleState, 4> VisionDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)
{
    // auto targetData = DragonVision::GetDragonVision()->getTargetInfo();

    // bool targetDataIsNullptr = targetData == nullptr;
    /*
    if (!targetDataIsNullptr)
    {
        m_lostGamePieceTimer->Stop();
        m_lostGamePieceTimer->Reset();
    }
    else
    {
        m_lostGamePieceTimer->Start();
    }

    if (!targetDataIsNullptr)
    {
        // if (m_vision->getPipeline(DragonVision::LIMELIGHT_POSITION::FRONT) == targetData->getTargetType())
        {
            // bool atTarget_angle = false;

            units::angle::radian_t angleError = units::angle::radian_t(0.0);

            // atTarget_angle = AtTargetAngle(targetData, &angleError);
            AtTargetAngle(targetData, &angleError);

            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "Angle Error (Deg)", units::angle::degree_t(angleError).to<double>());

            // Do not move in the X direction until the other measure angle is within a certain tolerance
            if (std::abs(units::angle::degree_t(angleError).to<double>()) < m_inhibitXspeedAboveAngularError.to<double>())
                m_moveInXDir = true;

            if (std::abs(units::angle::degree_t(angleError).to<double>()) > m_stopXSpeedAboveAngleError.to<double>())
                m_moveInXDir = false;

             if (m_vision->getPipeline(DragonVision::LIMELIGHT_POSITION::FRONT) == DragonLimelight::PIPELINE_MODE::APRIL_TAG)
            {
                m_moveInXDir = false;
            }

            if (m_moveInXDir)
            {
                units::velocity::meters_per_second_t xSpeed = units::velocity::meters_per_second_t(0.0);
                units::length::inch_t xError = targetData->getXdistanceToTargetRobotFrame() - units::length::inch_t(m_centerOfRobotToBumperEdge_in + m_gamePieceToBumperOffset);

                if (units::math::abs(xError).to<double>() < m_xErrorThreshold)
                {
                    m_xErrorUnderThreshold = true;
                }

                xSpeed = units::length::meter_t(xError * m_visionKP_X) / 1_s;

                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XSpeed Before Limiting (MPS)", xSpeed.to<double>());

                xSpeed = limitVelocityToBetweenMinAndMax(xSpeed);

                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "XSpeed After Limiting (MPS)", xSpeed.to<double>());

                chassisMovement.chassisSpeeds.vx = xSpeed;
            }
        }
    }

    else if (m_xErrorUnderThreshold)
    {
        chassisMovement.chassisSpeeds.vx = units::velocity::meters_per_second_t(m_minimumSpeed_mps);
    }*/

    return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
}

void VisionDrive::Init(ChassisMovement &chassisMovement)
{
}

units::velocity::meters_per_second_t VisionDrive::limitVelocityToBetweenMinAndMax(units::velocity::meters_per_second_t velocity)
{
    double sign = velocity.to<double>() < 0 ? -1 : 1;

    if (std::abs(velocity.to<double>()) < m_minimumSpeed_mps)
        velocity = units::length::meter_t(m_minimumSpeed_mps * sign) / 1_s;

    if (std::abs(velocity.to<double>()) > m_maximumSpeed_mps)
        velocity = units::length::meter_t(m_maximumSpeed_mps * sign) / 1_s;

    return velocity;
}

units::angular_velocity::radians_per_second_t VisionDrive::limitAngularVelocityToBetweenMinAndMax(units::angular_velocity::radians_per_second_t angularVelocity)
{
    double sign = angularVelocity.to<double>() < 0 ? -1 : 1;

    if (std::abs(angularVelocity.to<double>()) < m_minimumOmega_radps)
        angularVelocity = units::angular_velocity::radians_per_second_t(m_minimumOmega_radps * sign);

    if (std::abs(angularVelocity.to<double>()) > m_maximumOmega_radps)
        angularVelocity = units::angular_velocity::radians_per_second_t(m_maximumOmega_radps * sign);

    return angularVelocity;
}

// visionapi - revisit this to return back atTargetX value for visiondrive
// bool VisionDrive::AtTargetX(std::shared_ptr<DragonVisionTarget> targetData)
/*{
    if (targetData != nullptr)
    {
        units::length::inch_t xError = targetData->getXdistanceToTargetRobotFrame() - units::length::inch_t(m_centerOfRobotToBumperEdge_in + m_visionAlignmentXoffset_in);
        DragonLimelight::PIPELINE_MODE pipelineMode = DragonVision::GetDragonVision()->getPipeline(DragonVision::LIMELIGHT_POSITION::FRONT);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "VisionDrive", "AtTargetX_XError", xError.to<double>());

        if (pipelineMode == DragonLimelight::PIPELINE_MODE::APRIL_TAG)
        {
            if (std::abs(xError.to<double>()) < m_linearTolerance_in)
            {
                return true;
            }
        }
        else
        {
            units::angle::degree_t verticalAngle = targetData->getVerticalAngleToTarget();

            // vertical angle is positive, so we are looking at high cone
            if (verticalAngle.to<double>() > 0.0)
            {
                if ((std::abs(xError.to<double>()) - m_highConeDistance) < m_linearTolerance_in)
                {
                    return true;
                }
            }
            else // vert angle is negative, so we're looking at low cone
            {
                if ((std::abs(xError.to<double>()) - m_lowConeDistance) < m_linearTolerance_in)
                {
                    return true;
                }
            }
        }
    }
   return false;
}*/
/*
    bool VisionDrive::AtTargetY(std::shared_ptr<DragonVisionTarget> targetData)
    {
        if (targetData != nullptr)
        {
            units::length::inch_t yError = targetData->getYdistanceToTargetRobotFrame();

            if (std::abs(yError.to<double>()) < m_linearTolerance_in)
            {
                return true;
            }
        }
        return false;
    }

    bool VisionDrive::AtTargetAngle(std::shared_ptr<DragonVisionTarget> targetData, units::angle::radian_t * error)
    {
        if (targetData != nullptr)
        {
            units::length::inch_t yError = targetData->getYdistanceToTargetRobotFrame();
            units::length::inch_t xError = targetData->getXdistanceToTargetRobotFrame();

            if (std::abs(xError.to<double>()) > 0.01)
            {
                *error = units::angle::radian_t(std::atan2(yError.to<double>(), xError.to<double>()));

                if (std::abs((*error).to<double>()) < m_AngularTolerance_rad)
                {
                    return true;
                }
            }
        }
        return false;
    }

    bool VisionDrive::isAligned(DragonLimelight::PIPELINE_MODE pipelineMode)
    {
         if (pipelineMode == DragonLimelight::PIPELINE_MODE::APRIL_TAG)
         {
             auto targetData = DragonVision::GetDragonVision()->getTargetInfo();
             if (targetData != nullptr)
             {
                 units::angle::radian_t angleError = units::angle::radian_t(0.0);
                 return AtTargetAngle(targetData, &angleError);
             }
         }
         else
         {
             return m_haveGamePiece;
         }
return false;
}*/

void VisionDrive::ResetVisionDrive()
{
    yErrorIntegral = units::length::inch_t(0);
    m_xErrorUnderThreshold = false;
    m_haveGamePiece = false;
    m_moveInXDir = false;
    m_lostGamePieceTimer->Stop();
    m_lostGamePieceTimer->Reset();
}

void VisionDrive::Update(RobotStateChanges::StateChange change, int value)
{
    if (change == RobotStateChanges::StateChange::HoldingGamePiece)
    {
        m_haveGamePiece = static_cast<RobotStateChanges::GamePiece>(value) != RobotStateChanges::None;
    }
}
