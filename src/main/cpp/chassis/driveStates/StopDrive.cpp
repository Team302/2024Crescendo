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

// Team302 Includes
#include "chassis/ChassisMovement.h"
#include "chassis/driveStates/RobotDrive.h"
#include "chassis/driveStates/StopDrive.h"
#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"

#include "utils/logging/Logger.h"

StopDrive::StopDrive(RobotDrive *robotDrive) : RobotDrive(robotDrive->GetChassis()),
                                               m_robotDrive(robotDrive)
{
}

std::string StopDrive::GetDriveStateName() const
{
    return std::string("StopDrive");
}

std::array<frc::SwerveModuleState, 4> StopDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)

{
    return {*m_flState, *m_frState, *m_blState, *m_brState};
}

void StopDrive::Init(ChassisMovement &chassisMovement)
{
    // set module angles
    if (m_chassis != nullptr)
    {
        auto module = m_chassis->GetFrontLeft();
        if (module != nullptr)
        {
            auto currState = module->GetState();
            m_flState->angle = currState.angle;
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "StopDrive", "fl", currState.angle.Degrees().to<double>());
        }

        module = m_chassis->GetFrontRight();
        if (module != nullptr)
        {
            auto currState = module->GetState();
            m_frState->angle = currState.angle;
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "StopDrive", "fr", currState.angle.Degrees().to<double>());
        }

        module = m_chassis->GetBackLeft();
        if (module != nullptr)
        {
            auto currState = module->GetState();
            m_blState->angle = currState.angle;
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "StopDrive", "bl", currState.angle.Degrees().to<double>());
        }

        module = m_chassis->GetBackRight();
        if (module != nullptr)
        {
            auto currState = module->GetState();
            m_brState->angle = currState.angle;
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "StopDrive", "br", currState.angle.Degrees().to<double>());
        }
    }

    // set the module speeds
    m_flState->speed = 0_mps;
    m_frState->speed = 0_mps;
    m_blState->speed = 0_mps;
    m_brState->speed = 0_mps;
}
