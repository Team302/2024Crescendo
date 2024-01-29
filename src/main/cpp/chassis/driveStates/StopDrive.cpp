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
#include "chassis/driveStates/AntiTip.h"
#include "chassis/driveStates/RobotDrive.h"
#include "chassis/driveStates/StopDrive.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"

StopDrive::StopDrive(RobotDrive *robotDrive) : RobotDrive(),
                                               m_robotDrive(robotDrive),
                                               m_chassis(nullptr)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}

std::array<frc::SwerveModuleState, 4> StopDrive::UpdateSwerveModuleStates(ChassisMovement &chassisMovement)

{
    if (chassisMovement.checkTipping)
    {
        AntiTip::CorrectForTipping(chassisMovement, m_maxspeed); // TODO:  can we remove this; robot drive handles this
        return m_robotDrive->UpdateSwerveModuleStates(chassisMovement);
    }

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
        }

        module = m_chassis->GetFrontRight();
        if (module != nullptr)
        {
            auto currState = module->GetState();
            m_frState->angle = currState.angle;
        }

        module = m_chassis->GetBackLeft();
        if (module != nullptr)
        {
            auto currState = module->GetState();
            m_blState->angle = currState.angle;
        }

        module = m_chassis->GetBackRight();
        if (module != nullptr)
        {
            auto currState = module->GetState();
            m_brState->angle = currState.angle;
        }
    }

    // set the module speeds
    m_flState->speed = 0_mps;
    m_frState->speed = 0_mps;
    m_blState->speed = 0_mps;
    m_brState->speed = 0_mps;
}