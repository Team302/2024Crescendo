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

// C++
#include <string>

// FRC Includes
#include "frc/controller/PIDController.h"
#include "frc/controller/ProfiledPIDController.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/Filesystem.h"
#include "frc/trajectory/TrajectoryUtil.h"
#include "units/angular_velocity.h"
#include "wpi/fs.h"

// 302 Includes
#include "auton/drivePrimitives/DrivePath.h"
#include "auton/drivePrimitives/DragonTrajectoryUtils.h"
#include "chassis/ChassisMovement.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/IChassis.h"
#include "utils/logging/Logger.h"
#include "chassis/driveStates/TrajectoryDrive.h"

using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePath::DrivePath() : m_chassis(nullptr),
                         m_timer(make_unique<Timer>()),
                         m_trajectory(),
                         m_runHoloController(true),
                         // max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                         m_headingOption(ChassisOptionEnums::HeadingOption::MAINTAIN),
                         m_heading(0.0),
                         m_maxTime(units::time::second_t(-1.0)),
                         m_ntName("DrivePath")

{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}
void DrivePath::Init(PrimitiveParams *params)
{
    m_pathname = params->GetPathName(); // Grabs path name from auton xml
    m_ntName = string("DrivePath: ") + m_pathname;
    m_headingOption = params->GetHeadingOption();
    m_heading = params->GetHeading();
    m_maxTime = params->GetTime();

    m_trajectory = DragonTrajectoryUtils::GetTrajectory(params);

    // Start timeout timer for path
    m_timer.get()->Reset();
    m_timer.get()->Start();
}
void DrivePath::Run()
{

    ChassisMovement moveInfo;
    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE;
    moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    moveInfo.headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;

    // Use the controller to calculate the chassis speeds for getting there
    if (m_runHoloController)
    {
        switch (m_headingOption)
        {
        case ChassisOptionEnums::HeadingOption::MAINTAIN:
            break;

        case ChassisOptionEnums::HeadingOption::TOWARD_GOAL:
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::TOWARD_GOAL;
            break;

        case ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE:
            moveInfo.headingOption = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;
            moveInfo.yawAngle = units::angle::degree_t(m_heading);

            break;

        default:
            break;
        }
    }
    else
    {
        moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::RAMSETE;
    }

    moveInfo.trajectory = m_trajectory;
    m_chassis->Drive(moveInfo);
}

bool DrivePath::IsDone()
{

    if (m_timer.get()->Get() > m_maxTime && m_timer.get()->Get() > units::time::second_t(0.0))
    {
        return true;
    }
    else
    {
        SwerveChassis *swerveChassis = dynamic_cast<SwerveChassis *>(m_chassis);
        TrajectoryDrive *trajectoryDrive = dynamic_cast<TrajectoryDrive *>(swerveChassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE));

        if (trajectoryDrive->IsDone()) // TrajectoryDrive is done -> log the reason why and end drive path primitive
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "WhyDone", trajectoryDrive->WhyDone());
            return true;
        }
        else // TrajectoryDrive isn't done
        {
            return false;
        }

        return false;
    }
}