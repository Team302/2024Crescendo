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
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/trajectory/TrajectoryUtil.h"
#include "units/angular_velocity.h"
#include "wpi/fs.h"

// 302 Includes
#include <auton/drivePrimitives/DrivePathPlanner.h>
#include <auton/drivePrimitives/DragonTrajectoryUtils.h>
#include <chassis/ChassisMovement.h>
#include <chassis/ChassisOptionEnums.h>
#include "configs/RobotConfigMgr.h"
#include "configs/RobotConfig.h"
#include <chassis/IChassis.h>
#include "utils/logging/Logger.h"
#include "chassis/swerve/driveStates/TrajectoryDrivePathPlanner.h"

// third party includes
#include "pathplanner/lib/path/PathPlannerTrajectory.h"
#include "pathplanner/lib/path/PathPlannerPath.h"

using namespace pathplanner;

using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePathPlanner::DrivePathPlanner() : m_chassis(nullptr),
                                       m_timer(make_unique<Timer>()),
                                       m_trajectory(),
                                       m_pathname(),
                                       // max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                                       m_maxTime(units::time::second_t(-1.0)),
                                       m_ntName("DrivePathPlanner")

{
    auto config = RobotConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}
void DrivePathPlanner::Init(PrimitiveParams *params)
{
    m_pathname = params->GetPathName(); // Grabs path name from auton xml
    m_ntName = string("DrivePathPlanner: ") + m_pathname;
    m_maxTime = params->GetTime();

    // TODO:  things have been obsoleted in 2024, so we need to re-work this
    //m_trajectory = PathPlannerPath::loadPath(m_pathname, PathConstraints(4.5_mps, 2.75_mps_sq));

    // Start timeout timer for path
    m_timer.get()->Reset();
    m_timer.get()->Start();
}
void DrivePathPlanner::Run()
{

    ChassisMovement moveInfo;
    moveInfo.driveOption = ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER;
    moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    moveInfo.headingOption = ChassisOptionEnums::HeadingOption::IGNORE;

    moveInfo.pathplannerTrajectory = m_trajectory;
    m_chassis->Drive(moveInfo);
}

bool DrivePathPlanner::IsDone()
{

    if (m_timer.get()->Get() > m_maxTime && m_timer.get()->Get().to<double>() > 0.0)
    {
        return true;
    }
    else
    {
        TrajectoryDrivePathPlanner *trajectoryDrive = dynamic_cast<TrajectoryDrivePathPlanner *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER));

        if (trajectoryDrive->IsDone()) // TrajectoryDrive is done -> log the reason why and end drive path primitive
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "WhyDone", trajectoryDrive->WhyDone());
            return true;
        }
        else // TrajectoryDrive isn't done
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, m_ntName, "WhyDone", "Not done");
            return false;
        }

        return false;
    }
}