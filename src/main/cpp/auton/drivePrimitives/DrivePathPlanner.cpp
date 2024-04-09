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
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"

// 302 Includes
#include "auton/drivePrimitives/DrivePathPlanner.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"
#include "chassis/ChassisMovement.h"
#include "chassis/ChassisOptionEnums.h"
#include "chassis/DragonDriveTargetFinder.h"
#include "chassis/driveStates/DriveToNote.h"
#include "chassis/driveStates/TrajectoryDrivePathPlanner.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include "DragonVision/DragonVision.h"
#include "mechanisms/base/StateMgr.h"
#include "mechanisms/MechanismTypes.h"
#include "utils/FMSData.h"
#include "utils/logging/Logger.h"
#include "chassis/driveStates/RobotDrive.h"

// third party includes
#include "pathplanner/lib/path/PathPlannerTrajectory.h"
#include "pathplanner/lib/path/PathPlannerPath.h"

using namespace pathplanner;

using namespace std;
using namespace frc;

using namespace wpi::math;

DrivePathPlanner::DrivePathPlanner() : IPrimitive(),
                                       m_chassis(nullptr),
                                       m_timer(make_unique<Timer>()),
                                       m_trajectory(),
                                       m_pathname(),
                                       m_pathGainsType(ChassisOptionEnums::PathGainsType::LONG),
                                       // max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                                       m_maxTime(units::time::second_t(-1.0)),
                                       m_ntName("DrivePathPlanner"),
                                       m_switchedToVisionDrive(false),
                                       m_visionAlignment(PrimitiveParams::VISION_ALIGNMENT::UNKNOWN)

{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
}
void DrivePathPlanner::Init(PrimitiveParams *params)
{
    m_pathname = params->GetPathName(); // Grabs path name from auton xml
    m_pathGainsType = params->GetPathGainsType();

    m_ntName = string("DrivePathPlanner: ") + m_pathname;
    m_maxTime = params->GetTime();
    m_switchedToVisionDrive = false;
    m_visionAlignment = params->GetVisionAlignment();
    m_checkDriveToNote = params->GetPathUpdateOption() == ChassisOptionEnums::PathUpdateOption::NOTE;
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("DrivePathPlanner"), m_pathname, m_chassis->GetPose().Rotation().Degrees().to<double>());

    // Start timeout timer for path

    InitMoveInfo();

    m_timer.get()->Reset();
    m_timer.get()->Start();
}

void DrivePathPlanner::InitMoveInfo()
{

    m_moveInfo.controllerType = ChassisOptionEnums::AutonControllerType::HOLONOMIC;
    m_moveInfo.headingOption = (m_visionAlignment == PrimitiveParams::VISION_ALIGNMENT::SPEAKER) ? ChassisOptionEnums::HeadingOption::FACE_SPEAKER : ChassisOptionEnums::HeadingOption::IGNORE;
    m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER;
    m_moveInfo.pathnamegains = m_pathGainsType;

    auto pose = m_chassis->GetPose();
    auto speed = m_chassis->GetChassisSpeeds();
    auto robotDrive = new RobotDrive(m_chassis);
    auto trajectoryDrivePathPlanner = new TrajectoryDrivePathPlanner(robotDrive);

    if (m_pathname == "DRIVE_TO_NOTE")
    {
        m_driveToNote = new DriveToNote(robotDrive, trajectoryDrivePathPlanner);
        m_driveToNote->Init(m_moveInfo);
        m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE;
        m_switchedToVisionDrive = true;
    }
    else
    {
        auto path = PathPlannerPath::fromPathFile(m_pathname);
        if (path.get() != nullptr)
        {
            if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kRed)
            {
                path = path.get()->flipPath();
            }

            m_trajectory = path.get()->getTrajectory(speed, pose.Rotation());
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("DrivePathPlanner"), string("Path not found"), m_pathname);
        }
    }

    auto endstate = m_trajectory.getEndState();
    m_finalPose = endstate.getTargetHolonomicPose();
    m_moveInfo.pathplannerTrajectory = m_trajectory;
    m_totalTrajectoryTime = m_trajectory.getTotalTime();
}
void DrivePathPlanner::Run()
{
    if (m_chassis != nullptr)
    {
        /*
         if (!m_switchedToVisionDrive)
        {
            auto info = DragonDriveTargetFinder::GetInstance()->GetPose(DragonVision::VISION_ELEMENT::NOTE);
            auto type = get<0>(info);
            if (type == DragonDriveTargetFinder::TARGET_INFO::VISION_BASED && m_visionAlignment == PrimitiveParams::VISION_ALIGNMENT::NOTE)
            {
                m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE;
                m_driveToNote->Init(m_moveInfo);
                m_switchedToVisionDrive = true;
            }
        }
        */
        m_chassis->Drive(m_moveInfo);
    }
}

bool DrivePathPlanner::IsDone()
{
    if (m_timer.get()->Get() > m_maxTime && m_timer.get()->Get().to<double>() > 0.0)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "why done", "Time Out");
        return true;
    }

    if (m_checkDriveToNote)
    {
        CheckForDriveToNote();
    }
    // TO DO Figure out how to be able to drive back don't just stop your trajectory (Cause next trajectory to also stop)
    /*
    else if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue)
    {
        return m_chassis->GetPose().X() >= (m_centerLine + m_offset);
    }
    else
    {
        return m_chassis->GetPose().X() <= (m_centerLine - m_offset);
    }*/
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "Switched To Vision Drive", m_switchedToVisionDrive);

    if (m_switchedToVisionDrive)
    {
        return m_driveToNote->IsDone();
    }
    auto *trajectoryDrive = dynamic_cast<TrajectoryDrivePathPlanner *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER));
    return trajectoryDrive != nullptr ? trajectoryDrive->IsDone() : false;
}

void DrivePathPlanner::CheckForDriveToNote()
{
    auto currentTime = m_timer.get()->Get();
    if (((currentTime) / m_totalTrajectoryTime >= m_percentageCompleteThreshold))
    {
        m_pathname = "DRIVE_TO_NOTE";
        InitMoveInfo();
    }
    else if (m_chassis != nullptr)
    {
        auto currentPose = m_chassis->GetPose();
        if (currentPose.Translation().Distance(m_finalPose.Translation()) < units::length::meter_t(1.0))
        {
            m_pathname = "DRIVE_TO_NOTE";
            InitMoveInfo();
        }
    }
}
