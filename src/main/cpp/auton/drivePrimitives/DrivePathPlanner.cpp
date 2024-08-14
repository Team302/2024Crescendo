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
#include "auton/drivePrimitives/AutonUtils.h"
#include "auton/drivePrimitives/DrivePathPlanner.h"
#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"
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
                                       m_driveToNote(nullptr),
                                       m_timer(make_unique<Timer>()),
                                       m_trajectory(),
                                       m_pathname(),
                                       m_choreoTrajectoryName(),
                                       m_pathGainsType(ChassisOptionEnums::PathGainsType::LONG),
                                       // max velocity of 1 rotation per second and a max acceleration of 180 degrees per second squared.
                                       m_maxTime(units::time::second_t(-1.0)),
                                       m_ntName("DrivePathPlanner"),
                                       m_isVisionDrive(false),
                                       m_visionAlignment(PrimitiveParams::VISION_ALIGNMENT::UNKNOWN)

{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    m_chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;
    if (m_chassis != nullptr)
    {
        m_driveToNote = dynamic_cast<DriveToNote *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE));
    }
}

void DrivePathPlanner::Init(PrimitiveParams *params)
{
    m_pathname = params->GetPathName(); // Grabs path name from auton xml
    m_choreoTrajectoryName = params->GetTrajectoryName();
    m_pathGainsType = params->GetPathGainsType();

    m_ntName = string("DrivePathPlanner: ") + m_pathname;
    m_maxTime = params->GetTime();
    m_isVisionDrive = (m_pathname == "DRIVE_TO_NOTE");
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
    if (m_isVisionDrive)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "In DriveToNote", true);

        m_driveToNote = dynamic_cast<DriveToNote *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE));

        m_driveToNote->InitFromTrajectory(m_moveInfo, m_trajectory);
        m_moveInfo.driveOption = ChassisOptionEnums::DriveStateType::DRIVE_TO_NOTE;

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Original time", "Original time: ", m_maxTime.value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Added time", "Added time", m_moveInfo.pathplannerTrajectory.getTotalTime().value());

        m_maxTime += m_moveInfo.pathplannerTrajectory.getTotalTime();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Total time", "Total time", m_maxTime.value());
    }
    else
    {
        shared_ptr<PathPlannerPath> path;
        if (m_pathname.empty())
            path = AutonUtils::GetPathFromTrajectory(m_choreoTrajectoryName);
        else
            path = AutonUtils::GetPathFromPathFile(m_pathname);

        if (AutonUtils::IsValidPath(path))
        {
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
        m_chassis->Drive(m_moveInfo);
    }
}

bool DrivePathPlanner::IsDone()
{

    if (m_timer.get()->Get() > m_maxTime && m_timer.get()->Get().to<double>() > 0.0)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "why done", "Time Out");
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "time:", m_timer.get()->Get().value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "Max time:", m_maxTime.value());

        return true;
    }

    if (m_checkDriveToNote && !m_isVisionDrive)
    {
        CheckForDriveToNote();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "DrivePathPlanner", "Switched To Vision Drive", m_isVisionDrive);

    if (m_isVisionDrive)
    {
        return m_driveToNote->IsDone();
    }
    auto *trajectoryDrive = dynamic_cast<TrajectoryDrivePathPlanner *>(m_chassis->GetSpecifiedDriveState(ChassisOptionEnums::DriveStateType::TRAJECTORY_DRIVE_PLANNER));

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "Trajectory drive is done:", trajectoryDrive->IsDone());

    return trajectoryDrive != nullptr ? trajectoryDrive->IsDone() : false;
}

void DrivePathPlanner::CheckForDriveToNote()
{
    // Need to check if there is a note
    DragonDriveTargetFinder *dt = DragonDriveTargetFinder::GetInstance();
    auto noteInfo = dt->GetPose(DragonVision::NOTE);
    if (get<0>(noteInfo) != DragonDriveTargetFinder::NOT_FOUND) // see a note
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "Note Found: ", true);
        auto notePose = get<1>(noteInfo);

        // check if we see a note is one we want to get
        if (ShouldConsiderNote(notePose.X())) // chase this note: need to check if we should switch to drive to note or not
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "Consider Note: ", true);

            auto chassispose = m_chassis->GetPose();
            auto distanceToNote = chassispose.Translation().Distance(notePose.Translation());

            auto currentTime = m_timer.get()->Get();
            auto percent = currentTime.value() / m_totalTrajectoryTime.value();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "time:", currentTime.value());
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "Done Percent:", static_cast<double>((currentTime.value()) / m_totalTrajectoryTime.value()));
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "Distance: ", distanceToNote.value());

            if (percent >= m_percentageCompleteThreshold || distanceToNote <= m_distanceThreshold) // switch to drive to note
            {
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "Switch to Drive To Note: ", true);

                m_pathname = "DRIVE_TO_NOTE";
                m_isVisionDrive = true;
                m_visionAlignment = PrimitiveParams::VISION_ALIGNMENT::NOTE;
                m_trajectory = m_driveToNote->CreateDriveToNoteTrajectory(chassispose, notePose);
                InitMoveInfo();
            }
            else
            {
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "Switch to Drive To Note: ", false);
            }
        }
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "Consider Note: ", false);
        }
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "Distance To Note", "Note Found: ", false);
    }
}

bool DrivePathPlanner::ShouldConsiderNote(units::length::meter_t xposition)
{
    // check if note is one we want to get
    if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::kBlue)
    {
        return ((xposition <= (m_centerLine + m_offset)));
    }

    return ((xposition >= (m_centerLine - m_offset)));
}
