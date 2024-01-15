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

// C++ Includes

// FRC Includes
#include <frc/trajectory/TrajectoryGenerator.h>

// Team 302 Includes
#include <chassis/swerve/driveStates/DragonTrajectoryGenerator.h>
#include <utils/WaypointXmlParser.h>
#include "utils/logging/Logger.h"
#include <utils/Waypoint2d.h>
#include <utils/FMSData.h>
#include <utils/DistanceBetweenPoses.h>

DragonTrajectoryGenerator::DragonTrajectoryGenerator(units::meters_per_second_t maxVelocity,
                                                     units::meters_per_second_squared_t maxAcceleration) : m_config(maxVelocity, maxAcceleration),
                                                                                                           m_fmsData(FMSData::GetInstance())
{
    // create map with enum and points parsed from xml
    WaypointXmlParser::GetInstance()->ParseWaypoints();
    std::vector<Waypoint2d> parsedWaypoints = WaypointXmlParser::GetInstance()->GetWaypoints();

    for (Waypoint2d waypoint : parsedWaypoints)
    {
        m_blueWaypoints.emplace(waypoint.waypointIdentifier, waypoint.bluePose);
        m_redWaypoints.emplace(waypoint.waypointIdentifier, waypoint.redPose);
    }
}

frc::Trajectory DragonTrajectoryGenerator::GenerateTrajectory(frc::Pose2d currentPose, TARGET_POSITION endPoint)
{
    std::vector<frc::Translation2d> intermediatePoints;

    WAYPOINTS endWaypoint;

    // check if we are going to grids
    if (endPoint != TARGET_POSITION::HUMAN_PLAYER_SUBSTATION)
    {
        double distToWallGrid = 0.0;
        double distToCoopGrid = 0.0;
        double distToHPGrid = 0.0;
        bool behindChargingPad = false;

        if (m_fmsData->GetAllianceColor() == frc::DriverStation::Alliance::kBlue)
        {
            distToWallGrid = DistanceBetweenPoses::GetDeltaBetweenPoses(currentPose, m_blueWaypoints[WAYPOINTS::GRID_WALL_COL_TWO]);
            distToCoopGrid = DistanceBetweenPoses::GetDeltaBetweenPoses(currentPose, m_blueWaypoints[WAYPOINTS::GRID_COOP_COL_TWO]);
            distToHPGrid = DistanceBetweenPoses::GetDeltaBetweenPoses(currentPose, m_blueWaypoints[WAYPOINTS::GRID_HP_COL_TWO]);
            behindChargingPad = currentPose.X() > m_blueWaypoints[WAYPOINTS::GRID_WALL_INTERMEDIATE].X() ? true : false;
        }
        else if (m_fmsData->GetAllianceColor() == frc::DriverStation::Alliance::kRed)
        {
            distToWallGrid = DistanceBetweenPoses::GetDeltaBetweenPoses(currentPose, m_redWaypoints[WAYPOINTS::GRID_WALL_COL_TWO]);
            distToCoopGrid = DistanceBetweenPoses::GetDeltaBetweenPoses(currentPose, m_redWaypoints[WAYPOINTS::GRID_COOP_COL_TWO]);
            distToHPGrid = DistanceBetweenPoses::GetDeltaBetweenPoses(currentPose, m_redWaypoints[WAYPOINTS::GRID_HP_COL_TWO]);
            behindChargingPad = currentPose.X() < m_redWaypoints[WAYPOINTS::GRID_WALL_INTERMEDIATE].X() ? true : false;
        }

        if (distToWallGrid < distToCoopGrid && distToWallGrid < distToHPGrid) // going to wall grid
        {
            switch (endPoint)
            {
            case TARGET_POSITION::COLUMN_ONE:
                endWaypoint = WAYPOINTS::GRID_WALL_COL_ONE;
                break;
            case TARGET_POSITION::COLUMN_TWO:
                endWaypoint = WAYPOINTS::GRID_WALL_COL_TWO;
                break;
            case TARGET_POSITION::COLUMN_THREE:
                endWaypoint = WAYPOINTS::GRID_WALL_COL_THREE;
                break;
            default:
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("Grid Wall"), std::string("Could not find target position"));
                break;
            }

            if (behindChargingPad)
            {
                intermediatePoints.emplace_back(frc::Translation2d{m_blueWaypoints[WAYPOINTS::GRID_WALL_INTERMEDIATE].X(),
                                                                   m_blueWaypoints[WAYPOINTS::GRID_WALL_INTERMEDIATE].Y()});
            }
        }
        else if (distToCoopGrid < distToWallGrid && distToCoopGrid < distToHPGrid)
        {
            switch (endPoint)
            {
            case TARGET_POSITION::COLUMN_ONE:
                endWaypoint = WAYPOINTS::GRID_COOP_COL_ONE;
                break;
            case TARGET_POSITION::COLUMN_TWO:
                endWaypoint = WAYPOINTS::GRID_COOP_COL_TWO;
                break;
            case TARGET_POSITION::COLUMN_THREE:
                endWaypoint = WAYPOINTS::GRID_COOP_COL_THREE;
                break;
            default:
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("Grid Coop"), std::string("Could not find target position"));
                break;
            }

            if (behindChargingPad)
            {
                intermediatePoints.emplace_back(frc::Translation2d{m_blueWaypoints[WAYPOINTS::GRID_COOP_INTERMEDIATE].X(),
                                                                   m_blueWaypoints[WAYPOINTS::GRID_COOP_INTERMEDIATE].Y()});
            }
        }
        else
        {
            switch (endPoint)
            {
            case TARGET_POSITION::COLUMN_ONE:
                endWaypoint = WAYPOINTS::GRID_HP_COL_ONE;
                break;
            case TARGET_POSITION::COLUMN_TWO:
                endWaypoint = WAYPOINTS::GRID_HP_COL_TWO;
                break;
            case TARGET_POSITION::COLUMN_THREE:
                endWaypoint = WAYPOINTS::GRID_HP_COL_THREE;
                break;
            default:
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("Grid HP"), std::string("Could not find target position"));
                break;
            }

            if (behindChargingPad)
            {
                intermediatePoints.emplace_back(frc::Translation2d{m_blueWaypoints[WAYPOINTS::GRID_HP_INTERMEDIATE].X(),
                                                                   m_blueWaypoints[WAYPOINTS::GRID_HP_INTERMEDIATE].Y()});
            }
        }
    }
    else // we are going to human player substation
    {
    }

    double distanceToFinalPoint = 0.0;

    frc::Trajectory resultingTrajectory;

    // if distance of the points is less that .1m away then return an empty trajectory
    if (m_fmsData->GetAllianceColor() == frc::DriverStation::Alliance::kBlue)
    {
        resultingTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(currentPose, intermediatePoints, m_blueWaypoints[endWaypoint], m_config);
        distanceToFinalPoint = DistanceBetweenPoses::GetDeltaBetweenPoses(currentPose, m_blueWaypoints[endWaypoint]);
    }
    else if (m_fmsData->GetAllianceColor() == frc::DriverStation::Alliance::kRed)
    {
        resultingTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(currentPose, intermediatePoints, m_redWaypoints[endWaypoint], m_config);
        distanceToFinalPoint = DistanceBetweenPoses::GetDeltaBetweenPoses(currentPose, m_redWaypoints[endWaypoint]);
    }

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, std::string("TrajectoryGenerator"), std::string("DistToFinalPoint"), std::to_string(distanceToFinalPoint));

    if (distanceToFinalPoint > 0.2)
    {
        return resultingTrajectory;
    }
    else
    {
        return frc::Trajectory();
    }
}