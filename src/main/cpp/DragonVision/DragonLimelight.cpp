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
#include <string>
#include <vector>
#include <cmath>

// FRC includes
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/time.h"
#include "networktables/DoubleArrayTopic.h"
#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/Timer.h"
#include "units/length.h"

// Team 302 includes
#include "DragonVision/DragonLimelight.h"
#include "utils/logging/Logger.h"

// Third Party Includes

/// TODO
/// Need to support DragonLimelight becoming a child of DragonCamera
/// Need to remove everything involving target height, should use apriltag field positions
/// Need to support new OriginFieldPosition function, look at limelight docs NetworkTables API

///-----------------------------------------------------------------------------------
/// Method:         DragonLimelight (constructor)
/// Description:    Create the object
///-----------------------------------------------------------------------------------
DragonLimelight::DragonLimelight(
    std::string networkTableName,                /// <I> networkTableName
    DragonCamera::PIPELINE initialPipeline, /// <I> enum for pipeline
    units::length::inch_t mountingXOffset,  /// <I> x offset of cam from robot center (forward relative to robot)
    units::length::inch_t mountingYOffset,  /// <I> y offset of cam from robot center (left relative to robot)
    units::length::inch_t mountingZOffset,  /// <I> z offset of cam from robot center (up relative to robot)
    units::angle::degree_t pitch,           /// <I> - Pitch of camera
    units::angle::degree_t yaw,             /// <I> - Yaw of camera
    units::angle::degree_t roll,            /// <I> - Roll of camera
    LED_MODE ledMode,
    CAM_MODE camMode,
    STREAM_MODE streamMode,
    SNAPSHOT_MODE snapMode) : DragonCamera(networkTableName, initialPipeline, mountingXOffset, mountingYOffset, mountingZOffset, pitch, yaw, roll),
                              m_networktable(nt::NetworkTableInstance::GetDefault().GetTable(std::string("limelight-" + networkTableName)))
{
    SetPipeline(initialPipeline);
    SetLEDMode(ledMode);
    SetCamMode(camMode);
    SetStreamMode(streamMode);
    ToggleSnapshot(snapMode);
}

/// @brief Assume that the current pipeline is AprilTag and that a target is detected
/// @return -1 if the network table cannot be found
int DragonLimelight::GetAprilTagID()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return round(nt->GetNumber("tid", -1));
    }

    return -1;
}

std::optional<VisionPose> DragonLimelight::GetFieldPosition()
{
    return GetBlueFieldPosition();
}

std::optional<VisionPose> DragonLimelight::GetFieldPosition(frc::DriverStation::Alliance alliance)
{
    if (alliance == frc::DriverStation::Alliance::kRed)
        return GetRedFieldPosition();
    else
    {
        return GetBlueFieldPosition();
    }
}

std::optional<VisionPose> DragonLimelight::GetRedFieldPosition()
{
    if (m_networktable.get() != nullptr)
    {
        auto redTopic = m_networktable.get()->GetDoubleArrayTopic("botpose_wpired");

        std::vector<double> redPosition = redTopic.GetEntry(std::array<double, 7>{}).Get(); // default value is empty array

        units::time::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
        units::time::millisecond_t timestamp = currentTime - units::millisecond_t(redPosition[6] / 1000.0);

        frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(redPosition[3]), units::angle::degree_t(redPosition[4]), units::angle::degree_t(redPosition[5])};

        return std::make_optional(VisionPose{frc::Pose3d{units::meter_t(redPosition[0]), units::meter_t(redPosition[1]), units::meter_t(redPosition[2]), rotation}, timestamp});
    }
    else
    {
        return std::nullopt;
    }
}

std::optional<VisionPose> DragonLimelight::GetBlueFieldPosition()
{
    if (m_networktable.get() != nullptr)
    {
        auto topic = m_networktable.get()->GetDoubleArrayTopic("botpose_wpiblue");
        std::vector<double> position = topic.GetEntry(std::array<double, 7>{}).Get(); // default value is empty array

        units::time::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
        units::time::millisecond_t timestamp = currentTime - units::millisecond_t(position[6] / 1000.0);

        frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(position[3]), units::angle::degree_t(position[4]), units::angle::degree_t(position[5])};
        return std::make_optional(VisionPose{frc::Pose3d{units::meter_t(position[0]), units::meter_t(position[1]), units::meter_t(position[2]), rotation}, timestamp});
    }
    else
    {
        return std::nullopt;
    }
}

std::optional<VisionPose> DragonLimelight::GetOriginFieldPosition()
{
    if (m_networktable.get() != nullptr)
    {
        auto redTopic = m_networktable.get()->GetDoubleArrayTopic("botpose");

        std::vector<double> position = redTopic.GetEntry(std::array<double, 7>{}).Get(); // default value is empty array

        units::time::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
        units::time::millisecond_t timestamp = currentTime - units::millisecond_t(position[6] / 1000.0);

        frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(position[3]), units::angle::degree_t(position[4]), units::angle::degree_t(position[5])};

        return std::make_optional(VisionPose{frc::Pose3d{units::meter_t(position[0]), units::meter_t(position[1]), units::meter_t(position[2]), rotation}, timestamp});
    }
    else
    {
        return std::nullopt;
    }
}

std::vector<double> DragonLimelight::Get3DSolve()
{
    std::vector<double> output;
    return output;
}

bool DragonLimelight::HasTarget()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return (nt->GetNumber("tv", 0.0) > 0.1);
    }
    return false;
}

units::angle::degree_t DragonLimelight::GetTx() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::angle::degree_t(nt->GetNumber("tx", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::angle::degree_t DragonLimelight::GetTy() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        double v = nt->GetNumber("ty", 0.0);

        return units::angle::degree_t(v);
    }
    return units::angle::degree_t(0.0);
}

units::angle::degree_t DragonLimelight::GetTargetYaw()
{
    if (abs(GetCameraRoll().to<double>()) < 1.0)
    {
        return -1.0 * GetTx();
    }
    else if (abs(GetCameraRoll().to<double>() - 90.0) < 1.0)
    {
        return GetTy();
    }
    else if (abs(GetCameraRoll().to<double>() - 180.0) < 1.0)
    {
        return GetTx();
    }
    else if (abs(GetCameraRoll().to<double>() - 270.0) < 1.0)
    {
        return -1.0 * GetTy();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("DragonLimelight"), std::string("GetTargetVerticalOffset"), std::string("Invalid limelight rotation"));
    return GetTx();
}

units::angle::degree_t DragonLimelight::GetTargetYawRobotFrame()
{
    // Get the horizontal angle to the target and convert to radians
    units::angle::radian_t limelightFrameHorizAngleRad = GetTargetYaw();

    units::length::inch_t targetXdistance = EstimateTargetXDistance();

    units::length::inch_t targetHorizOffset = targetXdistance * tan(limelightFrameHorizAngleRad.to<double>());

    units::length::inch_t targetHorizOffsetRobotFrame = targetHorizOffset + GetMountingYOffset(); // the offset is positive if the limelight is to the left of the center of the robot
    units::length::inch_t targetDistanceRobotFrame = targetXdistance + GetMountingXOffset();      // the offset is negative if the limelight is behind the center of the robot

    units::angle::radian_t angleOffset = units::angle::radian_t(atan((targetHorizOffsetRobotFrame / targetDistanceRobotFrame).to<double>()));
    return angleOffset;
}

units::angle::degree_t DragonLimelight::GetTargetPitch()
{
    if (abs(GetCameraRoll().to<double>()) < 1.0)
    {
        return GetTy();
    }
    else if (abs(GetCameraRoll().to<double>() - 90.0) < 1.0)
    {
        return GetTx();
    }
    else if (abs(GetCameraRoll().to<double>() - 180.0) < 1.0)
    {
        return -1.0 * GetTy();
    }
    else if (abs(GetCameraRoll().to<double>() - 270.0) < 1.0)
    {
        return -1.0 * GetTx();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, std::string("DragonLimelight"), std::string("GetTargetVerticalOffset"), std::string("Invalid limelight rotation"));
    return GetTy();
}

units::angle::degree_t DragonLimelight::GetTargetPitchRobotFrame()
{
    units::length::inch_t targetXDistance = EstimateTargetXDistance_RelToRobotCoords();

    if (targetXDistance != units::length::inch_t(0.0))
    {

        units::length::inch_t targetZDistance = EstimateTargetZDistance_RelToRobotCoords();

        units::angle::degree_t targetPitchToRobot = units::angle::degree_t(atan2(targetZDistance.to<double>(), targetXDistance.to<double>()));
        return targetPitchToRobot;
    }

    return units::angle::degree_t(0.0);
}

double DragonLimelight::GetTargetArea()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->GetNumber("ta", 0.0);
    }
    return 0.0;
}

units::angle::degree_t DragonLimelight::GetTargetSkew()
{
    if (m_networktable != nullptr)
    {
        return units::angle::degree_t(m_networktable->GetNumber("ts", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::time::millisecond_t DragonLimelight::GetPipelineLatency()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::time::second_t(nt->GetNumber("tl", 0.0));
    }
    return units::time::second_t(0.0);
}

void DragonLimelight::SetLEDMode(DragonLimelight::LED_MODE mode)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("ledMode", mode);
    }
}

void DragonLimelight::SetCamMode(DragonLimelight::CAM_MODE mode)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("camMode", mode);
    }
}

bool DragonLimelight::UpdatePipeline()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->PutNumber("pipeline", m_pipeline);
    }
    return false;
}

void DragonLimelight::SetStreamMode(DragonLimelight::STREAM_MODE mode)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("stream", mode);
    }
}

void DragonLimelight::SetCrosshairPos(double crosshairPosX, double crosshairPosY)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("cx0", crosshairPosX);
        nt->PutNumber("cy0", crosshairPosY);
    }
}

void DragonLimelight::SetSecondaryCrosshairPos(double crosshairPosX, double crosshairPosY)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("cx1", crosshairPosX);
        nt->PutNumber("cy1", crosshairPosY);
    }
}

// MAX of 32 snapshots can be saved
void DragonLimelight::ToggleSnapshot(DragonLimelight::SNAPSHOT_MODE toggle)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        nt->PutNumber("snapshot", toggle);
    }
}

void DragonLimelight::PrintValues()
{ /*
    Should do something similar but in DragonCamera instead of DragonLimelight

     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues HasTarget"), to_string(HasTarget()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues XOffset"), to_string(GetTargetHorizontalOffset().to<double>()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues YOffset"), to_string(GetTargetVerticalOffset().to<double>()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Area"), to_string(GetTargetArea()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Skew"), to_string(GetTargetSkew().to<double>()));
     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Latency"), to_string(GetPipelineLatency().to<double>()));
 */
}

units::length::inch_t DragonLimelight::EstimateTargetXDistance()
{
    units::length::meter_t mountingHeight = m_cameraPose.Z();

    units::angle::degree_t mountingAngle = m_cameraPose.Rotation().Y();

    if (GetAprilTagID() == -1)
    {
        // d=(h2-h1)/tan(a1+a2)
        units::length::inch_t estimatedTargetDistance = (m_noteVerticalOffset - mountingHeight) / units::math::tan(mountingAngle + GetTargetPitch());

        return estimatedTargetDistance;
    }

    else
    {
        auto botpose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");
        std::vector<double> xdistance = botpose.GetEntry(std::array<double, 6>{}).Get(); // default value is empty array

        return units::length::inch_t(xdistance[0]);
    }
}

units::length::inch_t DragonLimelight::EstimateTargetYDistance()
{
    if (GetAprilTagID() == -1)
    {
        units::length::inch_t estimatedTargetDistance = EstimateTargetXDistance() * units::math::tan(m_cameraPose.Rotation().Z() + GetTargetYaw());
        return estimatedTargetDistance;
    }

    else
    {
        auto botpose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");
        std::vector<double> xdistance = botpose.GetEntry(std::array<double, 6>{}).Get(); // default value is empty array

        return units::length::inch_t(xdistance[1]);
    }
}

units::length::inch_t DragonLimelight::EstimateTargetZDistance()
{
    if (GetAprilTagID() == -1)
    {
        units::length::inch_t estimatedTargetZDistance = m_cameraPose.Z() - m_noteVerticalOffset;
        return estimatedTargetZDistance;
    }

    else
    {
        auto botpose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");
        std::vector<double> xdistance = botpose.GetEntry(std::array<double, 6>{}).Get(); // default value is empty array

        return units::length::inch_t(xdistance[2]);
    }
}

units::length::inch_t DragonLimelight::EstimateTargetXDistance_RelToRobotCoords()
{
    if (EstimateTargetXDistance().to<double>() != -1.0)
    {
        units::length::inch_t targetXoffset_RF_inch = EstimateTargetXDistance() + GetMountingXOffset(); ///< the offset is negative if the limelight is behind the center of the robot

        return targetXoffset_RF_inch;
    }
    else
        return units::length::inch_t(-1.0);
}

units::length::inch_t DragonLimelight::EstimateTargetYDistance_RelToRobotCoords()
{
    if (EstimateTargetYDistance().to<double>() != -1.0)
    {
        units::length::inch_t targetYoffset_RF_inch = EstimateTargetYDistance() + GetMountingYOffset(); ///< the offset is positive if the limelight is to the left of the center of the robot

        return targetYoffset_RF_inch;
    }
    else
        return units::length::inch_t(-1.0);
}

units::length::inch_t DragonLimelight::EstimateTargetZDistance_RelToRobotCoords()
{
    if (EstimateTargetZDistance().to<double>() != -1.0)
    {
        units::length::inch_t targetZoffset_RF_inch = EstimateTargetZDistance() + GetMountingZOffset(); ///< the offset is positive if the limelight is above the center of the robot

        return targetZoffset_RF_inch;
    }
    else
        return units::length::inch_t(-1.0);
}

std::optional<VisionData> DragonLimelight::GetDataToNearestAprilTag()
{
    if (GetAprilTagID() == -1)
    {
        return std::nullopt;
    }

    auto targetPose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");

    std::vector<double> vector = targetPose.GetEntry(std::array<double, 6>{}).Get();

    frc::Rotation3d rotation = frc::Rotation3d(units::angle::degree_t(vector[3]), units::angle::degree_t(vector[4]), units::angle::degree_t(vector[5]));
    auto transform = frc::Transform3d(units::length::meter_t(vector[0]), units::length::meter_t(vector[1]), units::length::meter_t(vector[2]), rotation);
    return std::make_optional(VisionData{transform, GetAprilTagID()});
}