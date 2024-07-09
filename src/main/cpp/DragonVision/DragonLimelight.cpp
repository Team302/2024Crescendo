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
#include "units/time.h"

// Team 302 includes
#include "DragonVision/DragonLimelight.h"
#include "utils/logging/Logger.h"
#include "DragonVision/DragonVision.h"

// Third Party Includes
#include "Limelight/LimelightHelpers.h"

/// TODO
/// Need to support DragonLimelight becoming a child of DragonCamera
/// Need to remove everything involving target height, should use apriltag field positions
/// Need to support new OriginFieldPosition function, look at limelight docs NetworkTables API

///-----------------------------------------------------------------------------------
/// Method:         DragonLimelight (constructor)
/// Description:    Create the object
///-----------------------------------------------------------------------------------
DragonLimelight::DragonLimelight(
    std::string networkTableName,           /// <I> networkTableName
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
                              m_networktable(nt::NetworkTableInstance::GetDefault().GetTable(std::string(networkTableName)))
{
    SetPipeline(initialPipeline);
    SetLEDMode(ledMode);
    SetCamMode(camMode);
    SetStreamMode(streamMode);
    ToggleSnapshot(snapMode);
    m_healthTimer = new frc::Timer();
}

bool DragonLimelight::HealthCheck()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {

        double currentHb = nt->GetNumber("hb", START_HB);
        // check if heartbeat has ever been set and network table is not default
        if (currentHb == START_HB)
        {
            return false;
        }
        else if (m_lastHeartbeat != currentHb)
        {
            m_lastHeartbeat = currentHb;
            m_healthTimer->Reset(); // reset when we see a new heartbeat
            m_healthTimer->Start();
            return true;
        }
        else if (m_healthTimer->Get().to<double>() < 0.5) // if we haven't seen a new heartbeat in 0.5 seconds
        {
            return true;
        }
    }
    return false;
}

/// @brief Assume that the current pipeline is AprilTag and that a target is detected
/// @return -1 if the network table cannot be found
std::optional<int> DragonLimelight::GetAprilTagID()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        double value = nt->GetNumber("tid", -1);
        int aprilTagInt = static_cast<int>(value + (value > 0 ? 0.5 : -0.5));
        if (aprilTagInt < 0)
        {
            return std::nullopt;
        }
        return aprilTagInt;
    }

    return std::nullopt;
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

        return VisionPose{frc::Pose3d{units::meter_t(redPosition[0]), units::meter_t(redPosition[1]), units::meter_t(redPosition[2]), rotation}, timestamp};
    }

    return std::nullopt;
}

/**
 * @brief Get the Blue Field Position object
 *
 */
std::optional<VisionPose> DragonLimelight::GetBlueFieldPosition()
{
    if (m_networktable.get() != nullptr)
    {
        auto topic = m_networktable.get()->GetDoubleArrayTopic("botpose_wpiblue");
        std::vector<double> position = topic.GetEntry(std::array<double, 7>{}).Get(); // default value is empty array

        units::time::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
        units::time::millisecond_t timestamp = currentTime - units::millisecond_t(position[6] / 1000.0);

        frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(position[3]), units::angle::degree_t(position[4]), units::angle::degree_t(position[5])};

        // frc::Rotation3d rotationToTarget = frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::math::atan2(units::meter_t(position[0]), units::meter_t(position[2]))); // roll pitch yaw

        return VisionPose{frc::Pose3d{units::meter_t(position[0]), units::meter_t(position[1]), units::meter_t(position[2]), rotation}, timestamp};
    }

    return std::nullopt;
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

        return VisionPose{frc::Pose3d{units::meter_t(position[0]), units::meter_t(position[1]), units::meter_t(position[2]), rotation}, timestamp};
    }

    return std::nullopt;
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

std::optional<units::angle::degree_t> DragonLimelight::GetTargetYaw()
{
    if (std::abs(GetCameraRoll().to<double>()) < 1.0)
    {
        return -1.0 * GetTx();
    }
    else if (std::abs(GetCameraRoll().to<double>() - 90.0) < 1.0)
    {
        return GetTy();
    }
    else if (std::abs(GetCameraRoll().to<double>() - 180.0) < 1.0)
    {
        return GetTx();
    }
    else if (std::abs(GetCameraRoll().to<double>() - 270.0) < 1.0)
    {
        return -1.0 * GetTy();
    }
    return GetTx();
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetYawRobotFrame()
{
    std::optional<units::length::inch_t> targetXdistance = EstimateTargetXDistance_RelToRobotCoords();
    std::optional<units::length::inch_t> targetYdistance = EstimateTargetYDistance_RelToRobotCoords();

    if (targetXdistance.has_value() && targetYdistance.has_value())
    {
        if (std::abs(targetXdistance.value().to<double>()) > 0)
        {
            return units::math::atan2(targetYdistance.value(), targetXdistance.value());
        }
        else
        {
            return units::angle::degree_t(0.0);
        }
    }

    return std::nullopt;
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetPitch()
{
    if (std::abs(GetCameraRoll().to<double>()) < 1.0)
    {
        return GetTy();
    }
    else if (std::abs(GetCameraRoll().to<double>() - 90.0) < 1.0)
    {
        return GetTx();
    }
    else if (std::abs(GetCameraRoll().to<double>() - 180.0) < 1.0)
    {
        return -1.0 * GetTy();
    }
    else if (std::abs(GetCameraRoll().to<double>() - 270.0) < 1.0)
    {
        return -1.0 * GetTx();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, std::string("DragonLimelight"), std::string("GetTargetVerticalOffset"), std::string("Invalid limelight rotation"));
    return GetTy();
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetPitchRobotFrame()
{
    std::optional<units::length::inch_t> targetXDistance = std::optional<units::length::inch_t>(EstimateTargetXDistance_RelToRobotCoords());
    std::optional<units::length::inch_t> targetZDistance = EstimateTargetZDistance_RelToRobotCoords();

    if (targetXDistance && targetZDistance)
    {
        units::angle::degree_t targetPitchToRobot = units::angle::degree_t(atan2(targetZDistance.value().to<double>(), targetXDistance.value().to<double>()));
        return targetPitchToRobot;
    }

    return std::nullopt;
}

std::optional<double> DragonLimelight::GetTargetArea()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->GetNumber("ta", 0.0);
    }

    return std::nullopt;
}

std::optional<units::angle::degree_t> DragonLimelight::GetTargetSkew()
{
    if (m_networktable != nullptr)
    {
        return units::angle::degree_t(m_networktable->GetNumber("ts", 0.0));
    }

    return std::nullopt;
}

/**
 * @brief Get the Pose object for the current location of the robot
 * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
 */
std::optional<VisionPose> DragonLimelight::EstimatePoseOdometryLimelight(bool megatag2)
{
    // Megatag 1
    if (m_networktable.get() != nullptr)
    {
        // Megatag 1
        if (!megatag2)
        {
            nt::DoubleArrayTopic topic = m_networktable.get()->GetDoubleArrayTopic("botpose_wpiblue");
            std::vector<double> position = topic.GetEntry(std::array<double, 7>{}).Get(); // default value is empty array

            units::time::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
            units::time::millisecond_t timestamp = currentTime - units::millisecond_t(position[6] / 1000.0);

            frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(position[3]), units::angle::degree_t(position[4]), units::angle::degree_t(position[5])};
            frc::Pose3d pose3d = frc::Pose3d{units::meter_t(position[0]), units::meter_t(position[1]), units::meter_t(position[2]), rotation};

            double numberOfTagsDetected = position[7];
            double averageTagTargetArea = position[10];

            // in case of invalid Limelight targets
            if (pose3d.ToPose2d().X() == units::meter_t(0.0))
            {
                return std::nullopt;
            }

            double xyStds;
            double degStds;
            // multiple targets detected
            if (numberOfTagsDetected == 0)
            {
                return std::nullopt;
            }
            else if (numberOfTagsDetected >= 2)
            {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (averageTagTargetArea > 0.8)
            {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (averageTagTargetArea > 0.1)
            {
                xyStds = 2.0;
                degStds = 30;
            }
            // conditions don't match to add a vision measurement
            else
            {
                return std::nullopt;
            }
            VisionPose LimelightVisionPose = {pose3d, timestamp, {xyStds, xyStds, degStds}, PoseEstimationStrategy::MEGA_TAG};
            return LimelightVisionPose;
        }

        // Megatag 2
        else
        {
            LimelightHelpers::PoseEstimate poseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(m_cameraName);
            double xyStds;
            double degStds;
            // multiple targets detected
            if (poseEstimate.tagCount == 0)
            {
                return std::nullopt;
            }
            // conditions don't match to add a vision measurement
            else
            {
                xyStds = .7;
                degStds = 9999999;
            }
            VisionPose pose = {frc::Pose3d{poseEstimate.pose}, poseEstimate.timestampSeconds, {xyStds, xyStds, degStds}, PoseEstimationStrategy::MEGA_TAG_2};
            return pose;
        }
    }
    return std::nullopt;
}

std::optional<units::time::millisecond_t> DragonLimelight::GetPipelineLatency()
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::time::second_t(nt->GetNumber("tl", 0.0));
    }

    return std::nullopt;
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

units::length::inch_t DragonLimelight::CalcXTargetToRobot(units::angle::degree_t camPitch, units::length::inch_t mountHeight, units::length::inch_t camXOffset, units::angle::degree_t tY)
{
    units::length::inch_t XDistance = units::length::inch_t((units::math::tan(units::angle::degree_t(90) + camPitch + tY) * mountHeight) + units::math::abs(camXOffset));
    if (GetCameraYaw() > units::degree_t(std::abs(90.0)))
    {
        return -1.0 * (XDistance + m_driveThroughOffset);
    }
    return XDistance + m_driveThroughOffset;
}

units::length::inch_t DragonLimelight::CalcYTargetToRobot(units::angle::degree_t camYaw, units::length::inch_t xTargetDistance, units::length::inch_t camYOffset, units::length::inch_t camXOffset, units::angle::degree_t tX)
{
    units::length::inch_t yDistance = units::length::inch_t(0.0);
    if (GetCameraYaw() > units::degree_t(std::abs(90.0)))
        yDistance = units::length::inch_t((units::math::tan(tX + camYaw) * (xTargetDistance - camXOffset)) + camYOffset);
    else
        yDistance = units::length::inch_t((units::math::tan(tX + camYaw) * (xTargetDistance - camXOffset)) - camYOffset);

    return yDistance;
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetXDistance()
{
    units::length::meter_t mountingHeight = m_cameraPose.Z();

    units::angle::degree_t mountingAngle = m_cameraPose.Rotation().Y() * -1;
    std::optional<units::angle::degree_t> targetPitch = GetTargetPitch();
    std::optional<int> aprilTagID = GetAprilTagID();
    if (!aprilTagID)
    {
        if (targetPitch)
        {
            double tangent = units::math::tan(mountingAngle + targetPitch.value());
            if (tangent == 0)
            {
                return std::nullopt;
            }
            else
            {
                units::length::inch_t estimatedTargetDistance = (m_noteVerticalOffset - mountingHeight) / tangent;

                return estimatedTargetDistance;
            }
        }
    }
    else
    {
        auto botpose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");
        std::vector<double> xdistance = botpose.GetEntry(std::array<double, 6>{}).Get(); // default value is empty array

        return units::length::inch_t(xdistance[0]);
    }

    return std::nullopt;
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetYDistance()
{
    std::optional<int> aprilTagID = GetAprilTagID();
    std::optional<units::angle::degree_t> targetYaw = GetTargetYaw();
    std::optional<units::length::inch_t> targetXdistance = EstimateTargetXDistance();
    if (!aprilTagID && targetYaw && targetXdistance)
    {
        units::length::inch_t estimatedTargetDistance = targetXdistance.value() * units::math::tan(m_cameraPose.Rotation().Z() + targetYaw.value());
        return estimatedTargetDistance;
    }
    else if (aprilTagID)
    {
        auto botpose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");
        std::vector<double> xdistance = botpose.GetEntry(std::array<double, 6>{}).Get(); // default value is empty array

        return units::length::inch_t(xdistance[1]);
    }

    return std::nullopt;
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetZDistance()
{

    if (!GetAprilTagID())
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

    return std::nullopt;
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetXDistance_RelToRobotCoords()
{
    units::angle::degree_t camPitch = GetCameraPitch();
    units::length::inch_t mountHeight = GetMountingZOffset();
    units::length::inch_t camXOffset = GetMountingXOffset();
    units::angle::degree_t Ty = GetTargetPitch().value();
    return CalcXTargetToRobot(camPitch, mountHeight, camXOffset, Ty);
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetYDistance_RelToRobotCoords()
{

    units::angle::degree_t camYaw = GetCameraYaw();
    units::length::inch_t camYOffset = GetMountingYOffset();
    units::length::inch_t camXOffset = GetMountingXOffset();
    units::angle::degree_t Tx = GetTargetYaw().value();
    units::length::inch_t xTargetDistance = CalcXTargetToRobot(GetCameraPitch(), GetMountingZOffset(), GetMountingXOffset(), GetTargetPitch().value());

    return CalcYTargetToRobot(camYaw, xTargetDistance, camYOffset, camXOffset, Tx);
}

std::optional<units::length::inch_t> DragonLimelight::EstimateTargetZDistance_RelToRobotCoords()
{
    std::optional<units::length::inch_t> zDistance = EstimateTargetZDistance();
    if (zDistance)
    {
        units::length::inch_t targetZoffset_RF_inch = zDistance.value() + GetMountingZOffset(); ///< the offset is positive if the limelight is above the center of the robot

        return targetZoffset_RF_inch;
    }

    return std::nullopt;
}

std::optional<VisionData> DragonLimelight::GetDataToNearestAprilTag()
{
    std::optional<int> tagId = GetAprilTagID();
    if (tagId)
    {
        auto targetPose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");

        std::vector<double> vector = targetPose.GetEntry(std::array<double, 6>{}).Get();

        frc::Rotation3d rotation = frc::Rotation3d(units::angle::degree_t(vector[5]), units::angle::degree_t(vector[3]), units::angle::degree_t(vector[4]));
        auto transform = frc::Transform3d(units::length::meter_t(vector[0]), units::length::meter_t(vector[1]), units::length::meter_t(vector[2]), rotation);

        return VisionData{transform, transform.Translation(), rotation, tagId.value()};
    }

    return std::nullopt;
}

std::optional<VisionData> DragonLimelight::GetDataToSpecifiedTag(int id)
{
    std::optional<int> detectedTag = GetAprilTagID();
    if (detectedTag.has_value())
    {
        if (detectedTag.value() == id)
        {
            auto targetPose = m_networktable.get()->GetDoubleArrayTopic("targetpose_robotspace");

            std::vector<double> vector = targetPose.GetEntry(std::array<double, 6>{}).Get();

            // targetpose_robotspace: 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
            frc::Rotation3d rotationTransform = frc::Rotation3d(units::angle::degree_t(vector[5]), units::angle::degree_t(vector[3]), -units::angle::degree_t(vector[4]));
            auto transform = frc::Transform3d(units::length::meter_t(vector[0]), units::length::meter_t(vector[1]), units::length::meter_t(vector[2]), rotationTransform);

            frc::Rotation3d rotationToTarget = frc::Rotation3d(units::angle::degree_t(0.0), units::angle::degree_t(0.0), units::math::atan2(transform.X(), transform.Z())); // roll pitch yaw

            return VisionData{transform, transform.Translation(), rotationToTarget, detectedTag.value()};
        }
    }

    return std::nullopt;
}