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
#include "units/length.h"

// Team 302 includes
#include "DragonVision/DragonLimelight.h"
#include "utils/logging/Logger.h"

// Third Party Includes

using namespace nt;
using namespace std;

/// TODO
/// Need to support DragonLimelight becoming a child of DragonCamera
/// Need to remove everything involving target height, should use apriltag field positions
/// Need to support new OriginFieldPosition function, look at limelight docs NetworkTables API

///-----------------------------------------------------------------------------------
/// Method:         DragonLimelight (constructor)
/// Description:    Create the object
///-----------------------------------------------------------------------------------
DragonLimelight::DragonLimelight(
    string tableName,                               /// <I> - network table name
    units::length::inch_t mountingHeight,           /// <I> - mounting height of the limelight
    units::length::inch_t mountingHorizontalOffset, /// <I> - mounting horizontal offset from the middle of the robot,
    units::length::inch_t forwardOffset,            /// <I> mounting offset forward/back
    units::angle::degree_t pitch,                   /// <I> - Pitch of limelight
    units::angle::degree_t yaw,                     /// <I> - Yaw of limelight
    units::angle::degree_t roll,                    /// <I> - Roll of limelight                /// <I> - clockwise rotation of limelight
    units::length::inch_t targetHeight,             /// <I> - height the target
    units::length::inch_t targetHeight2,            /// <I> - height of second target
    LED_MODE ledMode,
    CAM_MODE camMode,
    STREAM_MODE streamMode,
    SNAPSHOT_MODE snapMode) : m_networktable(NetworkTableInstance::GetDefault().GetTable(tableName.c_str())),
                              m_mountHeight(mountingHeight),
                              m_mountingHorizontalOffset(mountingHorizontalOffset),
                              m_mountingForwardOffset(forwardOffset),
                              m_yaw(yaw),
                              m_pitch(pitch),
                              m_roll(roll),
                              m_targetHeight(targetHeight),
                              m_targetHeight2(targetHeight2)
{
    SetPipeline(PIPELINE_MODE::OFF);
    SetLEDMode(ledMode);
    SetCamMode(camMode);
    SetStreamMode(streamMode);
    ToggleSnapshot(snapMode);
    SetLimelightPosition(mountingHeight,
                         mountingHorizontalOffset,
                         forwardOffset,
                         pitch,
                         yaw,
                         roll);
}

void DragonLimelight::SetLimelightPosition(units::length::inch_t mountHeight,
                                           units::length::inch_t mountHorizontalOffset,
                                           units::length::inch_t mountForwardOffset,
                                           units::angle::degree_t pitch,
                                           units::angle::degree_t yaw,
                                           units::angle::degree_t roll)
{
    m_mountHeight = mountHeight;
    m_mountingHorizontalOffset = mountHorizontalOffset;
    m_mountingForwardOffset = mountForwardOffset;
    m_pitch = pitch;
    m_yaw = yaw;
    m_roll = roll;
}

DragonLimelight::PIPELINE_MODE DragonLimelight::getPipeline() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return (PIPELINE_MODE)(nt->GetNumber("getpipe", PIPELINE_MODE::UNKNOWN));
    }
    return PIPELINE_MODE::UNKNOWN;
}

/// @brief Assume that the current pipeline is AprilTag and that a target is detected
/// @return -1 if the network table cannot be found
int DragonLimelight::getAprilTagID() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return round(nt->GetNumber("tid", -1));
    }

    return -1;
}

frc::Pose2d DragonLimelight::GetRedFieldPosition() const
{
    if (m_networktable.get() != nullptr)
    {
        auto blueTopic = m_networktable.get()->GetDoubleArrayTopic("botpose_wpiblue");

        std::vector<double> bluePosition = blueTopic.GetEntry(std::array<double, 7>{}).Get(); // default value is empty array

        // do we want to also store total latency (7th element in array) here?

        frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(bluePosition[3]), units::angle::degree_t(bluePosition[4]), units::angle::degree_t(bluePosition[5])};
        return frc::Pose3d{units::meter_t(bluePosition[0]), units::meter_t(bluePosition[1]), units::meter_t(bluePosition[2]), rotation}.ToPose2d();
    }
    else
    {
        return frc::Pose2d{};
    }
}

frc::Pose2d DragonLimelight::GetBlueFieldPosition() const
{
    if (m_networktable.get() != nullptr)
    {
        auto topic = m_networktable.get()->GetDoubleArrayTopic("botpose_wpiblue");
        std::vector<double> position = topic.GetEntry(std::array<double, 7>{}).Get(); // default value is empty array

        // do we want to also store total latency (7th element in array) here?

        frc::Rotation3d rotation = frc::Rotation3d{units::angle::degree_t(position[3]), units::angle::degree_t(position[4]), units::angle::degree_t(position[5])};
        return frc::Pose3d{units::meter_t(position[0]), units::meter_t(position[1]), units::meter_t(position[2]), rotation}.ToPose2d();
    }
    else
    {
        return frc::Pose2d{};
    }
}

std::vector<double> DragonLimelight::Get3DSolve() const
{
    std::vector<double> output;
    return output;
}

bool DragonLimelight::HasTarget() const
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
        // return units::angle::degree_t(nt->GetNumber("ty", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::angle::degree_t DragonLimelight::GetTargetHorizontalOffset() const
{
    if (abs(m_roll.to<double>()) < 1.0)
    {
        return -1.0 * GetTx();
    }
    else if (abs(m_roll.to<double>() - 90.0) < 1.0)
    {
        return GetTy();
    }
    else if (abs(m_roll.to<double>() - 180.0) < 1.0)
    {
        return GetTx();
    }
    else if (abs(m_roll.to<double>() - 270.0) < 1.0)
    {
        return -1.0 * GetTy();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonLimelight"), string("GetTargetVerticalOffset"), string("Invalid limelight rotation"));
    return GetTx();
}

units::angle::degree_t DragonLimelight::GetTargetHorizontalOffsetRobotFrame(units::length::inch_t *targetDistOffset_RF, units::length::inch_t *targetDistfromRobot_RF) const
{
    // Get the horizontal angle to the target and conver to radians
    units::angle::degree_t limelightFrameHorizAngle = GetTargetHorizontalOffset();
    units::angle::radian_t limelightFrameHorizAngleRad = limelightFrameHorizAngle;

    units::length::inch_t targetXdistance = EstimateTargetXdistance();

    units::length::inch_t targetHorizOffset = targetXdistance * tan(limelightFrameHorizAngleRad.to<double>());

    units::length::inch_t targetHorizOffsetRobotFrame = targetHorizOffset + m_mountingHorizontalOffset; // the offset is negative if the limelight is to the left of the center of the robot
    units::length::inch_t targetDistanceRobotFrame = targetXdistance + m_mountingForwardOffset;         // the offset is negative if the limelight is behind the center of the robot

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("targetHorizOffset_inch "), targetHorizOffset.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("targetHorizOffsetRobotFrame_inch "), targetHorizOffsetRobotFrame.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("targetDistanceRobotFrame_inch "), targetDistanceRobotFrame.to<double>());

    // units::angle::radian_t angleOffset = units::angle::radian_t(atan((targetHorizOffsetRobotFrame / targetDistanceRobotFrame).to<float>()));
    units::angle::radian_t angleOffset = units::angle::radian_t(0);

    *targetDistOffset_RF = targetHorizOffsetRobotFrame;
    *targetDistfromRobot_RF = targetDistanceRobotFrame;

    return angleOffset;
}

units::angle::degree_t DragonLimelight::GetTargetVerticalOffset() const
{
    if (abs(m_roll.to<double>()) < 1.0)
    {
        return GetTy();
    }
    else if (abs(m_roll.to<double>() - 90.0) < 1.0)
    {
        return GetTx();
    }
    else if (abs(m_roll.to<double>() - 180.0) < 1.0)
    {
        return -1.0 * GetTy();
    }
    else if (abs(m_roll.to<double>() - 270.0) < 1.0)
    {
        return -1.0 * GetTx();
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("DragonLimelight"), string("GetTargetVerticalOffset"), string("Invalid limelight rotation"));
    return GetTy();
}

double DragonLimelight::GetTargetArea() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->GetNumber("ta", 0.0);
    }
    return 0.0;
}

units::angle::degree_t DragonLimelight::GetTargetSkew() const
{
    //   auto nt = m_networktable.get();
    if (m_networktable != nullptr)
    {
        return units::angle::degree_t(m_networktable->GetNumber("ts", 0.0));
    }
    return units::angle::degree_t(0.0);
}

units::time::microsecond_t DragonLimelight::GetPipelineLatency() const
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return units::time::second_t(nt->GetNumber("tl", 0.0));
    }
    return units::time::second_t(0.0);
}

void DragonLimelight::SetTargetHeight(
    units::length::inch_t targetHeight)
{
    m_targetHeight = targetHeight;
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

bool DragonLimelight::SetPipeline(int pipeline)
{
    auto nt = m_networktable.get();
    if (nt != nullptr)
    {
        return nt->PutNumber("pipeline", pipeline);
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
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues HasTarget"), to_string(HasTarget()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues XOffset"), to_string(GetTargetHorizontalOffset().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues YOffset"), to_string(GetTargetVerticalOffset().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Area"), to_string(GetTargetArea()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Skew"), to_string(GetTargetSkew().to<double>()));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("PrintValues Latency"), to_string(GetPipelineLatency().to<double>()));
}

std::optional<units::length::inch_t> DragonLimelight::GetTargetHeight() const
{
    PIPELINE_MODE mode = getPipeline();
    if (mode == PIPELINE_MODE::APRIL_TAG)
    {
        return m_aprilTagInfo.GetHeight(getAprilTagID());
    }
    else if ((mode == PIPELINE_MODE::CUBE) || mode == PIPELINE_MODE::CONE)
    {
        return units::length::inch_t(5);
    }
    else if ((mode == PIPELINE_MODE::CONE_SUBSTATION) || (mode == PIPELINE_MODE::CUBE_SUBSTATION))
    {
        return units::length::inch_t(42);
    }

    return {};
}

units::length::inch_t DragonLimelight::EstimateTargetXdistance() const
{
    units::angle::degree_t totalAngleFromHorizontal; ///< the vertical angle from a horizontal datum to the target
    units::angle::degree_t limelightAngleFromHorizontal;
    units::angle::degree_t limelightRoll = GetLimelightRoll();

    // First determin the limelightAngleFromHorizontal depending on the mounting orientation
    if (abs(limelightRoll.to<double>()) < 1.0)
    {
        limelightAngleFromHorizontal = GetLimelightPitch();
    }
    else if (abs(limelightRoll.to<double>() - 90.0) < 1.0)
    {
        limelightAngleFromHorizontal = -1.0 * GetLimelightYaw();
    }
    else if (abs(limelightRoll.to<double>() - 180.0) < 1.0)
    {
        limelightAngleFromHorizontal = -1.0 * GetLimelightPitch();
    }
    else if (abs(limelightRoll.to<double>() - 270.0) < 1.0)
    {
        limelightAngleFromHorizontal = GetLimelightYaw();
    }
    else
    {
        // not handled. The only allowed Roll values are 0, 90, 180, 270
    }

    totalAngleFromHorizontal = (limelightAngleFromHorizontal + GetTargetVerticalOffset());

    units::angle::radian_t angleRad = totalAngleFromHorizontal; // angle in radians
    double tanOfAngle = tan(angleRad.to<double>());

    std::optional<units::length::inch_t> theTargetHeight = GetTargetHeight();

    if (theTargetHeight.has_value())
    {
        auto deltaHeight = theTargetHeight.value() - GetLimelightMountingHeight();
        units::length::inch_t x_distanceToTarget = units::length::inch_t(deltaHeight / tanOfAngle);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("totalAngleFromHorizontal "), totalAngleFromHorizontal.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("angleRad "), angleRad.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("limelightAngleFromHorizontal "), limelightAngleFromHorizontal.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("Limelight mounting angle "), GetLimelightPitch().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("target vertical angle "), GetTargetVerticalOffset().to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("deltaHeight "), deltaHeight.to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("tanOfAngle "), tanOfAngle);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("x_distanceToTarget "), (x_distanceToTarget).to<double>());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("targetHeight "), theTargetHeight.value().to<double>());

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("x_distanceToTarget_LL_inch "), x_distanceToTarget.to<double>());

        return x_distanceToTarget;
    }

    return units::length::inch_t(0);
}

units::length::inch_t DragonLimelight::EstimateTargetYdistance() const
{
    // Get the horizontal angle to the target and convert to radians
    units::angle::degree_t limelightFrameHorizAngle = GetTargetHorizontalOffset();
    units::angle::radian_t limelightFrameHorizAngleRad = limelightFrameHorizAngle;

    units::length::inch_t targetXdistance = EstimateTargetXdistance();

    units::length::inch_t targetYoffset = targetXdistance * tan(limelightFrameHorizAngleRad.to<double>()); // * -1 beacuse if the angle is positve, the distance is in the neg y direction

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("targetYoffset_LL_inch "), targetYoffset.to<double>());

    return targetYoffset;
}

units::length::inch_t DragonLimelight::EstimateTargetXdistance_RelToRobotCoords() const
{
    units::length::inch_t targetXoffset_RF_inch = EstimateTargetXdistance() + m_mountingForwardOffset; ///< the offset is negative if the limelight is behind the center of the robot

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("targetXoffset_RF_inch "), targetXoffset_RF_inch.to<double>());

    return targetXoffset_RF_inch;
}

units::length::inch_t DragonLimelight::EstimateTargetYdistance_RelToRobotCoords() const
{
    units::length::inch_t targetYoffset_RF_inch = EstimateTargetYdistance() + m_mountingHorizontalOffset; ///< the offset is negative if the limelight is to the left of the center of the robot

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("DragonLimelight"), string("targetYoffset_RF_inch "), targetYoffset_RF_inch.to<double>());

    return targetYoffset_RF_inch;
}
