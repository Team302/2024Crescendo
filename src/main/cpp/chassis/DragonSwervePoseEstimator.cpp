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
#include <iostream>
#include <map>
#include <memory>
#include <cmath>

// FRC includes
#include "frc/DriverStation.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Transform2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "networktables/NetworkTableInstance.h"
#include "units/acceleration.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"
#include "frc/kinematics/SwerveModulePosition.h"

// Team 302 includes
#include "chassis/driveStates/FieldDrive.h"
#include "chassis/driveStates/HoldDrive.h"
#include "chassis/driveStates/RobotDrive.h"
#include "chassis/driveStates/StopDrive.h"
#include "chassis/driveStates/TrajectoryDrivePathPlanner.h"
#include "chassis/headingStates/FaceAmp.h"
#include "chassis/headingStates/FaceCenterStage.h"
#include "chassis/headingStates/FaceGamePiece.h"
#include "chassis/headingStates/FaceLeftStage.h"
#include "chassis/headingStates/FaceRightStage.h"
#include "chassis/headingStates/FaceSpeaker.h"
#include "chassis/headingStates/IgnoreHeading.h"
#include "chassis/headingStates/ISwerveDriveOrientation.h"
#include "chassis/headingStates/MaintainHeading.h"
#include "chassis/headingStates/SpecifiedHeading.h"
#include "chassis/SwerveChassis.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include "configs/RobotElementNames.h"
#include "utils/AngleUtils.h"
#include "utils/ConversionUtils.h"
#include "utils/FMSData.h"
#include "utils/logging/Logger.h"

// Third Party Includes
#include "ctre/phoenix6/Pigeon2.hpp"
