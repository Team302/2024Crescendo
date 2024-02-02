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
#pragma once

// C++ Includes
#include <memory>

// Team302 Includes
#include "auton/PrimitiveParams.h"
#include "auton/drivePrimitives/IPrimitive.h"
#include "configs/RobotConfig.h"
#include "configs/RobotConfigMgr.h"
#include "chassis/SwerveChassis.h"
#include "chassis/ChassisOptionEnums.h"
#include "DragonVision/DragonVision.h"

// FRC,WPI Includes
#include "frc/controller/HolonomicDriveController.h"
#include "frc/controller/RamseteController.h"
#include "frc/Filesystem.h"
#include "frc/geometry/Pose2d.h"
#include "frc/trajectory/TrajectoryConfig.h"
#include "frc/trajectory/TrajectoryUtil.h"
#include "wpi/SmallString.h"
#include "frc/Timer.h"
#include "units/time.h"

class VisionDrivePrimitive : public IPrimitive
{
public:
    VisionDrivePrimitive();

    virtual ~VisionDrivePrimitive() = default;

    void Init(PrimitiveParams *params) override;
    void Run() override;
    bool IsDone() override;

private:
    SwerveChassis *m_chassis;
    ChassisOptionEnums::HeadingOption m_headingOption;
    std::string m_ntName;
    DragonCamera::PIPELINE m_pipelineMode;

    frc::Timer *m_timer;
    units::time::second_t m_timeout;

    DragonVision *m_vision;
};