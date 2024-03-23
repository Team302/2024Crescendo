
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

#include "units/length.h"

#include "chassis/LaserCanPoseEstimator.h"
#include "chassis/ChassisConfig.h"
#include "chassis/ChassisConfigMgr.h"

LaserCanPoseEstimator::LaserCanPoseEstimator() : m_chassis(nullptr),
                                                 m_frontLeft(nullptr),
                                                 m_frontRight(nullptr)
{
    m_chassis = ChassisConfigMgr::GetInstance()->GetCurrentConfig()->GetSwerveChassis();
    if (m_chassis != nullptr)
    {
        // TODO: get LaserCans
    }
}

LaserCanPoseEstimator::~LaserCanPoseEstimator()
{
}

frc::Pose2d LaserCanPoseEstimator::GetPose()
{
    frc::Pose2d pose{};
    units::length::millimeter_t frontLeftDist = units::length::millimeter_t(-1.0);
    units::length::millimeter_t frontRightDist = units::length::millimeter_t(-1.0);
    if (m_frontLeft != nullptr)
    {
        auto measurement = m_frontLeft->get_measurement();
        if (measurement.has_value() && measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT)
        {
            frontLeftDist = units::length::millimeter_t(measurement.value().distance_mm);
        }
    }

    if (m_frontRight != nullptr)
    {
        auto measurement = m_frontRight->get_measurement();
        if (measurement.has_value() && measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT)
        {
            frontRightDist = units::length::millimeter_t(measurement.value().distance_mm);
        }
    }

    if (m_chassis != nullptr)
    {
        auto chassisPose = m_chassis->GetPose();
    }
    return pose;
}
