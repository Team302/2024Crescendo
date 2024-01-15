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

#include <string>

#include "chassis/swerve/SwerveChassis.h"
#include "chassis/swerve/SwerveModule.h"
#include "chassis/swerve/SwerveChassisBuilder.h"

using std::string;

SwerveChassisBuilder::SwerveChassisBuilder() : m_modules(),
                                               m_wheelBase(units::length::inch_t(0.0)),
                                               m_wheelTrack(units::length::inch_t(0.0)),
                                               m_maxSpeed(units::velocity::meters_per_second_t(0.0)),
                                               m_maxAngularSpeed(units::radians_per_second_t(0.0)),
                                               m_maxAcceleration(units::acceleration::meters_per_second_squared_t(0.0)),
                                               m_maxAngularAcceleration(units::angular_acceleration::radians_per_second_squared_t(0.0)),
                                               m_networkTableName(string(""))
{
    m_modules = {nullptr, nullptr, nullptr, nullptr};
}

void SwerveChassisBuilder::SetNetworkTableName(string ntName)
{
    m_networkTableName = ntName;
}
void SwerveChassisBuilder::DefineGeometry(units::length::inch_t wheelBase,
                                          units::length::inch_t wheelTrack)
{
    m_wheelBase = wheelBase;
    m_wheelTrack = wheelTrack;
}

void SwerveChassisBuilder::DefineMaxSpeeds(units::velocity::meters_per_second_t maxSpeed,
                                           units::radians_per_second_t maxAngularSpeed)
{
    m_maxSpeed = maxSpeed;
    m_maxAngularSpeed = maxAngularSpeed;
}

void SwerveChassisBuilder::DefineMaxAccel(units::acceleration::meters_per_second_squared_t maxAcceleration,
                                          units::angular_acceleration::radians_per_second_squared_t maxAngularAcceleration)
{
    m_maxAcceleration = maxAcceleration;
    m_maxAngularAcceleration = maxAngularAcceleration;
}
bool SwerveChassisBuilder::IsValid()
{
    return (!m_networkTableName.empty() &&
            m_wheelBase.to<double>() > 0.0 &&
            m_wheelTrack.to<double>() > 0.0 &&
            m_maxAcceleration.to<double>() > 0.0 &&
            m_maxAngularAcceleration.to<double>() > 0.0 &&
            m_maxAngularSpeed.to<double>() > 0.0 &&
            m_maxSpeed.to<double>() > 0.0);
}

SwerveChassis *SwerveChassisBuilder::Commit()
{
    if (IsValid())
    {
        return new SwerveChassis(m_modules[0],
                                 m_modules[1],
                                 m_modules[2],
                                 m_modules[3],
                                 m_wheelBase,
                                 m_wheelTrack,
                                 m_maxSpeed,
                                 m_maxAngularSpeed,
                                 m_maxAcceleration,
                                 m_maxAngularAcceleration,
                                 m_networkTableName);
    }
    return nullptr;
}
