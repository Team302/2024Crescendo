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

#include <string>

#include "chassis/swerve/SwerveChassis.h"
#include "chassis/swerve/SwerveModule.h"

class SwerveChassisBuilder
{
public:
    SwerveChassisBuilder();
    ~SwerveChassisBuilder() = default;

    void SetNetworkTableName(std::string ntName);

    void AddSwerveModules(SwerveModule *frontleft,
                          SwerveModule *frontright,
                          SwerveModule *backleft,
                          SwerveModule *backright);
    void DefineGeometry(units::length::inch_t wheelBase,
                        units::length::inch_t wheelTrack);
    void DefineMaxSpeeds(units::velocity::meters_per_second_t maxSpeed,
                         units::radians_per_second_t maxAngularSpeed);
    void DefineMaxAccel(units::acceleration::meters_per_second_squared_t maxAcceleration,
                        units::angular_acceleration::radians_per_second_squared_t maxAngularAcceleration);
    bool IsValid();
    SwerveChassis *Commit();

private:
    std::array<SwerveModule *, 4> m_modules;
    units::length::inch_t m_wheelBase;
    units::length::inch_t m_wheelTrack;
    units::velocity::meters_per_second_t m_maxSpeed;
    units::radians_per_second_t m_maxAngularSpeed;
    units::acceleration::meters_per_second_squared_t m_maxAcceleration;
    units::angular_acceleration::radians_per_second_squared_t m_maxAngularAcceleration;
    std::string m_networkTableName;
};