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

// FRC Includes
#include <frc/geometry/Pose2d.h>
#include <math.h>

// Team302 Includes
#include "auton/AutonGrid.h"

// Thirdparty includes
Autongrid *Autongrid::m_instance = nullptr;

Autongrid *Autongrid::GetInstance()
{
    if (Autongrid::m_instance == nullptr)
    {
        Autongrid::m_instance = new Autongrid();
    }
}

bool Autongrid::IsPoseInGrid(XGRID xgrid, YGRID ygrid, frc::Pose2d robotPose)
{
    auto x = static_cast<double>((xgrid));
    auto y = static_cast<double>((ygrid));

    return ((robotPose.X()) <= m_gridRes * (x) && (robotPose.X()) >= m_gridRes * (x - 1) && (robotPose.Y()) <= m_gridRes * (y) && (robotPose.Y()) >= m_gridRes * (y - 1));
}
bool Autongrid::IsPoseInZone(XGRID xgrid1, XGRID xgrid2, YGRID ygrid1, YGRID ygrid2, frc::Pose2d robotPose)
{
    auto x = static_cast<double>((xgrid1));
    auto y = static_cast<double>((ygrid1));
    auto x2 = static_cast<double>((xgrid2));
    auto y2 = static_cast<double>((ygrid2));

    return ((robotPose.X()) <= m_gridRes * (x2) && (robotPose.X()) >= m_gridRes * (x - 1) && (robotPose.Y()) <= m_gridRes * (y2) && (robotPose.Y()) >= m_gridRes * (y - 1));
}