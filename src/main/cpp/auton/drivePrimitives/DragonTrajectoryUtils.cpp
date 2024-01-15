
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

#include <frc/Filesystem.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryUtil.h>

#include <auton/drivePrimitives/DragonTrajectoryUtils.h>
#include <auton/PrimitiveParams.h>

using frc::Trajectory;
using frc::TrajectoryUtil;

Trajectory DragonTrajectoryUtils::GetTrajectory(PrimitiveParams *params)
{
    auto path = params->GetPathName();
    if (!path.empty()) // only go if path name found
    {
        // Read path into trajectory for deploy directory.  JSON File ex. Bounce1.wpilib.json
        auto deployDir = frc::filesystem::GetDeployDirectory();
        deployDir += "/paths/output/" + path;

        return TrajectoryUtil::FromPathweaverJson(deployDir); // Creates a trajectory or path that can be used in the code, parsed from pathweaver json
    }
    return Trajectory();
}
