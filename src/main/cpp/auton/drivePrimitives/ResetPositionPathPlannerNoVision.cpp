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
#include <memory>
#include <string>

// Team 302 includes
#include "auton/drivePrimitives/AutonUtils.h"
#include "auton/drivePrimitives/IPrimitive.h"
#include "auton/drivePrimitives/ResetPositionPathPlannerNoVision.h"
#include "auton/PrimitiveParams.h"
#include "chassis/configs/ChassisConfig.h"
#include "chassis/configs/ChassisConfigMgr.h"
#include "chassis/SwerveChassis.h"
#include "utils/logging/Logger.h"
#include "utils/FMSData.h"

// Third Party Includes
#include "pathplanner/lib/path/PathPlannerPath.h"

using namespace std;
using namespace frc;
using namespace pathplanner;

ResetPositionPathPlannerNoVision::ResetPositionPathPlannerNoVision() : IPrimitive()
{
}

void ResetPositionPathPlannerNoVision::Init(PrimitiveParams *param)
{
    auto config = ChassisConfigMgr::GetInstance()->GetCurrentConfig();
    auto chassis = config != nullptr ? config->GetSwerveChassis() : nullptr;

    if (chassis != nullptr)
    {
        auto path = AutonUtils::GetPathFromPathFile(param->GetPathName());
        if (AutonUtils::IsValidPath(path))
        {
            auto initialPose = path.get()->getPreviewStartingHolonomicPose();
            chassis->SetYaw(initialPose.Rotation().Degrees());
            chassis->ResetPose(initialPose);
        }
    }
}

void ResetPositionPathPlannerNoVision::Run()
{
}

bool ResetPositionPathPlannerNoVision::IsDone()
{
    return true;
}