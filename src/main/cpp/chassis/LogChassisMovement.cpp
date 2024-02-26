
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
#include <vector>

#include "chassis/LogChassisMovement.h"
#include "utils/logging/Logger.h"

using std::string;

void LogChassisMovement::Print(ChassisMovement &moveinfo)
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("driveOption"), moveinfo.driveOption);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("vx"), moveinfo.chassisSpeeds.vx.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("vy"), moveinfo.chassisSpeeds.vy.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("omega"), moveinfo.chassisSpeeds.omega.to<double>());
    auto ppstates = moveinfo.pathplannerTrajectory.getStates();
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("trajectory"), ppstates.empty() ? string("false") : string("true"));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("centerOfRotationOffset X"), moveinfo.centerOfRotationOffset.X().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("centerOfRotationOffset Y"), moveinfo.centerOfRotationOffset.Y().to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("headingOption"), moveinfo.headingOption);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("noMovementOption"), moveinfo.noMovementOption);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("controllerType"), moveinfo.driveOption);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("yawAngle"), moveinfo.yawAngle.to<double>());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("checkTipping"), moveinfo.checkTipping ? string("true") : string("false"));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ChassiMovement"), string("tippingCorrection"), moveinfo.tippingCorrection);
}
