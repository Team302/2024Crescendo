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

#include "chassis/SwerveModule.h"
#include "chassis/SwerveModuleConstants.h"
#include "chassis/ChassisConfigChassis_9998.h"
#include "utils/logging/Logger.h"

using ctre::phoenix6::hardware::Pigeon2;
using std::string;

void ChassisConfigChassis_9998::DefinePigeon()
{
    m_pigeon2 = new Pigeon2(0, "Canivore");
}

void ChassisConfigChassis_9998::DefineChassis()
{

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineChassis"), string("arrived"));
    m_leftFrontModule = new SwerveModule(SwerveModuleConstants::ModuleID::LEFT_FRONT,
                                         SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                         m_leftfrontdriveID, m_leftsideInvert,
                                         m_leftfrontturnID, false,
                                         m_leftfrontturnID, m_leftfrontOffset);
    m_leftBackModule = new SwerveModule(SwerveModuleConstants::ModuleID::LEFT_BACK,
                                        SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                        m_leftbackdriveID, m_leftsideInvert,
                                        m_leftbackturnID, false,
                                        m_leftbackturnID, m_leftbackOffset);
    m_rightFrontModule = new SwerveModule(SwerveModuleConstants::ModuleID::RIGHT_FRONT,
                                          SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                          m_rightfrontdriveID, m_rightsideInvert,
                                          m_rightfrontturnID, false,
                                          m_rightfrontturnID, m_rightfrontOffset);
    m_rightBackModule = new SwerveModule(SwerveModuleConstants::ModuleID::RIGHT_BACK,
                                         SwerveModuleConstants::ModuleType::SDS_MK4I_L3_COLSON,
                                         m_rightbackdriveID, m_rightsideInvert,
                                         m_rightbackturnID, false,
                                         m_rightbackturnID, m_rightbackOffset);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineChassis"), string("module 4"));
    m_chassis = new SwerveChassis(m_leftFrontModule,
                                  m_rightFrontModule,
                                  m_leftBackModule,
                                  m_rightBackModule,
                                  m_pigeon2,
                                  m_base,
                                  m_track,
                                  string("SwerveChassis"));
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("ChassisConfigChassis_9998::DefineChassis"), string("end"));
}
