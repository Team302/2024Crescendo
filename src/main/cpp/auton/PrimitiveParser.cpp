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

#include <map>
#include <string>

#include <frc/Filesystem.h>

#include <auton/AutonSelector.h>
#include <auton/PrimitiveEnums.h>
#include <auton/PrimitiveParams.h>
#include <auton/PrimitiveParser.h>
#include <auton/drivePrimitives/IPrimitive.h>
#include "utils/logging/Logger.h"
#include <pugixml/pugixml.hpp>
using namespace std;
using namespace pugi;

PrimitiveParamsVector PrimitiveParser::ParseXML(string fulldirfile)
{

    PrimitiveParamsVector paramVector;
    auto hasError = false;

    // auto deployDir = frc::filesystem::GetDeployDirectory();
    // auto autonDir = deployDir + "/auton/";

    // string fulldirfile = autonDir;
    // fulldirfile += fileName;
    // initialize the xml string to enum maps
    map<string, PRIMITIVE_IDENTIFIER> primStringToEnumMap;
    primStringToEnumMap["DO_NOTHING"] = DO_NOTHING;
    primStringToEnumMap["HOLD_POSITION"] = HOLD_POSITION;
    primStringToEnumMap["DRIVE_PATH_PLANNER"] = DRIVE_PATH_PLANNER;
    primStringToEnumMap["RESET_POSITION"] = RESET_POSITION;
    primStringToEnumMap["RESET_POSITION_PATH_PLANNER"] = RESET_POSITION_PATH_PLANNER;
    primStringToEnumMap["VISION_ALIGN"] = VISION_ALIGN;

    map<string, ChassisOptionEnums::HeadingOption> headingOptionMap;
    headingOptionMap["MAINTAIN"] = ChassisOptionEnums::HeadingOption::MAINTAIN;
    headingOptionMap["SPECIFIED_ANGLE"] = ChassisOptionEnums::HeadingOption::SPECIFIED_ANGLE;
    headingOptionMap["FACE_GAME_PIECE"] = ChassisOptionEnums::HeadingOption::FACE_GAME_PIECE;
    headingOptionMap["IGNORE"] = ChassisOptionEnums::HeadingOption::IGNORE;
    headingOptionMap["FACE_SPEAKER"] = ChassisOptionEnums::HeadingOption::FACE_SPEAKER;
    headingOptionMap["FACE_AMP"] = ChassisOptionEnums::HeadingOption::FACE_AMP;
    headingOptionMap["FACE_LEFT_STAGE"] = ChassisOptionEnums::HeadingOption::FACE_LEFT_STAGE;
    headingOptionMap["FACE_RIGHT_STAGE"] = ChassisOptionEnums::HeadingOption::FACE_RIGHT_STAGE;
    headingOptionMap["FACE_CENTER_STAGE"] = ChassisOptionEnums::HeadingOption::FACE_CENTER_STAGE;

    xml_document doc;
    xml_parse_result result = doc.load_file(fulldirfile.c_str());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PrimitiveParser", "Original File", fulldirfile.c_str());
    if (!result)
    {
        auto deployDir = frc::filesystem::GetDeployDirectory();
        auto autonDir = deployDir + "/auton/";

        string updfulldirfile = autonDir;
        updfulldirfile += fulldirfile;

        result = doc.load_file(updfulldirfile.c_str());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PrimitiveParser", "updated File", updfulldirfile.c_str());
    }

    if (result)
    {
        xml_node auton = doc.root();
        for (xml_node node = auton.first_child(); node; node = node.next_sibling())
        {
            for (xml_node primitiveNode = node.first_child(); primitiveNode; primitiveNode = primitiveNode.next_sibling())
            {
                Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "PrimitiveParser", "node", primitiveNode.name());

                if (strcmp(primitiveNode.name(), "snippet") == 0)
                {
                    for (xml_attribute attr = primitiveNode.first_attribute(); attr; attr = attr.next_attribute())
                    {
                        if (strcmp(attr.name(), "file") == 0)
                        {
                            auto filename = string(attr.value());
                            auto snippetParams = ParseXML(filename);
                            if (!snippetParams.empty())
                            {
                                for (auto snippet : snippetParams)
                                {
                                    paramVector.emplace_back(snippet);
                                }
                                // paramVector.insert(paramVector.end(), snippetParams.begin(), snippetParams.end());
                            }
                            else
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("snippet had no params"), attr.value());
                                hasError = true;
                            }
                        }
                        else
                        {
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("snippet unknown attr"), attr.name());
                            hasError = true;
                        }
                    }
                }
                else if (strcmp(primitiveNode.name(), "primitive") == 0)
                {
                    auto primitiveType = UNKNOWN_PRIMITIVE;
                    units::time::second_t time = units::time::second_t(15.0);
                    auto distance = 0.0;
                    auto headingOption = ChassisOptionEnums::HeadingOption::MAINTAIN;
                    auto heading = 0.0;
                    std::string pathName;
                    // auto armstate = ArmStateMgr::ARM_STATE::HOLD_POSITION_ROTATE;
                    // auto extenderstate = ExtenderStateMgr::EXTENDER_STATE::HOLD_POSITION_EXTEND;
                    // auto intakestate = IntakeStateMgr::INTAKE_STATE::HOLD;
                    auto pipelineMode = DragonCamera::PIPELINE::UNKNOWN;

                    // @ADDMECH Initialize your mechanism state
                    for (xml_attribute attr = primitiveNode.first_attribute(); attr; attr = attr.next_attribute())
                    {
                        if (strcmp(attr.name(), "id") == 0)
                        {
                            auto paramStringToEnumItr = primStringToEnumMap.find(attr.value());
                            if (paramStringToEnumItr != primStringToEnumMap.end())
                            {
                                primitiveType = paramStringToEnumItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid id"), attr.value());
                                hasError = true;
                            }
                        }
                        else if (strcmp(attr.name(), "time") == 0)
                        {
                            time = units::time::second_t(attr.as_float());
                        }
                        else if (strcmp(attr.name(), "distance") == 0)
                        {
                            distance = attr.as_float();
                        }
                        else if (strcmp(attr.name(), "headingOption") == 0)
                        {
                            auto headingItr = headingOptionMap.find(attr.value());
                            if (headingItr != headingOptionMap.end())
                            {
                                headingOption = headingItr->second;
                            }
                            else
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid heading option"), attr.value());
                                hasError = true;
                            }
                        }
                        else if (strcmp(attr.name(), "heading") == 0)
                        {
                            heading = attr.as_float();
                        }
                        else if (strcmp(attr.name(), "pathname") == 0)
                        {
                            pathName = attr.value();
                        }
                        else if (strcmp(attr.name(), "pipeline") == 0)
                        {
                            if (strcmp(attr.value(), "UNKNOWN") == 0)
                            {
                                pipelineMode = DragonCamera::PIPELINE::UNKNOWN;
                            }
                            else if (strcmp(attr.value(), "OFF") == 0)
                            {
                                pipelineMode = DragonCamera::PIPELINE::OFF;
                            }
                            else if (strcmp(attr.value(), "APRIL_TAG") == 0)
                            {
                                pipelineMode = DragonCamera::PIPELINE::APRIL_TAG;
                            }
                            else if (strcmp(attr.value(), "MACHINE_LEARNING") == 0)
                            {
                                pipelineMode = DragonCamera::PIPELINE::MACHINE_LEARNING;
                            }
                            else if (strcmp(attr.value(), "COLOR_THRESHOLD") == 0)
                            {
                                pipelineMode = DragonCamera::PIPELINE::COLOR_THRESHOLD;
                            }

                            else
                            {
                                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("PrimitiveParser::ParseXML invalid pipeline mode"), attr.value());
                                hasError = true;
                            }
                        }

                        // @ADDMECH add case for your mechanism state to get the statemgr / state
                        else
                        {
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML invalid attribute"), attr.name());
                            hasError = true;
                        }
                    }
                    if (!hasError)
                    {
                        paramVector.emplace_back(new PrimitiveParams(primitiveType,
                                                                     time,
                                                                     distance,
                                                                     headingOption,
                                                                     heading,
                                                                     pathName,
                                                                     pipelineMode
                                                                     // @ADDMECH add parameter for your mechanism state
                                                                     // armstate,
                                                                     // extenderstate,
                                                                     // intakestate,
                                                                     ));
                    }
                    else
                    {
                        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML"), string("Has Error"));
                    }
                }
            }
        }
    }
    else
    {
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML error parsing file"), fileName);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("PrimitiveParser"), string("ParseXML error message"), result.description());
    }

    std::string path;
    for (auto itr = paramVector.rbegin(); itr != paramVector.rend(); ++itr)
    {
        auto param = *itr;
        if (param->GetID() == PRIMITIVE_IDENTIFIER::RESET_POSITION)
        {
            if (param->GetPathName().empty())
            {
                param->SetPathName(path);
            }
        }
    }
    auto slot = 0;
    for (auto param : paramVector)
    {
        string ntName = string("Primitive ") + to_string(slot);
        auto logger = Logger::GetLogger();
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Primitive ID"), to_string(param->GetID()));
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Time"), param->GetTime().to<double>());
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Distance"), param->GetDistance());
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Heading Option"), to_string(param->GetHeadingOption()));
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Heading"), param->GetHeading());
        logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("Path Name"), param->GetPathName());
        // @ADDMECH Log state data
        // logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("armstate"), param->GetArmState());
        // logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("extenderstate"), param->GetExtenderState());
        // logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("intakestate"), param->GetIntakeState());
        // logger->LogData(LOGGER_LEVEL::PRINT, ntName, string("PIPELINE_MODE"), param->GetIntakeState());
        slot++;
    }

    return paramVector;
}
