
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

//========================================================================================================
/// StateDataXmlParser.cpp
//========================================================================================================
///
/// File Description:
///     Top-level XML parsing file for the state data.  This definition will construct objects to help
///     set the state targets and control modes.
///
///     This parsing leverages the 3rd party Open Source Pugixml library (https://pugixml.org/).
///
///     The state definition XML files are in:  /home/lvuser/config/states/XXX.xml where the XXX
///     is the mechanism name.
///
//========================================================================================================

// C++ Includes
#include <memory>
#include <string>
#include <cstring>

// FRC includes
#include <frc/Filesystem.h>

// Team 302 includes
#include "mechanisms/controllers/ControlData.h"
#include "mechanisms/controllers/MechanismTargetData.h"
#include "mechanisms/MechanismTypes.h"
#include "utils/logging/Logger.h"
#include <mechanisms/controllers/ControlDataXmlParser.h>
#include <mechanisms/controllers/MechanismTargetXmlParser.h>
#include <mechanisms/controllers/StateDataXmlParser.h>

// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace pugi;
using namespace std;

/// @brief      Parse a mechanismState.xml file
/// @param [in] MechanismTypes::MECHANISM_TYPE  - mechanism that the states are for
/// @return     state data
vector<MechanismTargetData *> StateDataXmlParser::ParseXML(MechanismTypes::MECHANISM_TYPE mechanism)
{
    // bool hasError = false;
    vector<MechanismTargetData *> targetDataVector;

    // set the file to parse
    auto filename = frc::filesystem::GetDeployDirectory();
    filename += string("/states/");
    /**
    auto mech = MechanismFactory::GetMechanismFactory()->GetMechanism(mechanism);
    if (mech == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("StateDataXmlParser"), string("ParseXML"), string("invalid mechanism"));
        hasError = true;
    }

    if (!hasError && mech != nullptr)
    {
        auto mechFile = mech->GetControlFileName();
        if (mechFile.empty())
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("StateDataXmlParser"), string("ParseXML"), string("mechanism without control file"));
            hasError = true;
        }

        if (!hasError)
        {
            // load the xml file into memory (parse it)
            filename += mechFile;
            xml_document doc;
            xml_parse_result result = doc.load_file(filename.c_str());

            // if it is good
            if (result)
            {
                unique_ptr<ControlDataXmlParser> controlDataXML = make_unique<ControlDataXmlParser>();
                unique_ptr<MechanismTargetXmlParser> mechanismTargetXML = make_unique<MechanismTargetXmlParser>();

                vector<ControlData *> controlDataVector;

                // get the root node <robot>
                xml_node parent = doc.root();
                for (xml_node node = parent.first_child(); node; node = node.next_sibling())
                {
                    // loop through the direct children of <robot> and call the appropriate parser
                    for (xml_node child = node.first_child(); child; child = child.next_sibling())
                    {
                        if (strcmp(child.name(), "controlData") == 0)
                        {
                            controlDataVector.push_back(controlDataXML.get()->ParseXML(child));
                        }
                        else if (strcmp(child.name(), "mechanismTarget") == 0)
                        {
                            targetDataVector.push_back(mechanismTargetXML.get()->ParseXML(child));
                        }
                        else
                        {
                            string msg = "unknown child ";
                            msg += child.name();
                            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("StateDataXmlParser"), string("ParseXML"), msg);
                        }
                    }
                }

                for (auto td : targetDataVector)
                {
                    td->Update(controlDataVector);
                }
            }
            else
            {
                string msg = "XML [";
                msg += filename;
                msg += "] parsed with errors, attr value: [";
                msg += doc.child("prototype").attribute("attr").value();
                msg += "]";
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("StateDataXmlParser"), string("ParseXML (1) "), msg);

                msg = "Error description: ";
                msg += result.description();
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("StateDataXmlParser"), string("ParseXML (2) "), msg);

                msg = "Error offset: ";
                msg += result.offset;
                msg += " error at ...";
                msg += filename;
                msg += result.offset;
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("StateDataXmlParser"), string("ParseXML (3) "), msg);
            }
        }
    }
    **/
    return targetDataVector;
}
