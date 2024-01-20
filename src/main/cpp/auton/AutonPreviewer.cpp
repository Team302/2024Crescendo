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
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>

// Team302 Includes
#include <auton/AutonPreviewer.h>
#include <auton/AutonSelector.h>

// Thirdparty includes
#include <pugixml/pugixml.hpp>

using namespace pugi;

AutonPreviewer::AutonPreviewer(CyclePrimitives *cyclePrims) : m_selector(cyclePrims->GetAutonSelector()),
                                                              m_prevChoice(""),
                                                              m_field(DragonField::GetInstance())
{
}

void AutonPreviewer::CheckCurrentAuton()
{
    std::string currentChoice = m_selector->GetSelectedAutoFile();
    if (currentChoice != m_prevChoice)
    {
        PopulateField();
        m_prevChoice = currentChoice;
    }
}

//
void AutonPreviewer::PopulateField()
{
    std::vector<frc::Trajectory> trajectories = GetTrajectories();
    m_field->ResetField();
    for (unsigned int i = 0; i < trajectories.size(); i++)
    {
        m_field->AddTrajectory("traj" + std::to_string(i), trajectories[i]);
    }
}

std::vector<frc::Trajectory> AutonPreviewer::GetTrajectories()
{
    std::string filename = m_selector->GetSelectedAutoFile();

    std::vector<std::string> trajectoryPaths;
    std::vector<frc::Trajectory> trajectories;

    auto autonFile = frc::filesystem::GetDeployDirectory() + "/auton/" + filename;

    // Parse the xml file to get all the trajectory paths
    xml_document doc;
    xml_parse_result result = doc.load_file(autonFile.c_str());

    if (result)
    {
        xml_node auton = doc.root();
        for (xml_node node = auton.first_child(); node; node = node.next_sibling())
        {
            for (xml_node primitiveNode = node.first_child(); primitiveNode; primitiveNode = primitiveNode.next_sibling())
            {
                if (strcmp(primitiveNode.name(), "primitive") == 0)
                {
                    for (xml_attribute attr = primitiveNode.first_attribute(); attr; attr = attr.next_attribute())
                    {
                        if (strcmp(attr.name(), "pathname") == 0)
                        {
                            trajectoryPaths.emplace_back(attr.value());
                        }
                    }
                }
            }
        }
    }

    // Now that we have the paths, convert from JSON to frc::Trajectory and
    auto pathDir = frc::filesystem::GetDeployDirectory() + "/paths/output/";

    for (std::string path : trajectoryPaths)
    {
        trajectories.emplace_back(frc::TrajectoryUtil::FromPathweaverJson(pathDir + path));
    }

    return trajectories;
}