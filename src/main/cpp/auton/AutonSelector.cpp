
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

// co-Author: notcharlie, creator of dumb code / copy paster of better code

// Includes
#include <string>
#include <vector>
#include <sys/stat.h>
#include <fstream>

#ifdef __linux
#include <dirent.h>
#endif

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Filesystem.h>
#include "networktables/NetworkTableInstance.h"

// Team302 includes
#include "auton/AutonSelector.h"
#include "utils/logging/Logger.h"
#include "utils/FMSData.h"

#include <pugixml/pugixml.hpp>

using namespace std;

//---------------------------------------------------------------------
// Method: 		<<constructor>>
// Description: This creates this object and reads the auto script (CSV)
//  			files and displays a list on the dashboard.
//---------------------------------------------------------------------
AutonSelector::AutonSelector()
{
	PutChoicesOnDashboard();
}

string AutonSelector::GetSelectedAutoFile()
{
	std::string autonfile(frc::filesystem::GetDeployDirectory());
	autonfile += std::string("/auton/");
	autonfile += GetAlianceColor();
	autonfile += GetStartPos();
	autonfile += GetNumofPiecesinauton();
	autonfile += GetParkOnChargeStation();
	autonfile += GetCube();
	autonfile += std::string(".xml");

	auto table = nt::NetworkTableInstance::GetDefault().GetTable("auton file");

	table.get()->PutString("determined name", autonfile);

	if (!FileExists(autonfile))
	{
		autonfile = frc::filesystem::GetDeployDirectory();
		autonfile += std::string("/auton/");
		autonfile += GetAlianceColor();
		autonfile += ("COOPThreeP.xml");

		table.get()->PutBoolean("File Exists", false);
	}
	else
	{
		table.get()->PutBoolean("File Exists", true);
	}

	table.get()->PutString("actual file", autonfile);

	return autonfile;
}

bool AutonSelector::FileExists(const std::string &name)
{

	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(name.c_str());
	if (result)
	{
		return true;
	}
	return false;
}

string AutonSelector::GetParkOnChargeStation()
{
	return m_chrgstatchooser.GetSelected();
}

string AutonSelector::GetAlianceColor()
{
	if (FMSData::GetInstance()->GetAllianceColor() == frc::DriverStation::Alliance::kRed)
	{
		return std::string("Red");
	}
	else
	{
		return std::string("Blue");
	}
}

string AutonSelector::GetStartPos()
{
	return m_startposchooser.GetSelected();
}

string AutonSelector::GetNumofPiecesinauton()
{
	return m_numofgamepiecechooser.GetSelected();
}

string AutonSelector::GetCube()
{
	return m_cubechooser.GetSelected();
}

//---------------------------------------------------------------------
// Method: 		PutChoicesOnDashboard
// Description: This puts the list of files in the m_csvFiles attribute
//				up on the dashboard for selection.
// Returns:		void
//---------------------------------------------------------------------
void AutonSelector::PutChoicesOnDashboard()
{

	m_startposchooser.AddOption("Gridcoop", "COOP");
	m_startposchooser.AddOption("Gridwall", "Wall");
	m_startposchooser.AddOption("Gridhp", "HP");
	frc::SmartDashboard::PutData("StartPos", &m_startposchooser);

	m_numofgamepiecechooser.AddOption("1", "One");
	m_numofgamepiecechooser.AddOption("2", "Two");
	m_numofgamepiecechooser.AddOption("3", "Three");
	m_numofgamepiecechooser.AddOption("4", "Four");
	frc::SmartDashboard::PutData("Numofpcs", &m_numofgamepiecechooser);

	m_chrgstatchooser.AddOption("yes", "P");
	m_chrgstatchooser.AddOption("no", "NP");
	frc::SmartDashboard::PutData("park on charg station", &m_chrgstatchooser);

	m_cubechooser.AddOption("Yes", "Cube");
	m_cubechooser.AddOption("No", "");
	frc::SmartDashboard::PutData("get cube", &m_cubechooser);
}