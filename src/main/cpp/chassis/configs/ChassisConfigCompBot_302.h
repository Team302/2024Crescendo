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
#include "chassis/configs/ChassisConfig.h"

#include "units/length.h"
#include "ctre/phoenix6/Pigeon2.hpp"

class ChassisConfigCompBot_302 : public ChassisConfig
{
public:
	ChassisConfigCompBot_302() = default;
	~ChassisConfigCompBot_302() = default;

protected:
	void DefinePigeon() override;
	void DefineChassis() override;

private:
	std::string m_canbusName = std::string("canivore");
	const int m_leftfrontdriveID = 10;
	const int m_leftfrontturnID = 11;
	const double m_leftfrontOffset = -0.18310546875;
	const bool m_leftfrontdriveInvert = false;
	const bool m_leftfrontturnInvert = true;
	const bool m_leftfrontcancoderInvert = false;

	const int m_leftbackdriveID = 14;
	const int m_leftbackturnID = 15;
	const double m_leftbackOffset = 0.332763671875;
	const bool m_leftbackdriveInvert = true;
	const bool m_leftbackturnInvert = false;
	const bool m_leftbackcancoderInvert = false;

	const int m_rightfrontdriveID = 12;
	const int m_rightfrontturnID = 13;
	const double m_rightfrontOffset = 0.40063476;
	const bool m_rightfrontdriveInvert = true;
	const bool m_rightfrontturnInvert = false;
	const bool m_rightfrontcancoderInvert = false;

	const int m_rightbackdriveID = 8;
	const int m_rightbackturnID = 9;
	const double m_rightbackOffset = 0.35571289;
	const bool m_rightbackdriveInvert = false;
	const bool m_rightbackturnInvert = true;
	const bool m_rightbackcancoderInvert = false;
};