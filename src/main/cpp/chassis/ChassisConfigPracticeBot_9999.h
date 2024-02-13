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
#include "chassis/ChassisConfig.h"

#include "units/length.h"
#include "ctre/phoenix6/Pigeon2.hpp"

class ChassisConfigPracticeBot_9999 : public ChassisConfig
{
public:
	ChassisConfigPracticeBot_9999() = default;
	~ChassisConfigPracticeBot_9999() = default;

protected:
	void DefinePigeon() override;
	void DefineChassis() override;

private:
	std::string m_canbusName = std::string("Canivore");
	const int m_leftfrontdriveID = 8;
	const int m_leftfrontturnID = 9;
	const double m_leftfrontOffset = 0.032470703125;
	const bool m_leftfrontdriveInvert = true;
	const bool m_leftfrontturnInvert = true;
	const bool m_leftfrontcancoderInvert = true;

	const int m_leftbackdriveID = 5;
	const int m_leftbackturnID = 2;
	const double m_leftbackOffset = 0.32568359375;
	const bool m_leftbackdriveInvert = true;
	const bool m_leftbackturnInvert = true;
	const bool m_leftbackcancoderInvert = true;

	const int m_rightfrontdriveID = 6;
	const int m_rightfrontturnID = 7;
	const double m_rightfrontOffset = -0.42919921875;
	const bool m_rightfrontdriveInvert = true;
	const bool m_rightfrontturnInvert = false;
	const bool m_rightfrontcancoderInvert = false;

	const int m_rightbackdriveID = 4;
	const int m_rightbackturnID = 3;
	const double m_rightbackOffset = -0.115478515625;
	const bool m_rightbackdriveInvert = false;
	const bool m_rightbackturnInvert = true;
	const bool m_rightbackcancoderInvert = true;
};