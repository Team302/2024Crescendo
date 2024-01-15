
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

// FRC includes
#include <frc/PowerDistribution.h>

/// @class PDPFactory
/// @brief This controls the creation of the PDP
class PDPFactory
{
public:
	/// @brief  Find or create the analog input factory
	/// @return PDPFactory* pointer to the factory
	static PDPFactory *GetFactory();

	frc::PowerDistribution *GetPDP() const { return m_pdp; };
	frc::PowerDistribution *CreatePDP(int canID, frc::PowerDistribution::ModuleType type);

private:
	PDPFactory();
	virtual ~PDPFactory() = default;

	frc::PowerDistribution *m_pdp;

	static PDPFactory *m_factory;
};