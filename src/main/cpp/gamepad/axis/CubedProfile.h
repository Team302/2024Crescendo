

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

#include <gamepad/axis/IProfile.h>

//========================================================================================================
/// @class CubedProfile
/// @brief This applies a cubic profile to the input values.
//========================================================================================================
class CubedProfile : public IProfile
{
public:
    //==================================================================================
    /// @brief  Static singleton method to create the object
    /// @return CubedProfile*  Singleton cubed profile object
    //==================================================================================
    static CubedProfile *GetInstance();

    //==================================================================================
    /// @brief:    Apply the profile
    /// @param  double inputVal - value to be scaled (have profile applied to)
    //==================================================================================
    void ApplyProfile(
        double &inputVal) const override;

private:
    CubedProfile() = default;
    ~CubedProfile() = default;

    static CubedProfile *m_instance;
};
