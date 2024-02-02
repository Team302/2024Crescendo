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

// C++ Includes

// FRC Includes

// Team 302 Includes
#include "chassis/SwerveModuleAttributes.h"

// Third Party Includes

class SwerveModuleConstants
{
public:
    enum ModuleID
    {
        LEFT_FRONT,
        RIGHT_FRONT,
        LEFT_BACK,
        RIGHT_BACK
    };

    enum ModuleType
    {
        SDS_MK4_L1,
        SDS_MK4_L2,
        SDS_MK4_L3,
        SDS_MK4_L4,
        SDS_MK4_L1_COLSON,
        SDS_MK4_L2_COLSON,
        SDS_MK4_L3_COLSON,
        SDS_MK4_L4_COLSON,
        SDS_MK4I_L1,
        SDS_MK4I_L2,
        SDS_MK4I_L3,
        SDS_MK4I_L1_COLSON,
        SDS_MK4I_L2_COLSON,
        SDS_MK4I_L3_COLSON
    };

    SwerveModuleConstants() = delete;
    ~SwerveModuleConstants() = delete;

    static SwerveModuleAttributes GetSwerveModuleAttrs(ModuleType type);
};
