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
// FRC Includes
#include <frc/geometry/Pose2d.h>

// Team302 Includes

// Thirdparty includes

class AutonGrid
{
public:
    enum XGRID
    {
        NO_VALUE = -1,
        X_1,
        X_2,
        X_3,
        X_4,
        X_5,
        X_6,
        X_7,
        X_8,
        X_9,
        X_10,
        X_11,
        X_12,
        X_13,
        X_14,
        X_15,
        X_16,
        X_17,
        X_18,
        X_19,
        X_20,
        X_21,
        X_22,
        X_23,
        X_24,
        X_25,
        X_26,
        X_27,
        X_28,
        X_29,
        X_30,
        X_31,
        X_32,
        X_33,
        X_34,
        X_35,
        X_36,
        X_37,
        X_38,
        X_39,
        X_40,
        X_41,
        X_42,
        X_43,
        X_44,
        X_45,
        X_46,
        X_47,
        X_48,
        X_49,
        X_50,
        X_51,
        X_52,
        X_53,
        X_54,
        EXCEEDING_VALUE // X_1 to X1_54
    };

    enum YGRID
    {
        NONE = -1,
        Y_1,
        Y_2,
        Y_3,
        Y_4,
        Y_5,
        Y_6,
        Y_7,
        Y_8,
        Y_9,
        Y_10,
        Y_11,
        Y_12,
        Y_13,
        Y_14,
        Y_15,
        Y_16,
        Y_17,
        Y_18,
        Y_19,
        Y_20,
        Y_21,
        Y_22,
        Y_23,
        Y_24,
        Y_25,
        Y_26,
        Y_27,
        EXCEEDED

    };

    static AutonGrid *GetInstance();

    bool IsPoseInZone(XGRID xgrid1, XGRID xgrid2, YGRID ygrid1, YGRID ygrid2, frc::Pose2d robotPose);

private:
    AutonGrid() = default;
    ~AutonGrid() = default;
    static AutonGrid *m_instance;
    units::length::foot_t m_gridRes = units::length::foot_t(1.0);
};