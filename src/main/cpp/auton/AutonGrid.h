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
        NO_VALUE,
        X_A,
        X_B,
        X_C,
        X_D,
        X_E,
        X_F,
        X_G,
        X_H,
        X_I,
        X_J,
        X_K,
        X_L,
        X_M,
        X_N,
        X_O,
        X_P,
        X_Q,
        X_R,
        X_S,
        X_T,
        X_U,
        X_V,
        X_W,
        X_X,
        X_Y,
        X_Z,
        X_AA,
        EXCEEDING_VALUE
    };

    enum YGRID
    {
        NONE,
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
        Y_28,
        Y_29,
        Y_30,
        Y_31,
        Y_32,
        Y_33,
        Y_34,
        Y_35,
        Y_36,
        Y_37,
        Y_38,
        Y_39,
        Y_40,
        Y_41,
        Y_42,
        Y_43,
        Y_44,
        Y_45,
        Y_46,
        Y_47,
        Y_48,
        Y_49,
        Y_50,
        Y_51,
        Y_52,
        Y_53,
        Y_54,
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