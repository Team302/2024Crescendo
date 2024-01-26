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
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
        M,
        N,
        O,
        P,
        Q,
        R,
        S,
        T,
        U,
        V,
        W,
        X,
        Y,
        Z,
        AA,
        EXCEEDING_VALUE
    };

    enum YGRID
    {
        NONE,
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX,
        SEVEN,
        EIGHT,
        NINE,
        TEN,
        ELEVEN,
        TWELVE,
        THIRTEEN,
        FOURTEEN,
        FIFTEEN,
        SIXTEEN,
        SEVENTEEN,
        EIGHTEEN,
        NINETEEN,
        TWENTY,
        TWENTYONE,
        TWENTYTWO,
        TWENTYTHREE,
        TWENTYFOUR,
        TWENTYFIVE,
        TWENTYSIX,
        TWENTYSEVEN,
        TWENTYEIGHT,
        TWENTYNINE,
        THIRTY,
        THIRTYONE,
        THIRTYTWO,
        THIRTYTHREE,
        THIRTYFOUR,
        THIRTYFIVE,
        THIRTYSIX,
        THIRTYSEVEN,
        THIRTYEIGHT,
        THIRTYNINE,
        FORTY,
        FORTYONE,
        FORTYTWO,
        FORTYTHREE,
        FORTYFOUR,
        FORTYFIVE,
        FORTYSIX,
        FORTYSEVEN,
        FORTYEIGHT,
        FORTYNINE,
        FIFTY,
        FIFTYONE,
        FIFTYTWO,
        FIFTYTHREE,
        FIFTYFOUR,
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