
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

#include <numbers>
#include <chassis/DragonTargetFinder.h>

using namespace std;

// in:
// out: Pose2d Field position of target center x,y,r(0_deg)
frc::Pose2d DragonTargetFinder::GetPosCenterTarget() const
{
    return PosCenterTarget;
}

// in: double x,double y - field center target position xy meters as double
// out:
void DragonTargetFinder::setPosCenterTarget(double x, double y)
{
    frc::Pose2d TempPose = frc::Pose2d(units::length::meter_t(x), units::length::meter_t(y), 0_deg);
    PosCenterTarget = TempPose;
}

// in: Current Position Pose2d
// out: Rotation2d Current rotation relative to field frame.
frc::Rotation2d DragonTargetFinder::GetCurrentRotaion(frc::Pose2d lCurPose)
{
    frc::Rotation2d CurrentRotaion = (lCurPose.Rotation()); // Current rotation pos in Sin Cos
    return CurrentRotaion;
}

// in: Current Position Pose2d
// out: Transform2d  robot distance from target X and Y.  R is mute due to target at 0_deg.
frc::Transform2d DragonTargetFinder::GetDistance2TargetXYR(frc::Pose2d lCurPose)
{
    frc::Transform2d Distance2Target = PosCenterTarget - lCurPose;
    return Distance2Target;
}
// in: Current Position Pose2d
// out: int field quadrant of robot current pose.  Relative to target and center robot.
int DragonTargetFinder::GetFieldQuadrant(frc::Pose2d lCurPose)
{
    //  What quadruarnt is the robot in based on center of target      +=Center Target
    //                  |
    //               II |   I
    //             -----+------
    //              III |   IV
    //                  |
    int i = 0;
    if (lCurPose.X() > PosCenterTarget.X() && lCurPose.Y() > PosCenterTarget.Y())
    {
        i = 1;
    } // -180 thru -90
    if (lCurPose.X() < PosCenterTarget.X() && lCurPose.Y() > PosCenterTarget.Y())
    {
        i = 2;
    } // 0 thru -90
    if (lCurPose.X() < PosCenterTarget.X() && lCurPose.Y() < PosCenterTarget.Y())
    {
        i = 3;
    } // 0 thru 90
    if (lCurPose.X() > PosCenterTarget.X() && lCurPose.Y() < PosCenterTarget.Y())
    {
        i = 4;
    } // 90  thru 180
    return i;
}

// in: Current Position Pose2d
// out: double Target angle relative to robots current rotation 0 to 180, -180 to 0
double DragonTargetFinder::GetAngle2Target(frc::Pose2d lCurPose)
{
    // return angle to target  180deg thru -180deg
    // 0 Degrees is pointing at center of target based on field position
    frc::Transform2d Distance2Target = GetDistance2TargetXYR(lCurPose);
    double dDistX2Target = Distance2Target.X().to<double>();
    double dDistY2Target = Distance2Target.Y().to<double>();
    // formulate a vector based on Distance to target
    double dHypotenuse = sqrt(dDistX2Target * dDistX2Target + dDistY2Target * dDistY2Target);
    // c = dhypotenuse
    // a= dDistY2Target
    // b= dDistX2Target
    // α = arcsin(a / c)
    // β = arcsin(b / c)

    // frc::Rotation2d Dist2TargetR = Distance2Target.Rotation();

    double dAngleARad = dDistY2Target / dHypotenuse;
    double dAngleAA = asin(dAngleARad);

    double dAngleBRad = dDistX2Target / dHypotenuse;
    double dAngleBB = asin(dAngleBRad);

    // Chassis Quadarant location based on radians to target.  ///////////////
    // int iQuadrantsLoc = 0; // Quadrants I,II,III,IV.  Standard radians rotation counter clockwise

    double dDeg2Target = (dAngleAA * (180.0 / numbers::pi)); // convert rad to degrees.
    double dDeg2TargetB = (dAngleBB * (180.0 / numbers::pi));

    if ((dAngleAA) < 0 && (dAngleBB > 0))
    {
        // iQuadrantsLoc = 1;
    } // neg quadraunt
    if ((dAngleAA) < 0 && (dAngleBB < 0))
    {
        // iQuadrantsLoc = 2;
        dDeg2Target = -90 + dDeg2TargetB;
    } // neg quadraunt
    if ((dAngleAA) > 0 && (dAngleBB < 0))
    {
        // iQuadrantsLoc = 3;
        dDeg2Target = 90 + abs(dDeg2TargetB);
    } // Pos quadraunt
    if ((dAngleAA) > 0 && (dAngleBB > 0))
    {
        // iQuadrantsLoc = 4;
    } // Pos quadraunt
    /////////////////////////////////////////////////////////////////////

    frc::SmartDashboard::PutNumber("dAngleAA", dAngleAA);
    frc::SmartDashboard::PutNumber("dAngleBB", dAngleBB);
    frc::SmartDashboard::PutNumber("dDeg2Target", dDeg2Target);
    frc::SmartDashboard::PutNumber("dDeg2TargetB", dDeg2TargetB);
    // Dist2TargetDEG

    return dDeg2Target;
}

// in: Current Position Pose2d
// out: double distance (meters) Field position, Center robot to center of target
double DragonTargetFinder::GetDistance2TargetHyp(frc::Pose2d lCurPose)
{
    // return distance to target straight line "Hypotenuse"
    frc::Transform2d Distance2Target = GetDistance2TargetXYR(lCurPose);
    double dDistX2Target = Distance2Target.X().to<double>();
    double dDistY2Target = Distance2Target.Y().to<double>();
    // formulate a vector based on Distance to target
    double dHypotenuse = sqrt(dDistX2Target * dDistX2Target + dDistY2Target * dDistY2Target);

    return dHypotenuse;
}

// in: Current Position Pose2d
// out: Target angle as double... Field angle robot center to center target
double DragonTargetFinder::GetTargetAngleD(frc::Pose2d lCurPose)
{
    frc::Rotation2d xCurRot2d = GetCurrentRotaion(lCurPose);
    double dCurDist2Zero_deg = units::angle::degree_t(xCurRot2d.Degrees()).to<double>(); //.to<double>();
    double dDeg2Target = GetAngle2Target(lCurPose);
    double dTargetAngle = dCurDist2Zero_deg + dDeg2Target;
    return dTargetAngle;
}

// in: Pose2d
// out: Target angle in Rotation 2d... Field angle robot center to center target
frc::Rotation2d DragonTargetFinder::GetTargetAngleR2d(frc::Pose2d lCurPose)
{
    frc::Rotation2d xCurRot2d = GetCurrentRotaion(lCurPose);
    double dCurDist2Zero_deg = units::angle::degree_t(xCurRot2d.Degrees()).to<double>(); //.to<double>();
    double dDeg2Target = GetAngle2Target(lCurPose);
    double dTargetAngle = dCurDist2Zero_deg + dDeg2Target;
    return units::angle::degree_t(dTargetAngle);
}
