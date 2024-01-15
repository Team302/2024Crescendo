
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

#include <frc/geometry/Pose2d.h>  
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#pragma once

class DragonTargetFinder
{
    public:
        //  in: Current Position Pose2d
        //  out: Rotation2d Current rotation relative to field frame.
        frc::Rotation2d GetCurrentRotaion(frc::Pose2d);

        //   in: Current Position Pose2d
        //   out: Transform2d robot distance from target X and Y. R is mute due to target at 0_deg.
        frc::Transform2d GetDistance2TargetXYR(frc::Pose2d);

        //  in: Current Position Pose2d
        //  out: int field quadrant of robot current pose. Relative to target and center robot.
        int GetFieldQuadrant(frc::Pose2d);
    
        //   in:
        //   out: Pose2d Field position of target center x,y,r(0_deg)    
        frc::Pose2d GetPosCenterTarget() const; 
    
        //   in: Current Position Pose2d
        //   out: double Target angle relative to robots current rotation 0 to 180, -180 to 0    
        double GetAngle2Target(frc::Pose2d);


        //    in: Current Position Pose2d
        //    out: double distance (meters) Field position, Center robot to center of target
        double GetDistance2TargetHyp(frc::Pose2d);

    
        //    in: Current Position Pose2d
        //    out: Target angle as double... Field angle robot center to center target    
        double GetTargetAngleD(frc::Pose2d);

        //    in: Pose2d
        //    out: Target angle in Rotation 2d... Field angle robot center to center target
        frc::Rotation2d GetTargetAngleR2d(frc::Pose2d);
    

        //   in: double x,double y - field center target position xy meters as double
        //   out:
        void setPosCenterTarget(double x, double y);


    private:
      frc::Pose2d PosCenterTarget =  frc::Pose2d(8.212_m, 4.162_m,0_deg); //default

      
};