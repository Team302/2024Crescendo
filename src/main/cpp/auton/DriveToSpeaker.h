#include <pathplanner/lib/path/PathPlannerPath.h>

using namespace pathplanner;

// Create a vector of bezier points from poses. Each pose represents one waypoint.
// The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
std::vector<frc::Pose2d> poses{
    frc::Pose2d(1.0_m, 1.0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(3.0_m, 1.0_m, frc::Rotation2d(0_deg)),
    frc::Pose2d(5.0_m, 3.0_m, frc::Rotation2d(90_deg))};
std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses(poses);

std::vector<frc::Translation2d> PathPlannerPath::bezierFromPoses(std::vector<frc::Pose2d> poses)
{
    pathplanner::PathPlannerPath(td::vector<frc::Translation2d> bezierPoints, PathConstraints constraints, GoalEndState goalEndState, bool reversed = false)
}

// Create the path using the bezier points created above
// We make a shared pointer here since the path following commands require a shared pointer
auto path = std::make_shared<PathPlannerPath>(
    bezierPoints,
    PathConstraints(3.0_mps, 3.0_mps_sq, 360_deg_per_s, 720_deg_per_s_sq), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
    GoalEndState(0.0_mps, frc::Rotation2d(-90_deg))                        // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
);

// Prevent the path from being flipped if the coordinates are already correct
path->preventFlipping = true;