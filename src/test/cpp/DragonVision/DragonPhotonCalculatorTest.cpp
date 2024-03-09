#include <gtest/gtest.h> //nevermind that intellisense doesn't seem to find this file
#include "DragonVision/DragonVisionStructs.h"
#include "DragonVision/DragonPhotonCalculator.h"

class DragonPhotonCalculatorTest : public ::testing::Test
{
protected:
};

TEST_F(DragonPhotonCalculatorTest, TestZeroPoseNoResult)
{
    photon::PhotonPipelineResult result = photon::PhotonPipelineResult{}; //create an empty result
    frc::Pose3d cameraPose = frc::Pose3d{}; // create a default 
    frc::Transform3d robotCenterToCamTransform = frc::Transform3d{}; // create a default transform

    std::optional<VisionData> visionData = DragonPhotonCalculator::GetDataToNearestAprilTag(cameraPose, result);
    EXPECT_FALSE(visionData.has_value());

    std::optional<units::angle::degrees_t> angle = DragonPhotonCalculator::GetTargetPitchRobotFrame(cameraPose, result);
    EXPECT_FALSE(angle.has_value());

    std::optional<units::angle::degrees_t> angle2 = DragonPhotonCalculator::GetTargetYawRobotFrame(cameraPose, result);
    EXPECT_FALSE(angle2.has_value());

    std::optional<VisionData> visionData4 = DragonPhotonCalculator::EstimateTargetXDistance_RelToRobotCoords(robotCenterToCamTransform, result);
    EXPECT_FALSE(visionData4.has_value());

    std::optional<VisionData> visionData5 = DragonPhotonCalculator::EstimateTargetYDistance_RelToRobotCoords(robotCenterToCamTransform, result);
    EXPECT_FALSE(visionData5.has_value());

    std::optional<VisionData> visionData6 = DragonPhotonCalculator::EstimateTargetZDistance_RelToRobotCoords(robotCenterToCamTransform, result);
    EXPECT_FALSE(visionData6.has_value());

}

TEST_F(DragonPhotonCalculatorTest, TestGetDataToNearestAprilTag)
{
    photon::PhotonPipelineResult result = photon::PhotonPipelineResult{};
    frc::Pose3d cameraPose = frc::Pose3d{};
    
    // Set up the result with a target
    photon::PhotonTrackedTarget target;
    frc::Transform3d camToTargetTransform;
    // Set up camToTargetTransform and other necessary values
    
    // Call the function under test
    std::optional<VisionData> visionData = DragonPhotonCalculator::GetDataToNearestAprilTag(cameraPose, result);
    
    // Assert that the visionData has a value
    EXPECT_TRUE(visionData.has_value());
    
    // Assert other expectations about the visionData values
    EXPECT_EQ(visionData.value().tagId, target.GetFiducialId());
    EXPECT_DOUBLE_EQ(visionData.value().transformToTarget.X().to<double>(), 0);
    EXPECT_DOUBLE_EQ(visionData.value().transformToTarget.Y().to<double>(), 0);
    EXPECT_DOUBLE_EQ(visionData.value().transformToTarget.Z().to<double>(), 0);
}