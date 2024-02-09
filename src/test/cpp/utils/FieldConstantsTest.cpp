#include <gtest/gtest.h> //nevermind that intellisense doesn't seem to find this file
#include "utils/FieldConstants.h"

class FieldConstantsTest : public ::testing::Test
{
    protected:
    FieldConstants fieldConstants = FieldConstants.getInstance();
};

TEST_F(FieldConstantsTest, GetFieldElementTest)
{
    FIELD_ELEMENT element = FIELD_ELEMENT::BLUE_CENTER_STAGE;

    frc::Pose3d result = fieldConstants.GetFieldElement(element);
    EXPECT_DOUBLE_EQ(result.Get(X).to<double>(), 5.32);
    EXPECT_DOUBLE_EQ(result.Get(Y).to<double>(), 4.11);
    EXPECT_DOUBLE_EQ(result.Get(Yaw).to<double>(), 0.0);
    EXPECT_DOUBLE_EQ(result.Get(Rotation).GetDegrees().to<double>(), 0.0);
    EXPECT_DOUBLE_NE(result.Get(X).to<double>(), 4);
}