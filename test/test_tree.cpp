#include <gtest/gtest.h>

#include <stdexcept>

#include <urdf_parser/urdf_parser.h>

#include "ad_kinematics/tree.h"

namespace {

// base -(revolute j1 about z, 1 m along x)-> l1 -(fixed, 0.5 m along x)-> tip
const char* kArm = R"(<robot name="arm">
  <link name="base"/><link name="l1"/><link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" velocity="1" effort="1"/>
  </joint>
  <joint name="fixed" type="fixed">
    <parent link="l1"/><child link="tip"/><origin xyz="0.5 0 0"/>
  </joint>
</robot>)";

const char* kSlider = R"(<robot name="slider">
  <link name="base"/><link name="carriage"/>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="carriage"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="1" velocity="1" effort="1"/>
  </joint>
</robot>)";

}

TEST(Tree, forwardKinematicsOfARevoluteAndAFixedJoint) {
  ad_kinematics::Tree tree(urdf::parseURDF(kArm));
  tree.disableTransformCache();
  ASSERT_EQ(1u, tree.getNumActiveJoints());
  EXPECT_EQ(std::vector<std::string>{"j1"}, tree.getActiveJointNames());

  // Rotating j1 by 90 deg swings the 0.5 m fixed offset from x to y
  ad_kinematics::Transformd tip = tree.computeTransform<double>("tip", {M_PI / 2});
  EXPECT_NEAR(1.0, tip.translation.x(), 1e-9);
  EXPECT_NEAR(0.5, tip.translation.y(), 1e-9);
  EXPECT_NEAR(0.0, tip.translation.z(), 1e-9);
}

TEST(Tree, rejectsUnsupportedJointTypes) {
  EXPECT_THROW(ad_kinematics::Tree(urdf::parseURDF(kSlider)), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
