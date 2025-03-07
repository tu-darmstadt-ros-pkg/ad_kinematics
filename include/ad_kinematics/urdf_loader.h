#ifndef AD_KINEMATICS__URDF_LOADER_H
#define AD_KINEMATICS__URDF_LOADER_H

#include <rclcpp/rclcpp.hpp>
#include "ad_kinematics/transforms.h"
#include <urdf_parser/urdf_parser.h>
namespace ad_kinematics {
namespace urdf_loader {
  Transformd toTransform(const urdf::Pose& p);
  Eigen::Quaterniond toRotation(const urdf::Rotation& r);
  Eigen::Vector3d toTranslation(const urdf::Vector3& v);

  std::shared_ptr<Joint> toJoint(const urdf::JointConstSharedPtr& urdf_joint, int q_index=-1);
}
}

#endif
