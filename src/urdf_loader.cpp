#include <ad_kinematics/urdf_loader.h>

#include <stdexcept>

namespace ad_kinematics {
namespace urdf_loader {

std::shared_ptr<Joint> toJoint(const urdf::JointConstSharedPtr& urdf_joint, int q_index) {
  std::shared_ptr<Joint> joint;
  Transform<double> parent_transform = toTransform(urdf_joint->parent_to_joint_origin_transform);
  bool is_mimic = urdf_joint->mimic != nullptr;
  if (is_mimic) {
    q_index = -1;
  }
  switch (urdf_joint->type) {
    case urdf::Joint::FIXED:
      joint.reset(new FixedJoint(urdf_joint->name, parent_transform.translation));
      break;
    case urdf::Joint::REVOLUTE: {
      Eigen::Vector3d axis = toTranslation(urdf_joint->axis);
      joint.reset(new RevoluteJoint(urdf_joint->name, parent_transform.translation, parent_transform.rotation * axis, q_index,
                                    urdf_joint->limits->upper, urdf_joint->limits->lower));
      break;
      }
    case urdf::Joint::CONTINUOUS: {
      Eigen::Vector3d axis = toTranslation(urdf_joint->axis);
      joint.reset(new ContinuousJoint(urdf_joint->name, parent_transform.translation, parent_transform.rotation * axis, q_index));
      break;
    }
    default:
      throw std::invalid_argument("ad_kinematics: joint '" + urdf_joint->name + "' has a type other than fixed, revolute or continuous.");
  }

  if (is_mimic) {
    joint->setMimic(urdf_joint->mimic->joint_name, urdf_joint->mimic->multiplier, urdf_joint->mimic->offset);
  }

  return joint;
}

Transform<double> toTransform(const urdf::Pose& p) {
  return Transformd(toRotation(p.rotation), toTranslation(p.position));
}

Eigen::Quaterniond toRotation(const urdf::Rotation &r) {
  return Eigen::Quaterniond(r.w, r.x, r.y, r.z);
}

Eigen::Vector3d toTranslation(const urdf::Vector3& v) {
  return Eigen::Vector3d(v.x, v.y, v.z);
}

}
}
