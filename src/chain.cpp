#include <ad_kinematics/chain.h>

#include <moveit/robot_model_loader/robot_model_loader.hpp>

namespace ad_kinematics {

Chain::Chain(const urdf::ModelInterfaceSharedPtr& urdf, const moveit::core::JointModelGroup *joint_group)
{
  init(urdf, joint_group);
}

unsigned int Chain::getNumActuatedJoints() {
  return num_actuated_joints_;
}

std::vector<int> Chain::getJointQIndices(const std::vector<std::string> &joint_names)
{
  std::vector<int> indices;
  for (const std::string& joint_name: joint_names) {
    bool found = false;
    for (const Link& link: chain_) {
      if (link.getParentJoint()->getName() == joint_name) {
        indices.push_back(link.getParentJoint()->getQIndex());
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_ERROR(rclcpp::get_logger("chain_logger"),"Could not find joint with name '%s'.", joint_name.c_str());
      indices.push_back(-1);
    }
  }
  return indices;
}

void Chain::init(const urdf::ModelInterfaceSharedPtr &urdf, const moveit::core::JointModelGroup* joint_group)
{
  num_actuated_joints_ = 0;
  buildChain(urdf->getRoot(), joint_group);
}

void Chain::buildChain(const urdf::LinkConstSharedPtr& root, const moveit::core::JointModelGroup* joint_group) {
  RCLCPP_INFO(rclcpp::get_logger("chain_logger"),"Parsing URDF");
  base_link_name_ = root->name;
  tip_link_name_ = base_link_name_;
  std::vector<const moveit::core::LinkModel*> link_models = joint_group->getLinkModels();

  urdf::LinkConstSharedPtr current_link = root;
  for (std::vector<const moveit::core::LinkModel*>::iterator it = link_models.begin(); it != link_models.end(); ++it) {
    std::string link_name = (*it)->getName();

    bool found = false;
    RCLCPP_INFO(rclcpp::get_logger("chain_logger"),"%s :", current_link->name.c_str());
    // TODO replace with 'std::find'
    for (std::vector<urdf::LinkSharedPtr>::const_iterator it_childs = current_link->child_links.begin();
         it_childs != current_link->child_links.end() && !found;
         ++it_childs) {

      RCLCPP_INFO(rclcpp::get_logger("chain_logger"), " --> %s", (*it_childs)->name.c_str());

      if ((*it_childs)->name == link_name) {
        addToChain(*it_childs);
        current_link = *it_childs;
        found = true;
      }
    }

    if (!found) {
      RCLCPP_WARN(rclcpp::get_logger("chain_logger"), "URDF Loader could not find link '%s'.", link_name.c_str());
    }

  }
  RCLCPP_INFO(rclcpp::get_logger("chain_logger"),"URDF parsing finished.");
}

bool Chain::addToChain(const urdf::LinkConstSharedPtr& urdf_link) {
  // q_index of joint matches current number of actuated joints
  // q_index parameter is only used, if joint is actually actuated
  std::shared_ptr<Joint> joint = urdf_loader::toJoint(urdf_link->parent_joint, getNumActuatedJoints());
  if (joint->isActuated()) {
    num_actuated_joints_++;
  }
//  RCLCPP_INFO(rclcpp::get_logger("chain_logger"),"Origin: " << joint->getOrigin() << ", Axis: " << joint->getAxis() << ", Pose(0): " << joint->pose(0.0).toString());
  Link link(urdf_link->name, urdf_loader::toTransform(urdf_link->parent_joint->parent_to_joint_origin_transform), joint);
  RCLCPP_INFO(rclcpp::get_logger("chain_logger"),"Adding link %s", urdf_link->name.c_str());
  chain_.push_back(link);
  tip_link_name_ = link.getName();
  return true;
}

std::string Chain::getTipLinkName() const
{
  return tip_link_name_;
}

std::vector<std::string> Chain::getJointNames() const
{
  std::vector<std::string> joint_names;
  for (const Link& link: chain_) {
    joint_names.push_back(link.getParentJoint()->getName());
  }
  return joint_names;
}

std::vector<std::string> Chain::getActuatedJointNames() const
{
  std::vector<std::string> joint_names;
  for (const Link& link: chain_) {
    if (link.getParentJoint()->isActuated()) {
      joint_names.push_back(link.getParentJoint()->getName());
    }
  }
  return joint_names;
}

std::string Chain::getBaseLinkName() const
{
  return base_link_name_;
}

std::vector<Link> Chain::getChain() const
{
  return chain_;
}

}
