#ifndef ARM_CTRL__IK_MODULE_HPP_
#define ARM_CTRL__IK_MODULE_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <optional>

class IKModule {
public:
  explicit IKModule(rclcpp::Node* node);
  std::optional<Eigen::Vector3d> computeIK(const Eigen::Vector3d& target_position, bool is_target_on_right = false);
  bool isReachable(const Eigen::Vector3d& target_position, bool is_target_on_right = false){ return computeIK(target_position, is_target_on_right).has_value(); }

private:
  Eigen::Vector3d offset;
  Eigen::Matrix3d R_base;

  double L1, L2, L3, D1, D2, D3;

  Eigen::Matrix4d computeDH(double alpha, double a, double d, double theta);
  Eigen::Matrix4d forwardKinematics(const Eigen::Vector3d& q);

  // Analytical Inverse Kinematics Functions
  std::vector<Eigen::Vector3d> generateIKCandidates(const Eigen::Vector3d& target);
  std::optional<Eigen::Vector3d> sortingIKCandidates(const std::vector<Eigen::Vector3d>& candidates, const Eigen::Vector3d& target);
};

#endif  // ARM_CTRL__IK_MODULE_HPP_
