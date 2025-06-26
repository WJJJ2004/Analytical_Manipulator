#ifndef ARM_CTRL__MAIN_NODE_HPP_
#define ARM_CTRL__MAIN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <arm_ctrl/msg/arm_ctrl_cmd.hpp>
#include <arm_ctrl/msg/arm_ctrl_flag.hpp>
#include <arm_ctrl/msg/arm_joint_angle.hpp>
#include <humanoid_interfaces/msg/master2_ik_msg.hpp>
#include <humanoid_interfaces/msg/imu_msg.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ik_module.hpp"
#include "trajectory_planner.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

class MainNode : public rclcpp::Node , public std::enable_shared_from_this<MainNode>
{
public:
  MainNode();

private:
  void imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  void cmdCallback(const arm_ctrl::msg::ArmCtrlCmd::SharedPtr msg);
  void flagCallback(const arm_ctrl::msg::ArmCtrlFlag::SharedPtr msg);
  void mainLoop();
  void publishJointCommands(const Eigen::Vector3d &q, bool is_target_on_right);

  // Subscribers
  rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<arm_ctrl::msg::ArmCtrlCmd>::SharedPtr cmd_sub_;
  rclcpp::Subscription<arm_ctrl::msg::ArmCtrlFlag>::SharedPtr flag_sub_;

  // Publishers
  rclcpp::Publisher<humanoid_interfaces::msg::Master2IkMsg>::SharedPtr ik_pub_;
  rclcpp::Publisher<arm_ctrl::msg::ArmJointAngle>::SharedPtr joint_angle_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // States
  bool is_master_ready_, imu_yaw_set_, ee_received_, trajectory_generated_;
  double imu_yaw_, initial_yaw_;
  double approach_distance_, duration_, dt_;
  geometry_msgs::msg::Point ee_target_;
  size_t trajectory_index_;
  int loop_squence = 0;

  // Modules
  std::unique_ptr<IKModule> ik_module_;
  TrajectoryPlanner planner_;
  std::vector<Eigen::Vector3d> trajectory_;
};

#endif  // ARM_CTRL__MAIN_NODE_HPP_
