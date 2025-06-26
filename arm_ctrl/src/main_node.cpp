#include "main_node.hpp"            // 클래스 정의
#include <memory>                   // std::make_shared
#include <functional>               // std::placeholders

using std::placeholders::_1;            // 콜백 바인딩용 placeholder
using namespace std;

MainNode::MainNode()
: Node("main_node"),
  is_master_ready_(false), imu_yaw_set_(false), ee_received_(false),
  trajectory_index_(0), trajectory_generated_(false),loop_squence(0)
{
  // 파라미터 선언
  this->declare_parameter<double>("L1", 100.00);
  this->declare_parameter<double>("L2", 100.00);
  this->declare_parameter<double>("L3", 100.00);
  this->declare_parameter<double>("D1", 100.00);
  this->declare_parameter<double>("D2", 100.00);
  this->declare_parameter<double>("D3", 100.00);
  this->declare_parameter<double>("approach_distance", 100.0);      // 접근 거리
  this->declare_parameter<double>("duration", 5.0);               // trajectory 생성 시간
  this->declare_parameter<double>("dt", 0.1);                     // trajectory 생성 시간 간격
  this->get_parameter("approach_distance", approach_distance_);
  this->get_parameter("duration", duration_);
  this->get_parameter("dt", dt_);

  // publishers
  ik_pub_ = this->create_publisher<humanoid_interfaces::msg::Master2IkMsg>(     // IK WALK로 보행 명령 퍼블리시
    "/master_to_ik", 10);
  joint_angle_pub_ = this->create_publisher<arm_ctrl::msg::ArmJointAngle>(
    "/joint_angle_cmd", 10);
  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/arm_controller/joint_trajectory", 10);                                     // Gazebo 시뮬레이터용 조인트 trajectory 퍼블리셔
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);                                                        // RViz 시각화용 조인트 상태 퍼블리셔

  // subscribers
  imu_sub_ = this->create_subscription<humanoid_interfaces::msg::ImuMsg>(
    "/imu_data", 10, std::bind(&MainNode::imuCallback, this, _1)); // IMU 데이터 받음
  cmd_sub_ = this->create_subscription<arm_ctrl::msg::ArmCtrlCmd>(
    "/arm_cmd", 10, std::bind(&MainNode::cmdCallback, this, _1));   // 비전 모듈에서 EE 좌표를 받음
  flag_sub_ = this->create_subscription<arm_ctrl::msg::ArmCtrlFlag>(
    "/arm_flag", 10, std::bind(&MainNode::flagCallback, this, _1)); // 마스터 준비 상태를 받음

  timer_ = this->create_wall_timer(100ms, std::bind(&MainNode::mainLoop, this));
  ik_module_ = std::make_unique<IKModule>(this);
  initial_yaw_ = 0.0;
}

void MainNode::imuCallback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
{
  imu_yaw_ = msg->yaw;
  if (!imu_yaw_set_) {
    initial_yaw_ = imu_yaw_;
    imu_yaw_set_ = true;
  }
}

void MainNode::cmdCallback(const arm_ctrl::msg::ArmCtrlCmd::SharedPtr msg)
{
  ee_target_.x = msg->x;
  ee_target_.y = msg->y;
  ee_target_.z = msg->z;

  ee_received_ = true;
}

void MainNode::flagCallback(const arm_ctrl::msg::ArmCtrlFlag::SharedPtr msg)
{
  is_master_ready_ = msg->is_master_ready;
}

void MainNode::publishJointCommands(const Eigen::Vector3d &q, bool is_target_on_right)
{
  arm_ctrl::msg::ArmJointAngle angle_msg;
  // Gazebo 시뮬레이터용 조인트 각도 메시지
  trajectory_msgs::msg::JointTrajectory gazebo_msg;
  // RViz 시각화용 메시지
  sensor_msgs::msg::JointState rviz_msg;
  rviz_msg.header.stamp = this->get_clock()->now();

  // 조인트 이름 지정
  rviz_msg.name = {
    "rotate_0", "rotate_1", "rotate_2", "rotate_3", "rotate_4", "rotate_5",
    "rotate_6", "rotate_7", "rotate_8", "rotate_9", "rotate_10", "rotate_11",
    "rotate_12", "rotate_13", "rotate_14", "rotate_15", "rotate_16", "rotate_17",
    "rotate_18", "rotate_19", "rotate_20", "rotate_21"
  };

  gazebo_msg.joint_names = rviz_msg.name;

  // Trajectory 메시지 초기화
  trajectory_msgs::msg::JointTrajectoryPoint traj_point;
  traj_point.positions.resize(22, 0.0);

  // 조인트 각도 설정
  if (is_target_on_right) {
    angle_msg.rotate_0 = -1.0 * q(0);
    angle_msg.rotate_2 = -1.0 * q(1);
    angle_msg.rotate_4 = -1.0 * q(2);

    angle_msg.rotate_1 = angle_msg.rotate_3 = angle_msg.rotate_5 = 0.0;

    traj_point.positions[0] = angle_msg.rotate_0;
    traj_point.positions[2] = angle_msg.rotate_2;
    traj_point.positions[4] = angle_msg.rotate_4;
  } else {
    angle_msg.rotate_0 = angle_msg.rotate_2 = angle_msg.rotate_4 = 0.0;

    angle_msg.rotate_1 = q(0);
    angle_msg.rotate_3 = q(1);
    angle_msg.rotate_5 = q(2);

    traj_point.positions[1] = angle_msg.rotate_1;
    traj_point.positions[3] = angle_msg.rotate_3;
    traj_point.positions[5] = angle_msg.rotate_5;
  }

  traj_point.time_from_start = rclcpp::Duration::from_seconds(0.1);
  gazebo_msg.points.push_back(traj_point);

  // rviz 메시지에 position 채우기
  rviz_msg.position = traj_point.positions;

  // 메시지 퍼블리시
  joint_angle_pub_->publish(angle_msg);
  traj_pub_->publish(gazebo_msg);
  joint_state_pub_->publish(rviz_msg);
}


void MainNode::mainLoop()
{
  // if (!is_master_ready_ || !ee_received_ || !imu_yaw_set_)
  // {
  //   if(!is_master_ready_) {
  //     RCLCPP_INFO(this->get_logger(), "Waiting for master ready...");
  //     return;
  //   }
  //   else if(!ee_received_) {
  //     RCLCPP_INFO(this->get_logger(), "Waiting for EE target...");
  //     return;
  //   }
  //   else if(!imu_yaw_set_) {
  //     RCLCPP_INFO(this->get_logger(), "Waiting for IMU data...");
  //     return;
  //   }
  // }

  if(!ee_received_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for EE target...");
      return;
    }

  Eigen::Vector3d current_target(ee_target_.x, ee_target_.y, ee_target_.z);
  bool is_target_on_right = (ee_target_.y < 0);

  switch(loop_squence)
  {
    case 0: // task space까지 접근 or IK 모듈로 접근 가능 여부 확인
      if (ik_module_->isReachable(current_target, is_target_on_right)) {
        loop_squence = 1;
      } else {
        RCLCPP_WARN(this->get_logger(), "Target unreachable.");

        // **********************************************************************
        // 보행 커멘드 or Arm2Maser Pub 내용 추가할 것
        // **********************************************************************
      }
      break;

    case 1: // EE 경로 생성
      if (!trajectory_generated_) {
        double push_yaw = initial_yaw_ + M_PI / 2.0;  // 벽과 수직하는 경로로 밀기
        trajectory_ = planner_.generateVerticalPushTrajectory(current_target, push_yaw, approach_distance_ , duration_, dt_);  // 수정 필요
        trajectory_index_ = 0;
        trajectory_generated_ = true;
        RCLCPP_INFO(this->get_logger(), "Trajectory generated.");
        loop_squence = 2;
      }
      break;
    case 2:
      RCLCPP_INFO(this->get_logger(), "Generated trajectory size: %ld", trajectory_.size());
      if (trajectory_index_ < trajectory_.size())
      {
        auto point = trajectory_[trajectory_index_++]; // Eigen::Vector3d
        auto q_opt = ik_module_->computeIK(point, is_target_on_right); // optional
        if (q_opt.has_value())
        {
          auto q = q_opt.value();
          std::cout << "trajectory_index_: "<< trajectory_index_ << std::endl;
          RCLCPP_INFO(this->get_logger(), "IK result: q = (%.3f, %.3f, %.3f)", q[0], q[1], q[2]);
          publishJointCommands(q_opt.value(), is_target_on_right);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "IK computation failed for point (%.3f, %.3f, %.3f)", point.x(), point.y(), point.z());
          // **********************************************************************
          // IK 실패 시 보행 커멘드 or Arm2Maser Pub 내용 추가할 것
          // **********************************************************************
        }
      }
      else
      {
        trajectory_generated_ = false;
        loop_squence = 0;
      }
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Invalid step");
      return;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainNode>());
  rclcpp::shutdown();
  return 0;
}
