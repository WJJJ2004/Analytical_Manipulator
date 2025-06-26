#ifndef ARM_CTRL__TRAJECTORY_PLANNER_HPP_
#define ARM_CTRL__TRAJECTORY_PLANNER_HPP_

#include <Eigen/Dense>
#include <vector>
#include <iomanip>
#include <iostream>

class TrajectoryPlanner {
public:
  std::vector<Eigen::Vector3d> generateVerticalPushTrajectory(
    const Eigen::Vector3d& target_coord,
    double push_yaw,
    double approach_distance,
    double duration,   // seconds
    double dt);        // seconds per step

  Eigen::Vector3d cubicInterp( // V = 0으로 3차원 선형 보간
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    double t);
};

#endif  // ARM_CTRL__TRAJECTORY_PLANNER_HPP_
