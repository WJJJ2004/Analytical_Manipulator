#include "trajectory_planner.hpp"
#include <cmath>

Eigen::Vector3d TrajectoryPlanner::cubicInterp(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    double t)
{
    Eigen::Vector3d delta = p1 - p0;
    return p0 + delta * (3 * t * t - 2 * t * t * t);
}

std::vector<Eigen::Vector3d> TrajectoryPlanner::generateVerticalPushTrajectory(
    const Eigen::Vector3d& target_coord,
    double push_yaw,
    double approach_dist,
    double duration,
    double dt)
{
        std::vector<Eigen::Vector3d> path;

    // 디버깅용 고정 방향: X축 방향 접근
    Eigen::Vector3d direction(1.0, 1.0, 1.0);
    Eigen::Vector3d start_point = target_coord - direction * approach_dist;

    int steps = static_cast<int>(duration / dt);
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Eigen::Vector3d p = cubicInterp(start_point, target_coord, t);
        path.push_back(p);
    }

    return path;
    // std::cout << "push_yaw=" << std::fixed << std::setprecision(3) << push_yaw
    //       << ", approach=" << approach_dist
    //       << ", duration=" << duration
    //       << ", dt=" << dt << std::endl;

    // std::vector<Eigen::Vector3d> path;

    // // 수직 방향 단위 벡터 (-sin(yaw), cos(yaw), 0)
    // Eigen::Vector3d direction_vector(-std::sin(push_yaw), std::cos(push_yaw), 0.0);

    // // 접근 시작점 = 목표점에서 수직 방향으로 approach_dist 떨어진 위치
    // Eigen::Vector3d start_point = target_coord - direction_vector * approach_dist;

    // int steps = static_cast<int>(duration / dt);
    // for (int i = 0; i <= steps; ++i) {
    //     double t = static_cast<double>(i) / steps;  // normalize [0,1]
    //     Eigen::Vector3d p = cubicInterp(start_point, target_coord, t);
    //     path.push_back(p);
    // }
    // return path;
}
