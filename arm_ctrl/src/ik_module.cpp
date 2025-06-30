#include "ik_module.hpp"
#include <cmath>
#include <iostream>

IKModule::IKModule(rclcpp::Node* node) {
    node->get_parameter("L1", L1);
    node->get_parameter("L2", L2);
    node->get_parameter("L3", L3);
    node->get_parameter("D1", D1);
    node->get_parameter("D2", D2);
    node->get_parameter("D3", D3);
    node->get_parameter("theta1_max", theta1_max);
    node->get_parameter("theta1_min", theta1_min);
    node->get_parameter("theta2_max", theta2_max);
    node->get_parameter("theta2_min", theta2_min);
    node->get_parameter("theta3_max", theta3_max);
    node->get_parameter("theta3_min", theta3_min);
    node->get_parameter("error_threshold", error_threshold);

    std::cout << "\033[1;33m";
    std::cout << "[IK] L1: " << L1 << ", L2: " << L2 << ", L3: " << L3 << "\n";
    std::cout << "[IK] D1: " << D1 << ", D2: " << D2 << ", D3: " << D3 << "\n";
    std::cout << "[IK] theta1 range: [" << theta1_min << ", " << theta1_max << "]\n";
    std::cout << "[IK] theta2 range: [" << theta2_min << ", " << theta2_max << "]\n";
    std::cout << "[IK] theta3 range: [" << theta3_min << ", " << theta3_max << "]\n";
    std::cout << "[IK] error threshold: " << error_threshold << "\n";
    std::cout << "\033[0m";

    prev_q << 0, 0, 0;
    offset << L3, L2, -L1;
    R_base << 0, 0, 1,
              0, 1, 0,
             -1, 0, 0;
}

Eigen::Matrix4d IKModule::computeDH(double alpha, double a, double d, double theta) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta), 0, a,
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha),
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha),
         0, 0, 0, 1;
    return T;
}

Eigen::Matrix4d IKModule::forwardKinematics(const Eigen::Vector3d& q) {
    double th1 = q[0], th2 = q[1], th3 = q[2];
    Eigen::Matrix4d T1 = computeDH(0, 0, 0, th1);
    Eigen::Matrix4d T2 = computeDH(M_PI_2, D1, 0, th2);
    Eigen::Matrix4d T3 = computeDH(0, D2, 0, th3);
    Eigen::Matrix4d T4 = computeDH(0, D3, 0, 0);
    return T1 * T2 * T3 * T4;
}

std::vector<Eigen::Vector3d> IKModule::generateIKCandidates(const Eigen::Vector3d& target_local) {
    std::vector<Eigen::Vector3d> candidates;

    double x = target_local.x();
    double y = target_local.y();
    double z = target_local.z();

    if(std::sqrt(x*x + y*y + z*z) > D1 + D2 + D3) {
        std::cerr << "[IK] TARGET OUT OF TASK SPACE: " << target_local.transpose() << std::endl;
        return candidates; // OUT OF Task Space
    }

    // compute theta1
    double theta1 = std::atan2(y, x);

    // 사형 좌표계 만들고 역축 좌표 변환 (코드 수정 x)
    double Swp_X = std::sqrt(x*x + y*y) - D1;
    double Swp_Y = z;

    double C3_num = Swp_X*Swp_X + Swp_Y*Swp_Y - D2*D2 - D3*D3;
    double C3_den = 2 * D2 * D3;
    double C3 = C3_num / C3_den;

    if (std::abs(C3) > 1.0) {
        std::cerr << "[IK] C3 OUT OF BOUND: " << C3 << std::endl;
        return candidates;
    }

    // theta3의 후보 2개 만들기
    double S3_pos = std::sqrt(1 - C3*C3);
    double S3_neg = -S3_pos;

    // theta3으로 2개의 C2 만듬 -> C2로 2개의 theta2 후보 만듬 -> 후보 컨테이너에 푸쉬때림
    for (double S3 : {S3_pos, S3_neg})
    {
        double theta3 = std::atan2(S3, C3);
        double C2_num = (D2 + D3*C3)*Swp_X + D3*S3*Swp_Y;
        double C2_den = (D2 + D3*C3)*(D2 + D3*C3) + (D3*S3)*(D3*S3);
        double C2 = C2_num / C2_den;

        if (std::abs(C2) > 1.0) {
            std::cerr << "[IK] C2 OUT OF BOUND: " << C2 << std::endl;
            continue;
        }

        double S2_pos = std::sqrt(1 - C2*C2);
        double S2_neg = -S2_pos;

        for (double S2 : {S2_pos, S2_neg})
        {
            double theta2 = std::atan2(S2, C2);
            candidates.emplace_back(Eigen::Vector3d(theta1, theta2, theta3));
        }
    }
    return candidates;
}

std::optional<Eigen::Vector3d> IKModule::sortingIKCandidates(const std::vector<Eigen::Vector3d>& candidates, const Eigen::Vector3d& target_world)
{
    double min_joint_change = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_q;
    bool found_valid = false;

    for (const auto& q : candidates)
    {
        // Joint limit check
        if (q[0] < theta1_min || q[0] > theta1_max ||
            q[1] < theta2_min || q[1] > theta2_max ||
            q[2] < theta3_min || q[2] > theta3_max)
        {
            std::cerr << "[IK] CANDIDATE JOINT LIMIT VIOLATION: ("
                      << q[0] << ", " << q[1] << ", " << q[2] << ")" << std::endl;
            continue;
        }

        // FK로 EE 위치 계산
        Eigen::Vector3d ee_world = R_base * forwardKinematics(q).block<3,1>(0,3) + offset;
        double pos_error = (ee_world - target_world).norm();

        // 너무 큰 허근 제거
        if (pos_error > error_threshold)
            continue;

        // 조인트 변화량 최소인 해 선택
        double joint_diff = (q - prev_q).norm();
        if (joint_diff < min_joint_change)
        {
            min_joint_change = joint_diff;
            best_q = q;
            found_valid = true;
        }
    }

    if (found_valid) {
        prev_q = best_q;  // 이전 해 업데이트 (optional)
        return best_q;
    }
    else {
        return std::nullopt;
    }
}

std::optional<Eigen::Vector3d> IKModule::computeIK(const Eigen::Vector3d& target_position) {
    Eigen::Vector3d target = target_position;

    Eigen::Vector3d target_local = R_base.transpose() * (target - offset);
    auto candidates = generateIKCandidates(target_local);
    return sortingIKCandidates(candidates, target);
}