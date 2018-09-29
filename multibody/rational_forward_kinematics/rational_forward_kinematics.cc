#include "drake/multibody/rational_forward_kinematics/rational_forward_kinematics.h"
#include "drake/multibody/multibody_tree/prismatic_mobilizer.h"
#include "drake/multibody/multibody_tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/space_xyz_mobilizer.h"
#include "drake/multibody/multibody_tree/weld_mobilizer.h"

namespace drake {
namespace multibody {
using symbolic::Polynomial;
using symbolic::RationalFunction;
RationalForwardKinematics::RationalForwardKinematics(
    const MultibodyTree<double>& tree)
    : tree_(tree),
      q_star_(tree_.num_positions()),
      p_WB_(tree_.num_bodies()),
      R_WB_(tree_.num_bodies()) {
  for (int i = 0; i < tree_.num_positions(); ++i) {
    q_star_(i) = symbolic::Variable("q" + std::to_string(i));
  }
}

void RationalForwardKinematics::CalcLinkPoses() {
  DRAKE_DEMAND(t_.size() == 0);
  for (BodyIndex body_index(1); body_index < tree_.num_bodies(); ++body_index) {
    const BodyTopology& body_topology =
        tree_.get_topology().get_body(body_index);
    const auto mobilizer =
        &(tree_.get_mobilizer(body_topology.inboard_mobilizer));
    if (dynamic_cast<const RevoluteMobilizer<double>*>(mobilizer) != nullptr) {
      const RevoluteMobilizer<double>* revolute_mobilizer =
          dynamic_cast<const RevoluteMobilizer<double>*>(mobilizer);
      const Eigen::Vector3d& axis_F = revolute_mobilizer->revolute_axis();
      // clang-format off
      const Eigen::Matrix3d A_F =
          (Eigen::Matrix3d() << 0, -axis_F(2), axis_F(1),
                                axis_F(2), 0, -axis_F(0),
                                -axis_F(1), axis_F(0), 0).finished();
      // clang-format on
      t_.conservativeResize(t_.rows() + 1);
      const symbolic::Variable t_angle("t" + std::to_string(t_.rows() - 1));
      t_(t_.rows() - 1) = t_angle;
      const symbolic::RationalFunction cos_t(Polynomial(1 - t_angle * t_angle),
                                             Polynomial(1 + t_angle * t_angle));
      const symbolic::RationalFunction sin_t(Polynomial(2 * t_angle),
                                             Polynomial(1 + t_angle * t_angle));
      const symbolic::Variable& theta_star =
          q_star_(revolute_mobilizer->get_topology().positions_start);
      const symbolic::RationalFunction cos_angle =
          cos(theta_star) * cos_t - sin(theta_star) * sin_t;
      const symbolic::RationalFunction sin_angle =
          sin(theta_star) * cos_t + cos(theta_star) * sin_t;
    } else if (dynamic_cast<const PrismaticMobilizer<double>*>(mobilizer) !=
               nullptr) {
    } else if (dynamic_cast<const WeldMobilizer<double>*>(mobilizer) !=
               nullptr) {
    } else if (dynamic_cast<const SpaceXYZMobilizer<double>*>(mobilizer) !=
               nullptr) {
    } else if (dynamic_cast<const QuaternionFloatingMobilizer<double>*>(
                   mobilizer) != nullptr) {
    } else {
      throw std::runtime_error(
          "RationalForwardKinematics: Can't handle this mobilizer.");
    }
  }
  return num_t;
}
}  // namespace multibody
}  // namespace drake
