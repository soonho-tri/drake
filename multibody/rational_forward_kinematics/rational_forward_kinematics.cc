#include "drake/multibody/rational_forward_kinematics/rational_forward_kinematics.h"

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
  RationalFunction rational_zero(0);
  RationalFunction rational_one(1);
  p_WB_[0] << rational_zero, rational_zero, rational_zero;
  // clang-format off
  R_WB_[0] << rational_one, rational_zero, rational_zero,
              rational_zero, rational_one, rational_zero,
              rational_zero, rational_zero, rational_one;
  // clang-format on

  CalcLinkPoses();
}

template <typename DerivedR, typename DerivedP>
void CalcChildPose(const Matrix3<RationalFunction>& R_WP,
                   const Vector3<RationalFunction>& p_WP,
                   const Mobilizer<double>& mobilizer,
                   const Matrix3<DerivedR>& R_FM, const Vector3<DerivedP>& p_FM,
                   Matrix3<RationalFunction>* R_WC,
                   Vector3<RationalFunction>* p_WC) {
  // Frame F is the inboard frame (attached to the parent link), and frame
  // M is the outboard frame (attached to the child link).
  const Frame<double>& frame_F = mobilizer.inboard_frame();
  const Frame<double>& frame_M = mobilizer.outboard_frame();
  const Isometry3<double> X_PF = frame_F.GetFixedPoseInBodyFrame();
  const Isometry3<double> X_MC = frame_M.GetFixedPoseInBodyFrame();
  const Matrix3<RationalFunction> R_WF = R_WP * X_PF.linear();
  const Vector3<RationalFunction> p_WF = R_WP * X_PF.translation() + p_WP;
  const Matrix3<RationalFunction> R_WM = R_WF * R_FM;
  const Vector3<RationalFunction> p_WM = R_WF * p_FM + p_WF;
  const Matrix3<double> R_MC = X_MC.linear();
  const Vector3<double> p_MC = X_MC.translation();
  std::cout << "R_WM:\n" << R_WM << "\n";
  std::cout << "R_MC:\n" << R_MC << "\n";
  std::cout << "p_WM:\n" << p_WM << "\n";
  std::cout << "p_MC:\n" << p_MC << "\n";
  *R_WC = R_WM * R_MC;
  *p_WC = R_WM * p_MC + p_WM;
}

void RationalForwardKinematics::CalcLinkPoseWithRevoluteJoint(
    BodyIndex body_index, const RevoluteMobilizer<double>* revolute_mobilizer) {
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
  const symbolic::Variable& theta_star =
      q_star_(revolute_mobilizer->get_topology().positions_start);
  const symbolic::RationalFunction cos_angle(
      Polynomial(cos(theta_star) * (1 - t_angle * t_angle) -
                     sin(theta_star) * 2 * t_angle,
                 {t_angle}),
      Polynomial(1 + t_angle * t_angle));
  const symbolic::RationalFunction sin_angle(
      Polynomial(sin(theta_star) * (1 - t_angle * t_angle) +
                     cos(theta_star) * 2 * t_angle,
                 {t_angle}),
      Polynomial(1 + t_angle * t_angle));
  std::cout << "cos_angle: " << cos_angle << "\n";
  std::cout << "sin_angle: " << sin_angle << "\n";
  // Frame F is the inboard frame (attached to the parent link), and frame
  // M is the outboard frame (attached to the child link).
  const Matrix3<RationalFunction> R_FM =
      Eigen::Matrix3d::Identity() + sin_angle * A_F +
      (1 - cos_angle) * A_F * A_F;
  const RationalFunction rational_zero(0);
  const Vector3<RationalFunction> p_FM(rational_zero, rational_zero,
                                       rational_zero);
  const BodyIndex parent_index =
      tree_.get_topology().get_body(body_index).parent_body;
  CalcChildPose(R_WB_[parent_index], p_WB_[parent_index], *revolute_mobilizer,
                R_FM, p_FM, &(R_WB_[body_index]), &(p_WB_[body_index]));
  std::cout << "R_WB_[" << body_index << "]:\n" << R_WB_[body_index] << "\n";
  std::cout << "p_WB_[" << body_index << "]:\n" << p_WB_[body_index] << "\n";
}

void RationalForwardKinematics::CalcLinkPoseWithWeldJoint(
    BodyIndex body_index, const WeldMobilizer<double>* weld_mobilizer) {
  const Isometry3<double> X_FM = weld_mobilizer->get_X_FM();
  const Matrix3<double> R_FM = X_FM.linear();
  const Vector3<double> p_FM = X_FM.translation();
  const BodyIndex parent_index =
      tree_.get_topology().get_body(body_index).parent_body;
  CalcChildPose(R_WB_[parent_index], p_WB_[parent_index], *weld_mobilizer, R_FM,
                p_FM, &(R_WB_[body_index]), &(p_WB_[body_index]));
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
      CalcLinkPoseWithRevoluteJoint(body_index, revolute_mobilizer);
    } else if (dynamic_cast<const PrismaticMobilizer<double>*>(mobilizer) !=
               nullptr) {
      throw std::runtime_error("Prismatic joint has not been handled yet.");
    } else if (dynamic_cast<const WeldMobilizer<double>*>(mobilizer) !=
               nullptr) {
      const WeldMobilizer<double>* weld_mobilizer =
          dynamic_cast<const WeldMobilizer<double>*>(mobilizer);
      CalcLinkPoseWithWeldJoint(body_index, weld_mobilizer);
    } else if (dynamic_cast<const SpaceXYZMobilizer<double>*>(mobilizer) !=
               nullptr) {
      throw std::runtime_error("Gimbal joint has not been handled yet.");
    } else if (dynamic_cast<const QuaternionFloatingMobilizer<double>*>(
                   mobilizer) != nullptr) {
      throw std::runtime_error("Free floating joint has not been handled yet.");
    } else {
      throw std::runtime_error(
          "RationalForwardKinematics: Can't handle this mobilizer.");
    }
  }
}

}  // namespace multibody
}  // namespace drake
