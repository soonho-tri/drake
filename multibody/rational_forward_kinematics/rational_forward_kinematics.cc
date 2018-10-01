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

template <typename Scalar1, typename Scalar2>
void CalcChildPose(const Matrix3<Scalar2>& R_WP, const Vector3<Scalar2>& p_WP,
                   const Mobilizer<double>& mobilizer,
                   const Matrix3<Scalar1>& R_FM, const Vector3<Scalar1>& p_FM,
                   Matrix3<Scalar2>* R_WC, Vector3<Scalar2>* p_WC) {
  // Frame F is the inboard frame (attached to the parent link), and frame
  // M is the outboard frame (attached to the child link).
  const Frame<double>& frame_F = mobilizer.inboard_frame();
  const Frame<double>& frame_M = mobilizer.outboard_frame();
  const Isometry3<double> X_PF = frame_F.GetFixedPoseInBodyFrame();
  const Isometry3<double> X_MC = frame_M.GetFixedPoseInBodyFrame();
  const Matrix3<Scalar2> R_WF = R_WP * X_PF.linear();
  const Vector3<Scalar2> p_WF = R_WP * X_PF.translation() + p_WP;
  const Matrix3<Scalar2> R_WM = R_WF * R_FM;
  const Vector3<Scalar2> p_WM = R_WF * p_FM + p_WF;
  const Matrix3<double> R_MC = X_MC.linear();
  const Vector3<double> p_MC = X_MC.translation();
  *R_WC = R_WM * R_MC;
  *p_WC = R_WM * p_MC + p_WM;
}

void RationalForwardKinematics::
    CalcLinkPoseAsMultilinearPolynomialWithRevoluteJoint(
        const RevoluteMobilizer<double>* revolute_mobilizer,
        const Matrix3<Polynomial>& R_WP, const Vector3<Polynomial>& p_WP,
        VectorX<symbolic::Variable>* cos_delta,
        VectorX<symbolic::Variable>* sin_delta,
        VectorX<symbolic::Variable>* t_angles, Matrix3<Polynomial>* R_WC,
        Vector3<Polynomial>* p_WC) {
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
  t_angles->conservativeResize(t_angles->size() + 1);
  (*t_angles)(t_angles->size() - 1) = t_angle;
  const symbolic::Variable cos_delta_i(
      "cos(delta_q(" +
      std::to_string(revolute_mobilizer->position_start_in_q()) + "))");
  const symbolic::Variable sin_delta_i(
      "sin(delta_q(" +
      std::to_string(revolute_mobilizer->position_start_in_q()) + "))");
  cos_delta->conservativeResize(cos_delta->size() + 1);
  sin_delta->conservativeResize(sin_delta->size() + 1);
  (*cos_delta)(cos_delta->size() - 1) = cos_delta_i;
  (*sin_delta)(sin_delta->size() - 1) = sin_delta_i;
  const symbolic::Variable& theta_star =
      q_star_(revolute_mobilizer->get_topology().positions_start);
  const symbolic::Polynomial cos_angle(
      cos(theta_star) * cos_delta_i - sin(theta_star) * sin_delta_i,
      {cos_delta_i, sin_delta_i});
  const symbolic::Polynomial sin_angle(
      sin(theta_star) * cos_delta_i + cos(theta_star) * sin_delta_i,
      {cos_delta_i, sin_delta_i});
  // Frame F is the inboard frame (attached to the parent link), and frame
  // M is the outboard frame (attached to the child link).
  const Matrix3<Polynomial> R_FM =
      Eigen::Matrix3d::Identity() +
      sin_angle * A_F.cast<symbolic::Polynomial>() +
      (1 - cos_angle) * A_F * A_F;
  const Polynomial polynomial_zero{};
  const Vector3<Polynomial> p_FM(polynomial_zero, polynomial_zero,
                                 polynomial_zero);
  CalcChildPose(R_WP, p_WP, *revolute_mobilizer, R_FM, p_FM, R_WC, p_WC);
}

void RationalForwardKinematics::CalcLinkPoseWithWeldJoint(
    const WeldMobilizer<double>* weld_mobilizer,
    const Matrix3<Polynomial>& R_WP, const Vector3<Polynomial>& p_WP,
    Matrix3<Polynomial>* R_WC, Vector3<Polynomial>* p_WC) {
  const Isometry3<double> X_FM = weld_mobilizer->get_X_FM();
  const Matrix3<double> R_FM = X_FM.linear();
  const Vector3<double> p_FM = X_FM.translation();
  CalcChildPose(R_WP, p_WP, *weld_mobilizer, R_FM, p_FM, R_WC, p_WC);
}

void RationalForwardKinematics::CalcLinkPoses() {
  DRAKE_DEMAND(t_.size() == 0);
  // We will first compute the link pose as multilinear polynomials, with
  // indeterminates cos_delta and sin_delta, representing cos(Δθ) and sin(Δθ)
  // respectively. We will then replace cos_delta and sin_delta in the link
  // pose with rational functions (1-t^2)/(1+t^2) and 2t/(1+t^2) respectively.
  // The reason why we don't use RationalFunction directly, is that currently
  // our rational function can't find the common factor in the denominator,
  // namely the sum between rational functions p1(x) / (q1(x) * r(x)) + p2(x) /
  // r(x) is computed as (p1(x) * r(x) + p2(x) * q1(x) * r(x)) / (q1(x) * r(x) *
  // r(x)), without handling the common factor r(x) in the denominator.
  VectorX<symbolic::Variable> cos_delta, sin_delta, t_angles;
  std::vector<Matrix3<Polynomial>> R_WB_poly(tree_.num_bodies());
  std::vector<Vector3<Polynomial>> p_WB_poly(tree_.num_bodies());
  const Polynomial poly_zero{};
  const Polynomial poly_one{1};
  // clang-format off
  R_WB_poly[0] << poly_one, poly_zero, poly_zero,
                  poly_zero, poly_one, poly_zero,
                  poly_zero, poly_zero, poly_one;
  p_WB_poly[0] << poly_zero, poly_zero, poly_zero;
  // clang-format on
  for (BodyIndex body_index(1); body_index < tree_.num_bodies(); ++body_index) {
    const BodyTopology& body_topology =
        tree_.get_topology().get_body(body_index);
    const auto mobilizer =
        &(tree_.get_mobilizer(body_topology.inboard_mobilizer));
    const BodyIndex parent_index =
        tree_.get_topology().get_body(body_index).parent_body;
    if (dynamic_cast<const RevoluteMobilizer<double>*>(mobilizer) != nullptr) {
      const RevoluteMobilizer<double>* revolute_mobilizer =
          dynamic_cast<const RevoluteMobilizer<double>*>(mobilizer);
      CalcLinkPoseAsMultilinearPolynomialWithRevoluteJoint(
          revolute_mobilizer, R_WB_poly[parent_index], p_WB_poly[parent_index],
          &cos_delta, &sin_delta, &t_angles, &(R_WB_poly[body_index]),
          &(p_WB_poly[body_index]));
    } else if (dynamic_cast<const PrismaticMobilizer<double>*>(mobilizer) !=
               nullptr) {
      throw std::runtime_error("Prismatic joint has not been handled yet.");
    } else if (dynamic_cast<const WeldMobilizer<double>*>(mobilizer) !=
               nullptr) {
      const WeldMobilizer<double>* weld_mobilizer =
          dynamic_cast<const WeldMobilizer<double>*>(mobilizer);
      CalcLinkPoseWithWeldJoint(
          weld_mobilizer, R_WB_poly[parent_index], p_WB_poly[parent_index],
          &(R_WB_poly[body_index]), &(p_WB_poly[body_index]));
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

  // Now convert the multilinear polynomial of cos and sin to rational function
  // of t.
  symbolic::Variables t_variables(t_);
  for (BodyIndex body_index{1}; body_index < tree_.num_bodies(); ++body_index) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        ReplaceCosAndSinWithRationalFunction(
            R_WB_poly[body_index](i, j).ToExpression(), cos_delta, sin_delta,
            t_angles, t_variables, &(R_WB_[body_index](i, j)));
      }
      ReplaceCosAndSinWithRationalFunction(
          p_WB_poly[body_index](i).ToExpression(), cos_delta, sin_delta,
          t_angles, t_variables, &(p_WB_[body_index](i)));
    }
  }
}

void ReplaceCosAndSinWithRationalFunction(
    const symbolic::Expression& e, const VectorX<symbolic::Variable>& cos_delta,
    const VectorX<symbolic::Variable>& sin_delta,
    const VectorX<symbolic::Variable>& t_angle, const symbolic::Variables& t,
    symbolic::RationalFunction* e_rational) {
  DRAKE_DEMAND(cos_delta.rows() == sin_delta.rows());
  DRAKE_DEMAND(cos_delta.rows() == t_angle.rows());
  VectorX<symbolic::Variable> cos_sin_delta(2 * cos_delta.rows());
  cos_sin_delta << cos_delta, sin_delta;
  const symbolic::Polynomial e_poly(e, symbolic::Variables(cos_sin_delta));
  // First find the angles whose cos or sin appear in the polynomial. This will
  // determine the denominator of the rational function.
  std::set<int> angle_indices;
  for (const auto& pair : e_poly.monomial_to_coefficient_map()) {
    // Also check that this monomial can't contain both cos_delta(i) and
    // sin_delta(i).
    for (int i = 0; i < cos_delta.rows(); ++i) {
      const int angle_degree =
          pair.first.degree(cos_delta(i)) + pair.first.degree(sin_delta(i));
      DRAKE_DEMAND(angle_degree <= 1);
      if (angle_degree == 1) {
        angle_indices.insert(i);
      }
    }
  }
  if (angle_indices.empty()) {
    *e_rational = RationalFunction(Polynomial(e, t));
    return;
  }
  Polynomial denominator(1);
  for (int angle_index : angle_indices) {
    denominator *= Polynomial(1 + t_angle(angle_index) * t_angle(angle_index));
  }
  Polynomial numerator{};
  for (const auto& pair : e_poly.monomial_to_coefficient_map()) {
    // If the monomial contains cos_delta(i), then replace cos_delta(i) with
    // 1 - t_angle(i) * t_angle(i).
    // If the monomial contains sin_delta(i), then replace sin_delta(i) with
    // 2 * t_angle(i).
    // Otherwise, multiplies with 1 + t_angle(i) * t_angle(i)
    symbolic::Polynomial numerator_monomial(pair.second, t);
    for (int angle_index : angle_indices) {
      if (pair.first.degree(cos_delta(angle_index)) > 0) {
        numerator_monomial *=
            Polynomial(1 - t_angle(angle_index) * t_angle(angle_index));
      } else if (pair.first.degree(sin_delta(angle_index)) > 0) {
        numerator_monomial *= Polynomial(2 * t_angle(angle_index));
      } else {
        numerator_monomial *=
            Polynomial(1 + t_angle(angle_index) * t_angle(angle_index));
      }
    }
    numerator += numerator_monomial;
  }
  *e_rational = RationalFunction(numerator, denominator);
}
}  // namespace multibody
}  // namespace drake
