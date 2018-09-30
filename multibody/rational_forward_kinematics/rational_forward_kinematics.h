#pragma once

#include "drake/common/symbolic.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/prismatic_mobilizer.h"
#include "drake/multibody/multibody_tree/quaternion_floating_mobilizer.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/space_xyz_mobilizer.h"
#include "drake/multibody/multibody_tree/weld_mobilizer.h"

namespace drake {
namespace multibody {
/**
 * We can represent the pose (position, orientation) of each link, as rational
 * functions, namely n(t) / d(t) where both the numerator n(t) and denominator
 * d(t) are polynomials of t, and t is some variable related to the generalized
 * position.
 *
 * One example is that for a rotation matrix with angle θ and axis a, the
 * rotation matrix can be written as I + sinθ A + (1-cosθ) A², where A is the
 * skew-symmetric matrix from axis a. We can use the half-angle formulat to
 * substitute the trigonometric function sinθ and cosθ as
 * cosθ = cos(θ*+Δθ) = cosθ*cosΔθ - sinθ*sinΔθ
 *      = (1-t²)/(1+t²) cosθ*- 2t/(1+t²) sinθ*     (1)
 * sinθ = sin(θ*+Δθ) = sinθ*cosΔθ - cosθ*sinΔθ
 *      = (1-t²)/(1+t²) sinθ*- 2t/(1+t²) cosθ*     (2)
 * where θ = θ*+Δθ, and t = tan(Δθ/2). θ* is some given angle.
 * With (1) and (2), both sinθ and cosθ are written as a rational function of t.
 * Thus the rotation matrix can be written as rational functions of t.
 */
class RationalForwardKinematics {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RationalForwardKinematics)

  explicit RationalForwardKinematics(const MultibodyTree<double>& tree);

  const Vector3<symbolic::RationalFunction>& p_WB(int body_index) const {
    return p_WB_[body_index];
  }

  const Matrix3<symbolic::RationalFunction>& R_WB(int body_index) const {
    return R_WB_[body_index];
  }

  const MultibodyTree<double>& tree() const { return tree_; }

  const VectorX<symbolic::Variable>& t() const { return t_; }

  const VectorX<symbolic::Variable>& q_star() const { return q_star_; }

 private:
  // Compute the pose of each link as fractional functions of t.
  // We will set up the indeterminates t also.
  // A revolute joint requires a single t, where t = tan(Δθ/2).
  // A prismatic joint requires a single t, where t = Δd, d being the
  // translational motion of the prismatic joint.
  // A free-floating joint requires 12 t, 3 for position, and 9 for the rotation
  // matrix.
  // A gimbal joint requires 9 t, for the rotation matrix.
  void CalcLinkPoses();

  // Compute the pose of the link, connected to its parent link through a
  // revolute joint.
  void CalcLinkPoseWithRevoluteJoint(
      BodyIndex body_index,
      const RevoluteMobilizer<double>* revolute_mobilizer);

  // Compute the pose of the link, connected to its parent link through a
  // weld joint.
  void CalcLinkPoseWithWeldJoint(BodyIndex body_index,
                                 const WeldMobilizer<double>* weld_mobilizer);

  const MultibodyTree<double>& tree_;
  // The variables used in computing the pose as rational functions. t_ are the
  // indeterminates in the rational functions.
  VectorX<symbolic::Variable> t_;
  // q_star_ is the nominal configuration of the robot. For example, for a
  // revolute joint, it could be θ* mentioned in the class documentation.
  // q_star_ are the decision variables of the rational functions.
  VectorX<symbolic::Variable> q_star_;
  // p_WB_[i] is the position of body i's frame origin, measured in the world
  // frame W. Each entry in p_WB_[i] is a rational function of t_;
  std::vector<Vector3<symbolic::RationalFunction>> p_WB_;
  // R_WB_[i] is the rotation matrix of body i's frame, measured in the world
  // frame W. Each entry in R_WB_[i] is a rational function of t_;
  std::vector<Matrix3<symbolic::RationalFunction>> R_WB_;
};
}  // namespace multibody
}  // namespace drake
