#pragma once

#include <memory>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/space_xyz_mobilizer.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace test {

/// This plant models the rotational motion of a torque free body in space.
/// This body is axially symmetric with rotational inertia about its axis of
/// revolution J and with a rotational inertia I about any axis perpendicular to
/// its axis of revolution.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template<typename T>
class FreeRotatingBodyPlant final : public internal::MultibodyTreeSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FreeRotatingBodyPlant)

  /// Constructor from known rotational inertia values.
  /// Rotational inertia values have units of kg⋅m².
  /// @param I
  ///   rotational inertia about any axis perpendicular to the axis of
  ///   revolution of the body.
  /// @param J
  ///   rotational inertia about the axis of revolution of the body.
  FreeRotatingBodyPlant(double I, double J);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit FreeRotatingBodyPlant(const FreeRotatingBodyPlant<U>&);

  /// Sets `state` to a default value corresponding to a configuration in which
  /// the free body frame B is coincident with the world frame W and the angular
  /// velocity has a value as returned by
  /// get_default_initial_angular_velocity().
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override;

  /// Returns the angular velocity `w_WB` stored in `context` of the free body B
  /// in the world frame W.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const;

  /// Stores in `context` the value of the angular velocity `w_WB` of the body
  /// in the world frame W.
  void set_angular_velocity(
      systems::Context<T>* context, const Vector3<T>& w_WB) const;

  /// Computes the pose `X_WB` of the body in the world frame.
  math::RigidTransform<T> CalcPoseInWorldFrame(
      const systems::Context<T>& context) const;

  /// Computes the spatial velocity `V_WB` of the body in the world frame.
  SpatialVelocity<T> CalcSpatialVelocityInWorldFrame(
      const systems::Context<T>& context) const;

  /// Returns the default value of the angular velocity set by default by
  /// SetDefaultState(). Currently a non-zero value.
  Vector3<double> get_default_initial_angular_velocity() const;

 private:
  const internal::MultibodyTree<T>& tree() const {
    return internal::GetInternalTree(*this);
  }

  void DoCalcTimeDerivatives(
      const systems::Context<T> &context,
      systems::ContinuousState<T> *derivatives) const override;

  void DoMapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      systems::VectorBase<T>* generalized_velocity) const override;

  void DoMapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      systems::VectorBase<T>* qdot) const override;

  // Helper method to build the MultibodyTree model for this plant.
  void BuildMultibodyTreeModel();

  double I_{0};
  double J_{0};

  const RigidBody<T>* body_{nullptr};
  const internal::SpaceXYZMobilizer<T>* mobilizer_{nullptr};
};

}  // namespace test
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::test::FreeRotatingBodyPlant)