#pragma once

#include <memory>

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simpler_car_input.h"
#include "drake/automotive/gen/simpler_car_params.h"
#include "drake/automotive/gen/simpler_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// SimplerCar models an idealized response to driving commands, neglecting all
/// physics. Note that SimplerCar can move forward, stop, turn left, and turn
/// right but *cannot* travel in reverse.
///
/// parameters:
///
/// * uses systems::Parameters wrapping a SimplerCarParams
///
/// state vector (planar for now):
///
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
///   heading is defined around the +z axis, so positive-turn-left
/// * velocity
///
/// input vector:
///
/// * steering angle (virtual center wheel angle);
///   a positive angle means a positive change in heading (left turn);
///   the value must lie within (-pi, +pi).
/// * throttle (0-1)
/// * brake (0-1)
///
/// output port 0: same as state vector.
/// output port 1: A PoseVector containing X_WC, where C is the car frame.
/// output port 2: A FrameVelocity containing Xdot_WC, where C is the car frame.
///
/// @tparam T must support certain arithmetic operations;
/// for details, see drake::symbolic::Expression.
///
/// Instantiated templates for the following ScalarTypes are provided:
///
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_plants
template <typename T>
class SimplerCar final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimplerCar)

  /** Constructs a default car. */
  SimplerCar();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit SimplerCar(const SimplerCar<U>&);

  // System<T> overrides
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  const systems::OutputPort<T>& state_output() const;
  const systems::OutputPort<T>& pose_output() const;
  const systems::OutputPort<T>& velocity_output() const;

 private:
  void CalcStateOutput(const systems::Context<T>&, SimplerCarState<T>*) const;
  // void CalcPose(const systems::Context<T>&,
  //               systems::rendering::PoseVector<T>*) const;

  void ImplCalcTimeDerivatives(const SimplerCarParams<T>& params,
                               const SimplerCarState<T>& state,
                               const SimplerCarInput<T>& input,
                               SimplerCarState<T>* rates) const;

  void CalcSteeringAngleConstraint(const systems::Context<T>&,
                                   VectorX<T>*) const;
  void CalcAccelerationConstraint(const systems::Context<T>&,
                                  VectorX<T>*) const;
  void CalcVelocityConstraint(const systems::Context<T>&, VectorX<T>*) const;
};

}  // namespace automotive
}  // namespace drake
