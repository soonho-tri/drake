#pragma once

#include <sstream>
#include <stdexcept>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {

using trajectories::Trajectory;
// using systems::System;
// typedef Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> RowMatrixXd;

class TimeVaryingLQRCostToGo : public systems::LeafSystem<double> {
 public:
  explicit TimeVaryingLQRCostToGo(const systems::System<double>& model,
                                  const Trajectory<double>& state_traj,
                                  const Trajectory<double>& input_traj,
                                  const Eigen::MatrixXd& Q,
                                  const Eigen::MatrixXd& R);

  /*
    template <typename T>
    const Eigen::MatrixXd get_S(
        const systems::System<double>& model,
        const systems::Context<T>& context,
        const Eigen::Ref<const Eigen::MatrixXd>& Q,
        const Eigen::Ref<const Eigen::MatrixXd>& R) {
      auto lqr = LinearQuadraticRegulator(model, context, Q, R);
      return lqr.S;
    };
  */

  /// Return the OutputPort associated with this Particle1dPlant.
  /// This method is called when connecting the ports in the diagram builder.
  template <typename T>
  const systems::OutputPort<T>& get_output_port() const {
    return systems::System<T>::get_output_port(0);
  }

  template <typename T>
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const {
    output->set_value(get_state(context).get_value());
  }

 private:
  // Casts the continuous state vector from a VectorBase to a BasicVector.
  template <typename T>
  static const systems::BasicVector<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const systems::BasicVector<T>&>(cstate.get_vector());
  }

  template <typename T>
  static const systems::BasicVector<T>& get_state(
      const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  template <typename T>
  static systems::BasicVector<T>& get_mutable_state(
      systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  /*
  // This is the calculator method that assigns values to the state output port.
  template <typename T>
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const {
    // Get current state from the context.
    const systems::VectorBase<T>& continuous_state_vector =
        context.get_continuous_state_vector();
    // Write system output.
    output->set_value(continuous_state_vector.CopyToVector());
  };
*/

  // Casts the mutable continuous state vector from a VectorBase to BasicVector.
  template <typename T>
  static systems::BasicVector<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<systems::BasicVector<T>&>(cstate->get_mutable_vector());
  }

  const System<double>* system_;
  // const std::unique_ptr<systems::System<double>> system_;
  const Trajectory<double>* state_traj_;

  const Trajectory<double>* input_traj_;
  const Eigen::MatrixXd Q_;
  const Eigen::MatrixXd R_;

  template <typename T>
  void DoCalcTimeDerivative(const systems::Context<T>& context,
                            systems::ContinuousState<T>* derivatives,
                            const Eigen::Ref<const Eigen::MatrixXd>& Q,
                            const Eigen::Ref<const Eigen::MatrixXd>& R) {
    // context is costtogo context
    // system_context is the plant context
    double t = context.get_time();
    // Get a system context
    auto system_context = system_->CreateDefaultContext();
    // Get the x* and u*
    // system_context->get_mutable_state(state_traj_->value(t));
    system_context->get_mutable_continuous_state_vector().SetFromVector(
        state_traj_->value(t));
    system_context->FixInputPort(0, input_traj_->value(t));
    // Linearize the dynamics f to get A(t), B(t)
    auto linear_system =
        systems::Linearize(*system_, *system_context, 0, kNoOutput);
    Eigen::MatrixXd A = linear_system->A();
    Eigen::MatrixXd B = linear_system->B();
    // Now get Sdot
    // Eigen::MatrixXd S = get_S(system_, context, Q, R);
    const systems::BasicVector<T>& S_flat = get_state(context);
    systems::BasicVector<T>& derivative = get_mutable_state(derivatives);
    const int n = sqrt(S_flat.size());
    /*
        const Eigen::MatrixXd S(n, n); // n is the dimension of the state of
       system_ for (int i = 0; i < n; ++i) { S.col(i) = S_flat.segment(i*n, n);
        }
        */
    Eigen::MatrixXd S(S_flat.data(), n, n);
    Eigen::MatrixXd Sdot = -(A.transpose() * S + S * A -
                             S * B * R.inverse() * B.transpose() * S + Q);
  };
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
