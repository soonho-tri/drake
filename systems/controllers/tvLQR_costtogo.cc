//
// Created by joyce on 7/9/18.
//
#include "drake/systems/controllers/tvLQR_costtogo.h"
#include <memory>
#include <utility>
#include "drake/common/eigen_types.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/primitives/linear_system.h"
namespace drake {
namespace systems {
namespace controllers {
using systems::System;
using trajectories::Trajectory;
TimeVaryingLQRCostToGo::TimeVaryingLQRCostToGo(
    const systems::System<double>& model, const Trajectory<double>& state_traj,
    const Trajectory<double>& input_traj, const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R)
    : systems::LeafSystem<double>(),
      system_(&model),
      state_traj_(&state_traj),
      input_traj_(&input_traj),
      Q_(Q),
      R_(R) {
  // Declare input port and output port
  const int number_generalized_position = 1;
  const int number_generalized_velocity = 1;
  const int number_miscellaneous_variables = 0;
  this->DeclareContinuousState(number_generalized_position,
                               number_generalized_velocity,
                               number_miscellaneous_variables);
  // Provide the output port with information as to how many scalars are to be
  // returned and which method to call to fill array that the port will pass.
  // Note: prototype_output_vector is just a model of how much space is needed.
  const int number_output_scalars = 2;
  systems::BasicVector<double> prototype_output_vector(number_output_scalars);
  this->DeclareVectorOutputPort(prototype_output_vector,
                                &TimeVaryingLQRCostToGo::CopyStateOut);
}
}  // namespace controllers
}  // namespace systems
}  // namespace drake
