// Simple Continuous Time System Example
//
// This is meant to be a sort of "hello world" example for the
// drake::system classes.  It defines a very simple continuous time system,
// simulates it from a given initial condition, and plots the result.

#include "drake/common/symbolic_formula.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"

// Simple Continuous Time System
//   xdot = -x + x^3
//   y = x
class SimpleContinuousTimeSystem
    : public drake::systems::LeafSystem<drake::symbolic::Expression> {
 public:
  SimpleContinuousTimeSystem() {
    this->DeclareOutputPort(drake::systems::kVectorValued, 2);
    this->DeclareContinuousState(1, 1, 0);
  }

  // xdot = -x + x^3
  void DoCalcTimeDerivatives(
      const drake::systems::Context<drake::symbolic::Expression>& context,
      drake::systems::ContinuousState<drake::symbolic::Expression>* derivatives)
      const override {
    // Obtain the state.
    const auto& state = context.get_continuous_state_vector();

    // Obtain the structure we need to write into.
    DRAKE_ASSERT(derivatives != nullptr);
    auto const new_derivatives = derivatives->get_mutable_vector();
    DRAKE_ASSERT(new_derivatives != nullptr);

    new_derivatives->SetAtIndex(0, state.GetAtIndex(1));
    new_derivatives->SetAtIndex(1, -9.8);
  }

  // y = x
  void DoCalcOutput(
      const drake::systems::Context<drake::symbolic::Expression>& context,
      drake::systems::SystemOutput<drake::symbolic::Expression>* output)
      const override {
    const drake::symbolic::Expression& x =
        context.get_continuous_state_vector().GetAtIndex(0);
    const drake::symbolic::Expression& v =
        context.get_continuous_state_vector().GetAtIndex(1);
    output->GetMutableVectorData(0)->SetAtIndex(0, x);
    output->GetMutableVectorData(0)->SetAtIndex(1, v);
  }
};

int main(int argc, char* argv[]) {
  // Create the simple system.
  SimpleContinuousTimeSystem system;

  // Create the simulator.
  drake::systems::Simulator<drake::symbolic::Expression> simulator(system);

  // Set the initial conditions x(0).
  drake::systems::ContinuousState<drake::symbolic::Expression>& xc =
      *simulator.get_mutable_context()->get_mutable_continuous_state();
  drake::symbolic::Variable h("h");
  xc[0] = h;
  xc[1] = 0.0;

  // Simulate for 10 seconds.
  simulator.StepTo(10);

  // Make sure the simulation converges to the stable fixed point at x=0.
  std::cerr << "h = " << xc[0] << "\t"
            << "v = " << xc[1] << "\n";
  // DRAKE_DEMAND(xc[0] < 1.0e-4);

  // TODO(russt): make a plot of the resulting trajectory.

  return 0;
}
