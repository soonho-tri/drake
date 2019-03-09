#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/automotive/simpler_car.h"
#include "drake/common/proto/call_matlab.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"

namespace drake {
namespace automotive {
namespace {

using std::endl;
using std::move;
using std::ostream;
using std::string;

class Profiler {
 public:
  explicit Profiler(std::string name, std::ostream& out = std::cerr);
  Profiler(const Profiler&) = delete;
  Profiler(Profiler&&) = delete;
  Profiler& operator=(const Profiler&) = delete;
  Profiler& operator=(Profiler&&) = delete;
  void Stop();

 private:
  const std::string name_;
  std::ostream& out_;
  const std::chrono::high_resolution_clock::time_point begin_;
};

Profiler::Profiler(string name, ostream& out)
    : name_{move(name)},
      out_(out),
      begin_(std::chrono::high_resolution_clock::now()) {}

void Profiler::Stop() {
  using duration = std::chrono::duration<double>;
  const auto diff = std::chrono::high_resolution_clock::now() - begin_;
  out_ << name_ << ": " << std::chrono::duration_cast<duration>(diff).count()
       << endl;
}

double deg2rad(double degrees) { return degrees * 4.0 * atan(1.0) / 180.0; }

GTEST_TEST(TrajectoryOptimizationTest, SimplerCarDircolTest) {
  Profiler profiler_problem_formulation("Problem Formulation");
  // Introduce constants.
  constexpr double r1 = 4;             // Inner Circle
  constexpr double r2 = 7;             // Outer Circle
  constexpr double R = (r1 + r2) / 2;  // Reference Radios
  constexpr double horizon = 5.0;
  const int kNumTimeSamples = 25;
  constexpr double kMinimumTimeStep = 0.01;
  constexpr double kMaximumTimeStep = horizon / kNumTimeSamples;

  SimplerCar<double> plant;
  auto context = plant.CreateDefaultContext();
  systems::trajectory_optimization::DirectCollocation prog(
      &plant, *context, kNumTimeSamples, kMinimumTimeStep, kMaximumTimeStep);

  // Create symbolic input and state to make the problem formulation easier.
  SimplerCarInput<symbolic::Expression> input;
  input.SetFromVector(prog.input().cast<symbolic::Expression>());

  SimplerCarState<symbolic::Expression> state;
  state.SetFromVector(prog.state().cast<symbolic::Expression>());

  // ===========
  // Constraints
  // ===========
  prog.AddEqualTimeIntervalsConstraints();

  // Initial condition.
  SimplerCarState<double> x0;
  x0.set_x(0);
  x0.set_y(r1 + (r2 - r1) / 3.0);
  x0.set_heading(deg2rad(30));
  prog.AddLinearConstraint(prog.initial_state() == x0.get_value());

  // Final conditions.
  SimplerCarState<double> xf;
  xf.set_x(r1 + (r2 - r1) / 2.0);
  xf.set_y(0.0);
  xf.set_heading(deg2rad(-90));
  prog.AddLinearConstraint(prog.final_state() == xf.get_value());

  // Bounds on state
  prog.AddConstraintToAllKnotPoints(state.x() <= 10);
  prog.AddConstraintToAllKnotPoints(state.x() >= -10);
  prog.AddConstraintToAllKnotPoints(state.y() <= 10);
  prog.AddConstraintToAllKnotPoints(state.y() >= -10);
  prog.AddConstraintToAllKnotPoints(state.heading() <= M_PI);
  prog.AddConstraintToAllKnotPoints(state.heading() >= -M_PI);

  // Bounds on input
  prog.AddConstraintToAllKnotPoints(input.velocity() <= 10);
  prog.AddConstraintToAllKnotPoints(input.velocity() >= 0);
  prog.AddConstraintToAllKnotPoints(input.steering() <= M_PI);
  prog.AddConstraintToAllKnotPoints(input.steering() >= -M_PI);

  // Reference bounding box constraints
  prog.AddConstraintToAllKnotPoints(pow(state.x(), 2) + pow(state.y(), 2) >=
                                    r1 * r1);
  prog.AddConstraintToAllKnotPoints(r2 * r2 >=
                                    pow(state.x(), 2) + pow(state.y(), 2));

  // =====
  // Costs
  // =====

  // Reference Trajectory: (x² + y² - R²)²
  prog.AddRunningCost(
      pow(pow(state.x(), 2) + pow(state.y(), 2) - R * R, 2).Expand());

  // Input
  prog.AddRunningCost(pow(input.velocity(), 2) + pow(input.steering(), 2));

  // TODO(soonho-tri): Port the "smoothness"

  // ==================
  // Initial Trajectory
  // ==================
  // Initial guess is a straight line from the initial state to the final
  auto initial_state_trajectory =
      trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          {0, 25}, {x0.get_value(), xf.get_value()});
  prog.SetInitialTrajectory(trajectories::PiecewisePolynomial<double>(),
                            initial_state_trajectory);

  // Ipopt Option
  prog.SetSolverOption(solvers::IpoptSolver::id(), "max_iter", 1000);

  solvers::IpoptSolver solver;
  profiler_problem_formulation.Stop();

  Profiler profiler_solve("Solve");
  const auto result = solver.Solve(prog, {}, {});
  profiler_solve.Stop();
  ASSERT_TRUE(result.is_success());

  // Plot the solution.
  // Note: see call_matlab.h for instructions on viewing the plot.
  Eigen::MatrixXd inputs = prog.GetInputSamples(result);
  Eigen::MatrixXd states = prog.GetStateSamples(result);

  std::cerr << "x = [";
  for (int i = 0; i < states.cols(); ++i) {
    std::cerr << states(0, i) << ", ";
  }
  std::cerr << "]\n";

  std::cerr << "y = [";
  for (int i = 0; i < states.cols(); ++i) {
    std::cerr << states(1, i) << ", ";
  }
  std::cerr << "]\n";

  std::cerr << prog.GetSampleTimes(result) << std::endl;

  std::cerr << states << std::endl;

  common::CallMatlab("plot", states.row(SimplerCarStateIndices::kX),
                     states.row(SimplerCarStateIndices::kY));
  common::CallMatlab("xlabel", "x (m)");
  common::CallMatlab("ylabel", "y (m)");

  // // Checks that the input commands found are not too large.
  // EXPECT_LE(inputs.row(0).lpNorm<1>(), 0.1);
  // EXPECT_LE(inputs.row(1).lpNorm<1>(), 1);

  // TODO(soonho-tri): Visualize the solution.
}

}  // namespace
}  // namespace automotive
}  // namespace drake
