#include "drake/solvers/parameterized_mathematical_program.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/solvers/osqp_solver.h"

namespace drake {
namespace solvers {
namespace {

using symbolic::Substitution;
using symbolic::Variable;

// From:
// https://optimization.mccormick.northwestern.edu/index.php/Quadratic_programming#Numerical_example
//
// Min p₀x₀² + x₁² + p₁x₀x₁ + 6x₁ + 2
// s.t.
//      2x₀ + 3x₁ ≥ p₂
//      x₀ ≥ 0
//      x₁ ≥ 0
GTEST_TEST(ParameterizedMathematicalProgramTest, Parameters) {
  // 1. Set up a parameterized mathematical program.
  ParameterizedMathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<2>();
  const Variable& x0{x[0]};
  const Variable& x1{x[1]};
  const auto p = prog.NewParameters<3>();
  const Variable& p0{p[0]};
  const Variable& p1{p[1]};
  const Variable& p2{p[2]};
  EXPECT_EQ(prog.num_parameters(), 3);

  // 2. Add cost and constraints.
  // Cost = p₀x₀² + x₁² + p₁x₀x₁ + 6x₁ + 2
  prog.AddCost(p0 * x0 * x0 + x1 * x1 + p1 * x0 * x1 + x0 + 6 * x1 + 2);
  // Constraint: 2x₀ + 3x₁ ≥ p₂
  prog.AddConstraint(2 * x0 + 3 * x1 >= p2);
  // Constraint: x₀ ≥ 0
  prog.AddConstraint(x0 >= 0);
  // Constraint: x₁ ≥ 0
  prog.AddConstraint(x1 >= 0);

  // 3. Instantiate the parameters with the following values.
  //      p₀ = 3.0
  //      p₁ = 2.0
  //      p₂ = 4.0
  {
    Substitution subst{{p0, 3.0}, {p1, 2.0}, {p2, 4.0}};
    prog.InstantiateParameters(subst);
  }

  // 4. Solve the problem using OSQP.
  OsqpSolver solver;
  {
    // Global solution:
    //      x₀ = 0.5
    //      x₁ = 1.0
    // From:
    // https://optimization.mccormick.northwestern.edu/index.php/Quadratic_programming#Numerical_example
    const MathematicalProgramResult result = solver.Solve(prog, {}, {});
    EXPECT_NEAR(result.GetSolution(x0), 0.5, 1e-15);
    EXPECT_NEAR(result.GetSolution(x1), 1.0, 1e-15);
  }

  // 5. Instantiate the parameters with different values.
  //      p₀ = 2.0
  //      p₁ = 3.0
  //      p₂ = 5.0
  {
    Substitution subst{{p0, 2.0}, {p1, 3.0}, {p2, 5.0}};
    prog.InstantiateParameters(subst);
    const MathematicalProgramResult result1 = solver.Solve(prog, {}, {});

    // 6. Construct a non-parametric mathematical program and solve using OSQP.
    // The two solutions should be the identical as OSQP receives the same
    // problem.
    MathematicalProgram non_param_prog;
    non_param_prog.AddDecisionVariables(x);
    // Cost = 2x₀² + x₁² + 3x₀x₁ + 6x₁ + 2
    non_param_prog.AddCost(2 * x0 * x0 + x1 * x1 + 3 * x0 * x1 + x0 + 6 * x1 +
                           2);
    // Constraint: 2x₀ + 3x₁ ≥ 5
    non_param_prog.AddConstraint(2 * x0 + 3 * x1 >= 5);
    // Constraint: x₀ ≥ 0
    non_param_prog.AddConstraint(x0 >= 0);
    // Constraint: x₁ ≥ 0
    non_param_prog.AddConstraint(x1 >= 0);
    const MathematicalProgramResult result2 =
        solver.Solve(non_param_prog, {}, {});

    EXPECT_EQ(result1.GetSolution(x0), result2.GetSolution(x0));
    EXPECT_EQ(result1.GetSolution(x1), result2.GetSolution(x1));
  }
}

}  // namespace
}  // namespace solvers
}  // namespace drake
