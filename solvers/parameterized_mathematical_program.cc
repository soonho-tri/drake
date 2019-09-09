#include "drake/solvers/parameterized_mathematical_program.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

namespace drake {
namespace solvers {

using symbolic::Environment;
using symbolic::Expression;
using symbolic::Formula;
using symbolic::Substitution;
using symbolic::Variable;
using symbolic::Variables;

bool ParameterizedMathematicalProgram::IncludeParameter(
    const Variables& variables) const {
  for (int i = 0; i < parameters_.size(); ++i) {
    if (variables.include(parameters_[i])) {
      return true;
    }
  }
  return false;
}

int ParameterizedMathematicalProgram::num_parameters() const {
  return parameters_.size();
}

void ParameterizedMathematicalProgram::AddParameters(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& new_parameters) {
  const int num_old_parameters = num_parameters();
  for (int i = 0; i < new_parameters.rows(); ++i) {
    if (new_parameters(i).is_dummy()) {
      throw std::runtime_error(
          fmt::format("new_parameters({}) should not be a dummy variable.", i));
    }
    if ((indeterminates_index().find(new_parameters(i).get_id()) !=
         indeterminates_index().end()) ||
        (parameters_index_.find(new_parameters(i).get_id()) !=
         parameters_index_.end()) ||
        (decision_variable_index().find(new_parameters(i).get_id()) !=
         decision_variable_index().end())) {
      throw std::runtime_error(fmt::format(
          "{} already exists in the optimization program.", new_parameters(i)));
    }
    if (new_parameters(i).get_type() != symbolic::Variable::Type::CONTINUOUS) {
      throw std::runtime_error("parameter should of type CONTINUOUS.\n");
    }
    parameters_index_.insert(
        std::make_pair(new_parameters(i).get_id(), num_old_parameters + i));
  }
  parameters_.conservativeResize(num_old_parameters + new_parameters.rows());
  parameters_.tail(new_parameters.rows()) = new_parameters;
}

void ParameterizedMathematicalProgram::AddConstraint(const Formula& f) {
  if (IncludeParameter(f.GetFreeVariables())) {
    constraints_.push_back(f);
  } else {
    MathematicalProgram::AddConstraint(f);
  }
}

void ParameterizedMathematicalProgram::AddCost(const Expression& e) {
  if (IncludeParameter(e.GetVariables())) {
    costs_.push_back(e);
  } else {
    MathematicalProgram::AddCost(e);
  }
}

void ParameterizedMathematicalProgram::InstantiateParameters(
    const Substitution& s) {
  for (const Expression& cost : costs_) {
    Binding<Cost> binding = internal::ParseCost(cost.Substitute(s));
    auto it = cost_map_.find(cost);
    if (it != cost_map_.end()) {
      // This cost has been instantiated before. Need to update the existing
      // cost. For now, we only support quadratic costs.
      const auto existing_quadratic_cost =
          std::dynamic_pointer_cast<QuadraticCost>(it->second);
      const auto new_quadratic_cost =
          std::dynamic_pointer_cast<QuadraticCost>(binding.evaluator());
      if (existing_quadratic_cost && new_quadratic_cost) {
        existing_quadratic_cost->UpdateCoefficients(new_quadratic_cost->Q(),
                                                    new_quadratic_cost->b(),
                                                    new_quadratic_cost->c());
      } else {
        throw std::runtime_error("Only support quadratic costs for now.");
      }
    } else {
      // This cost has not been instantiated before.
      MathematicalProgram::AddCost(binding);
      cost_map_.emplace(cost, binding.evaluator());
    }
  }
  for (const Formula& constraint : constraints_) {
    Binding<Constraint> binding =
        internal::ParseConstraint(constraint.Substitute(s));
    auto it = constraint_map_.find(constraint);
    if (it != constraint_map_.end()) {
      // This constraint has been instantiated before. Need to update the
      // existing cost. For now, we only support quadratic costs.
      const auto existing_linear_constraint =
          std::dynamic_pointer_cast<LinearConstraint>(it->second);
      const auto new_linear_constraint =
          std::dynamic_pointer_cast<LinearConstraint>(binding.evaluator());
      if (existing_linear_constraint && new_linear_constraint) {
        existing_linear_constraint->UpdateCoefficients(
            new_linear_constraint->A(), new_linear_constraint->lower_bound(),
            new_linear_constraint->upper_bound());
      } else {
        throw std::runtime_error("Only support linear constraints for now.");
      }
    } else {
      // This constraint has not been instantiated before.
      MathematicalProgram::AddConstraint(binding);
      constraint_map_.emplace(constraint, binding.evaluator());
    }
  }
}

}  // namespace solvers
}  // namespace drake
