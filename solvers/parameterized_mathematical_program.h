#pragma once

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/binding.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

/// This class extends drake::solvers::MathematicalProgram by introducing
/// symbolic parameters, which can be instantiated with a given substitution (a
/// mapping from a symbolic variable to a double value).
class ParameterizedMathematicalProgram : public MathematicalProgram {
 public:
  /// Returns the number of parameters.
  int num_parameters() const;

  /// Returns parameters_index, a mapping from a variable ID to its parameter
  /// index in this program.
  const std::unordered_map<symbolic::Variable::Id, int>& parameters_index()
      const {
    return parameters_index_;
  }

  /// Adds parameters @p new_parameters to this program.
  void AddParameters(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& new_parameters);

  /// Creates a new vector of parameters and adds them to this program.
  template <int rows>
  Vector<symbolic::Variable, rows> NewParameters(
      const std::string& name = "p") {
    int offset = (name.compare("p") == 0) ? num_parameters() : 0;
    Vector<symbolic::Variable, rows> ret;
    for (int i = 0; i < rows; ++i) {
      ret[i] =
          symbolic::Variable{name + "(" + std::to_string(offset + i) + ")"};
    }
    AddParameters(ret);
    return ret;
  }

  /// Adds constraint @p f into this program.
  void AddConstraint(const symbolic::Formula& f);

  /// Adds cost @p e into this program.
  void AddCost(const symbolic::Expression& e);

  /// Instantiates the symbolic parameters in this program using the
  /// substitution @p s.
  void InstantiateParameters(const symbolic::Substitution& s);

 private:
  // Returns true if @p variables includes a parameter in this program.
  bool IncludeParameter(const symbolic::Variables& variables) const;

  std::vector<symbolic::Expression> costs_;
  std::vector<symbolic::Formula> constraints_;
  std::unordered_map<symbolic::Expression, std::shared_ptr<Cost>> cost_map_;
  std::unordered_map<symbolic::Formula, std::shared_ptr<Constraint>>
      constraint_map_;

  VectorX<symbolic::Variable> parameters_;
  std::unordered_map<symbolic::Variable::Id, int> parameters_index_;
};
}  // namespace solvers
}  // namespace drake
