#include "drake/common/symbolic_interval_evaluation.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace drake {
namespace symbolic {

using std::accumulate;
using std::ostringstream;
using std::pair;
using std::runtime_error;

namespace {

// Visitor helper-class to implement DrealSolver::CheckSatisfiability and
// DrealSolver::Minimize.
class IntervalEvaluator {
 public:
  Interval Evaluate(const Expression& e, const IntervalEnvironment& env) const {
    return symbolic::VisitExpression<Interval>(this, e, env);
  }

 private:
  Interval VisitAddition(const Expression& e,
                         const IntervalEnvironment& env) const {
    const double c{get_constant_in_addition(e)};
    const auto& expr_to_coeff_map{get_expr_to_coeff_map_in_addition(e)};
    return accumulate(
        expr_to_coeff_map.begin(), expr_to_coeff_map.end(), Interval{c},
        [this, &env](const Interval& intv,
                     const pair<symbolic::Expression, double>& p) {
          return intv + p.second * Evaluate(p.first, env);
        });
  }
  Interval VisitMultiplication(const Expression& e,
                               const IntervalEnvironment& env) const {
    const double c{get_constant_in_multiplication(e)};
    const auto& base_to_exponent_map{
        get_base_to_exponent_map_in_multiplication(e)};
    return accumulate(
        base_to_exponent_map.begin(), base_to_exponent_map.end(), Interval{c},
        [this, &env](
            const Interval& intv,
            const pair<symbolic::Expression, symbolic::Expression>& p) {
          return intv * pow(Evaluate(p.first, env), Evaluate(p.second, env));
        });
  }
  Interval VisitDivision(const Expression& e,
                         const IntervalEnvironment& env) const {
    return Evaluate(get_first_argument(e), env) /
           Evaluate(get_second_argument(e), env);
  }
  Interval VisitVariable(const Expression& e,
                         const IntervalEnvironment& env) const {
    const Variable& var{get_variable(e)};
    auto it = env.find(var);
    if (it != env.end()) {
      return it->second;
    } else {
      ostringstream oss;
      oss << "IntervalEvaluator: Variable " << var
          << " is not found in a given environment.";
      throw runtime_error(oss.str());
    }
  }
  Interval VisitConstant(const Expression& e,
                         const IntervalEnvironment&) const {
    return Interval{get_constant_value(e)};
  }
  Interval VisitLog(const Expression& e, const IntervalEnvironment& env) const {
    return log(Evaluate(get_argument(e), env));
  }
  Interval VisitPow(const Expression& e, const IntervalEnvironment& env) const {
    return pow(Evaluate(get_first_argument(e), env),
               Evaluate(get_second_argument(e), env));
  }
  Interval VisitAbs(const Expression& e, const IntervalEnvironment& env) const {
    return abs(Evaluate(get_argument(e), env));
  }
  Interval VisitExp(const Expression& e, const IntervalEnvironment& env) const {
    return exp(Evaluate(get_argument(e), env));
  }
  Interval VisitSqrt(const Expression& e,
                     const IntervalEnvironment& env) const {
    return sqrt(Evaluate(get_argument(e), env));
  }
  Interval VisitSin(const Expression& e, const IntervalEnvironment& env) const {
    return sin(Evaluate(get_argument(e), env));
  }
  Interval VisitCos(const Expression& e, const IntervalEnvironment& env) const {
    return cos(Evaluate(get_argument(e), env));
  }
  Interval VisitTan(const Expression& e, const IntervalEnvironment& env) const {
    return tan(Evaluate(get_argument(e), env));
  }
  Interval VisitAsin(const Expression& e,
                     const IntervalEnvironment& env) const {
    return atan(Evaluate(get_argument(e), env));
  }
  Interval VisitAcos(const Expression& e,
                     const IntervalEnvironment& env) const {
    return acos(Evaluate(get_argument(e), env));
  }
  Interval VisitAtan(const Expression& e,
                     const IntervalEnvironment& env) const {
    return atan(Evaluate(get_argument(e), env));
  }
  Interval VisitAtan2(const Expression& e,
                      const IntervalEnvironment& env) const {
    return atan2(Evaluate(get_first_argument(e), env),
                 Evaluate(get_second_argument(e), env));
  }
  Interval VisitSinh(const Expression& e,
                     const IntervalEnvironment& env) const {
    return sinh(Evaluate(get_argument(e), env));
  }
  Interval VisitCosh(const Expression& e,
                     const IntervalEnvironment& env) const {
    return cosh(Evaluate(get_argument(e), env));
  }
  Interval VisitTanh(const Expression& e,
                     const IntervalEnvironment& env) const {
    return tanh(Evaluate(get_argument(e), env));
  }
  Interval VisitMin(const Expression& e, const IntervalEnvironment& env) const {
    return min(Evaluate(get_first_argument(e), env),
               Evaluate(get_second_argument(e), env));
  }
  Interval VisitMax(const Expression& e, const IntervalEnvironment& env) const {
    return max(Evaluate(get_first_argument(e), env),
               Evaluate(get_second_argument(e), env));
  }
  Interval VisitCeil(const Expression& e,
                     const IntervalEnvironment& env) const {
    throw 1;
  }
  Interval VisitFloor(const Expression& e,
                      const IntervalEnvironment& env) const {
    throw 1;
  }
  Interval VisitIfThenElse(const Expression& e,
                           const IntervalEnvironment& env) const {
    throw 1;
  }
  Interval VisitUninterpretedFunction(const Expression& e,
                                      const IntervalEnvironment& env) const {
    throw 1;
  }

  // Makes VisitExpression a friend of this class so that VisitExpression can
  // use its private methods.
  friend Interval VisitExpression<Interval>(const IntervalEvaluator*,
                                            const Expression&,
                                            const IntervalEnvironment&);
};
}  // namespace

Interval Evaluate(const Expression& e, const IntervalEnvironment& env) {
  return IntervalEvaluator{}.Evaluate(e, env);
}

}  // namespace symbolic
}  // namespace drake
