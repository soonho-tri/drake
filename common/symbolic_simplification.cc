#include "drake/common/symbolic_simplification.h"

#include <stdexcept>
#include <utility>

namespace drake {
namespace symbolic {

using std::pair;
using std::runtime_error;

Expression SinSquare(const Expression& e) {
  if (is_pow(e)) {
    const Expression& base = get_first_argument(e);
    const Expression& exponent = get_second_argument(e);
    if (is_sin(base) && is_constant(exponent) &&
        (get_constant_value(exponent) == 2.0)) {
      const Expression& arg = get_argument(base);
      return 1 - pow(cos(arg), 2);
    }
  }
  return e;
}

Expression CosSquare(const Expression& e) {
  if (is_pow(e)) {
    const Expression& base = get_first_argument(e);
    const Expression& exponent = get_second_argument(e);
    if (is_constant(exponent) && (get_constant_value(exponent) == 2.0)) {
      if (is_cos(base)) {
        const Expression& arg = get_argument(base);
        return 1 - pow(sin(arg), 2);
      }
    }
  }
  return e;
}

Expression SinSumOfAngle(const Expression& e) {
  // sin(X + Y) => sin(X)cos(Y) + cos(X)sin(Y)
  if (is_sin(e)) {
    const Expression& arg = get_argument(e);
    if (is_addition(arg)) {
      const double c0 = get_constant_in_addition(arg);
      const auto expr_to_coeff_map = get_expr_to_coeff_map_in_addition(arg);
      if (c0 == 0.0 && expr_to_coeff_map.size() <= 1) {
        // This is sin(X) case.
        return e;
      }
      Expression x;
      if (c0 != 0.0) {
        // This is sin(c₀ + (∑cᵢeᵢ)) case.
        x = c0;
      } else {
        // This is sin(c₁e₁ + (∑_{i>=2} cᵢeᵢ)) case.
        x = expr_to_coeff_map.begin()->first *
            expr_to_coeff_map.begin()->second;
      }
      const Expression y = arg - x;
      return sin(x) * CosSumOfAngle(cos(y)) + cos(x) * SinSumOfAngle(sin(y));
    }
  }
  return e;
}

Expression CosSumOfAngle(const Expression& e) {
  // cos(X + Y) => cos(X)cos(Y) - sin(X)sin(Y)
  if (is_cos(e)) {
    const Expression& arg = get_argument(e);
    if (is_addition(arg)) {
      const double c0 = get_constant_in_addition(arg);
      const auto expr_to_coeff_map = get_expr_to_coeff_map_in_addition(arg);
      if (c0 == 0.0 && expr_to_coeff_map.size() <= 1) {
        // This is cos(X) case.
        return e;
      }
      Expression x;
      if (c0 != 0.0) {
        // This is cos(c₀ + (∑cᵢeᵢ)) case.
        x = c0;
      } else {
        // This is sin(c₁e₁ + (∑_{i>=2} cᵢeᵢ)) case.
        x = expr_to_coeff_map.begin()->first *
            expr_to_coeff_map.begin()->second;
      }
      const Expression y = arg - x;
      return cos(x) * CosSumOfAngle(cos(y)) - sin(x) * SinSumOfAngle(sin(y));
    }
  }
  return e;
}

namespace {

class CongruenceVisitor {
 public:
  explicit CongruenceVisitor(const Pattern& pattern);
  Expression Visit(const Expression& e) const;

 private:
  Expression VisitVariable(const Expression& e) const;
  Expression VisitConstant(const Expression& e) const;
  Expression VisitAddition(const Expression& e) const;
  Expression VisitMultiplication(const Expression& e) const;
  Expression VisitPow(const Expression& e) const;
  Expression VisitDivision(const Expression& e) const;
  Expression VisitAbs(const Expression& e) const;
  Expression VisitLog(const Expression& e) const;
  Expression VisitExp(const Expression& e) const;
  Expression VisitSqrt(const Expression& e) const;
  Expression VisitSin(const Expression& e) const;
  Expression VisitCos(const Expression& e) const;
  Expression VisitTan(const Expression& e) const;
  Expression VisitAsin(const Expression& e) const;
  Expression VisitAcos(const Expression& e) const;
  Expression VisitAtan(const Expression& e) const;
  Expression VisitAtan2(const Expression& e) const;
  Expression VisitSinh(const Expression& e) const;
  Expression VisitCosh(const Expression& e) const;
  Expression VisitTanh(const Expression& e) const;
  Expression VisitMin(const Expression& e) const;
  Expression VisitMax(const Expression& e) const;
  Expression VisitCeil(const Expression& e) const;
  Expression VisitFloor(const Expression& e) const;
  Expression VisitIfThenElse(const Expression& e) const;
  Expression VisitUninterpretedFunction(const Expression& e) const;

  // Makes VisitExpression a friend of this class so that it can use private
  // methods.
  friend Expression VisitExpression<Expression>(const CongruenceVisitor*,
                                                const Expression&);

  const Pattern pattern_;
};

CongruenceVisitor::CongruenceVisitor(const Pattern& pattern)
    : pattern_{pattern} {}

Expression CongruenceVisitor::Visit(const Expression& e) const {
  return VisitExpression<Expression>(this, e);
}

Expression CongruenceVisitor::VisitVariable(const Expression& e) const {
  return pattern_(e);
}

Expression CongruenceVisitor::VisitConstant(const Expression& e) const {
  return pattern_(e);
}

Expression CongruenceVisitor::VisitAddition(const Expression& e) const {
  Expression new_e = pattern_(get_constant_in_addition(e));
  for (const pair<const Expression, double>& p :
       get_expr_to_coeff_map_in_addition(e)) {
    const Expression& e_i = p.first;
    const double coeff_i = p.second;
    new_e += pattern_(Visit(e_i) * pattern_(coeff_i));
  }
  return pattern_(new_e);
}

Expression CongruenceVisitor::VisitMultiplication(const Expression& e) const {
  Expression new_e = pattern_(get_constant_in_multiplication(e));
  for (const pair<const Expression, Expression>& p :
       get_base_to_exponent_map_in_multiplication(e)) {
    const Expression& base_i = p.first;
    const Expression& exponent_i = p.second;
    new_e *= pattern_(pow(Visit(base_i), Visit(exponent_i)));
  }
  return pattern_(new_e);
}

Expression BinaryCase(
    const Expression& e, const Pattern& pattern,
    const std::function<Expression(const Expression&, const Expression&)>&
        binary_function) {
  const Expression& arg1{get_first_argument(e)};
  const Expression& arg2{get_second_argument(e)};
  return pattern(binary_function(pattern(arg1), pattern(arg2)));
}

Expression UnaryCase(
    const Expression& e, const Pattern& pattern,
    const std::function<Expression(const Expression&)>& unary_function) {
  const Expression& arg{get_argument(e)};
  return pattern(unary_function(pattern(arg)));
}

Expression CongruenceVisitor::VisitPow(const Expression& e) const {
  return BinaryCase(e, pattern_,
                    [](const Expression& arg1, const Expression& arg2) {
                      return pow(arg1, arg2);
                    });
}

Expression CongruenceVisitor::VisitDivision(const Expression& e) const {
  return BinaryCase(e, pattern_,
                    [](const Expression& arg1, const Expression& arg2) {
                      return arg1 / arg2;
                    });
}

Expression CongruenceVisitor::VisitAbs(const Expression& e) const {
  return UnaryCase(e, pattern_, [](const Expression& arg) { return abs(arg); });
}

Expression CongruenceVisitor::VisitLog(const Expression& e) const {
  return UnaryCase(e, pattern_, [](const Expression& arg) { return log(arg); });
}

Expression CongruenceVisitor::VisitExp(const Expression& e) const {
  return UnaryCase(e, pattern_, [](const Expression& arg) { return exp(arg); });
}

Expression CongruenceVisitor::VisitSqrt(const Expression& e) const {
  return UnaryCase(e, pattern_,
                   [](const Expression& arg) { return sqrt(arg); });
}

Expression CongruenceVisitor::VisitSin(const Expression& e) const {
  return UnaryCase(e, pattern_, [](const Expression& arg) { return sin(arg); });
}

Expression CongruenceVisitor::VisitCos(const Expression& e) const {
  return UnaryCase(e, pattern_, [](const Expression& arg) { return cos(arg); });
}

Expression CongruenceVisitor::VisitTan(const Expression& e) const {
  return UnaryCase(e, pattern_, [](const Expression& arg) { return tan(arg); });
}

Expression CongruenceVisitor::VisitAsin(const Expression& e) const {
  return UnaryCase(e, pattern_,
                   [](const Expression& arg) { return asin(arg); });
}

Expression CongruenceVisitor::VisitAcos(const Expression& e) const {
  return UnaryCase(e, pattern_,
                   [](const Expression& arg) { return acos(arg); });
}

Expression CongruenceVisitor::VisitAtan(const Expression& e) const {
  return UnaryCase(e, pattern_,
                   [](const Expression& arg) { return atan(arg); });
}

Expression CongruenceVisitor::VisitAtan2(const Expression& e) const {
  return BinaryCase(e, pattern_,
                    [](const Expression& arg1, const Expression& arg2) {
                      return atan2(arg1, arg2);
                    });
}

Expression CongruenceVisitor::VisitSinh(const Expression& e) const {
  return UnaryCase(e, pattern_,
                   [](const Expression& arg) { return sinh(arg); });
}

Expression CongruenceVisitor::VisitCosh(const Expression& e) const {
  return UnaryCase(e, pattern_,
                   [](const Expression& arg) { return cosh(arg); });
}

Expression CongruenceVisitor::VisitTanh(const Expression& e) const {
  return UnaryCase(e, pattern_,
                   [](const Expression& arg) { return tanh(arg); });
}

Expression CongruenceVisitor::VisitMin(const Expression& e) const {
  return BinaryCase(e, pattern_,
                    [](const Expression& arg1, const Expression& arg2) {
                      return min(arg1, arg2);
                    });
}

Expression CongruenceVisitor::VisitMax(const Expression& e) const {
  return BinaryCase(e, pattern_,
                    [](const Expression& arg1, const Expression& arg2) {
                      return max(arg1, arg2);
                    });
}

Expression CongruenceVisitor::VisitCeil(const Expression& e) const {
  return UnaryCase(e, pattern_,
                   [](const Expression& arg) { return ceil(arg); });
}

Expression CongruenceVisitor::VisitFloor(const Expression& e) const {
  return UnaryCase(e, pattern_,
                   [](const Expression& arg) { return floor(arg); });
}

Expression CongruenceVisitor::VisitIfThenElse(const Expression& e) const {
  // TODO(soonho-tri): Support this.
  throw runtime_error(
      "CongruenceVisitor does not support IfThenElse expressions yet.");
}

Expression CongruenceVisitor::VisitUninterpretedFunction(
    const Expression& e) const {
  return pattern_(e);
}

}  // namespace

Pattern MakeCongruence(const Pattern& pattern) {
  return [visitor = CongruenceVisitor{pattern}](const Expression& e) {
    return visitor.Visit(e);
  };
}

Pattern MakeTry(const Pattern& pattern1, const Pattern& pattern2) {
  return [pattern1, pattern2](const Expression& e) {
    const Expression result1 = pattern1(e);
    if (!e.EqualTo(result1)) {
      return result1;
    } else {
      return pattern2(e);
    }
  };
}

Pattern MakeFixpoint(const Pattern& pattern) {
  return [pattern](Expression e) {
    while (true) {
      Expression old_e = e;
      e = pattern(e);
      if (old_e == e) {
        return e;
      }
    }
  };
}

}  // namespace symbolic
}  // namespace drake
