#include "drake/common/monomial_util.h"

#include <algorithm>
#include <map>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_expression_cell.h"
#include "drake/common/symbolic_expression_visitor.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/symbolic_variables.h"

namespace drake {
namespace symbolic {

using std::accumulate;
using std::map;
using std::max;
using std::move;
using std::pair;
using std::runtime_error;
using std::unordered_map;

// /**
//  * Adds a term to the polynomial.
//  * Find if the monomial in the new term exists in the polynomial or not. If
//  it * does, then increment the corresponding coefficient. Otherwise add a new
//  * pair (monomial, coefficient) to the map.
//  * @param monomial The monomial in the new term.
//  * @param coefficient The coefficient in the new term.
//  * @param polynomial The polynomial that the new term is added to.
//  */
// void AddTermToPolynomial(const Monomial& monomial,
//                          const Expression& coefficient,
//                          MonomialToCoefficientMap* polynomial) {
//   auto it = polynomial->find(monomial);
//   if (it == polynomial->end()) {
//     polynomial->emplace_hint(it, monomial, coefficient);
//   } else {
//     Expression new_coeff = it->second + coefficient;
//     if (is_zero(new_coeff)) {
//       polynomial->erase(it);
//     } else {
//       it->second = new_coeff;
//     }
//   }
// }

// /**
//  * For a polynomial e = c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ),
//  * compute the square of the polynomial.
//  * Note that x and kᵢ can both be vectors, for example x = (x₀, x₁),
//  * kᵢ = (1, 2), then pow(x, kᵢ) = x₀x₁²
//  * @param map maps the monomial in the input polynomial to its coefficient.
//  * @return maps the monomial in the output polynomial to its coefficient.
//  */
// MonomialToCoefficientMap PolynomialSqaure(const MonomialToCoefficientMap&
// map) {
//   MonomialToCoefficientMap map_square;
//   map_square.reserve(map.size() * (map.size() + 1) / 2);
//   for (auto it1 = map.begin(); it1 != map.end(); ++it1) {
//     for (auto it2 = it1; it2 != map.end(); ++it2) {
//       Monomial new_monomial = it1->first * it2->first;
//       Expression new_coeff = it1->second * it2->second;
//       if (it1 != it2) {
//         // Two cross terms.
//         new_coeff *= 2;
//       }
//       AddTermToPolynomial(new_monomial, new_coeff, &map_square);
//     }
//   }
//   return map_square;
// }

class DegreeVisitor {
 public:
  int Visit(const Expression& e, const Variables& vars) const {
    return VisitPolynomial<int>(this, e, vars);
  }

 private:
  int VisitVariable(const Expression& e, const Variables& vars) const {
    return vars.include(get_variable(e)) ? 1 : 0;
  }

  int VisitConstant(const Expression&, const Variables&) const { return 0; }

  int VisitAddition(const Expression& e, const Variables& vars) const {
    int degree = 0;
    for (const auto& p : get_expr_to_coeff_map_in_addition(e)) {
      degree = max(degree, Visit(p.first, vars));
    }
    return degree;
  }

  int VisitMultiplication(const Expression& e, const Variables& vars) const {
    const auto& base_to_exponent_map =
        get_base_to_exponent_map_in_multiplication(e);
    return accumulate(
        base_to_exponent_map.begin(), base_to_exponent_map.end(), 0,
        [this, &vars](const int& degree,
                      const pair<Expression, Expression>& p) {
          const Expression& base{p.first};
          const Expression& exponent{p.second};
          return degree + Visit(base, vars) *
                              static_cast<int>(get_constant_value(exponent));
        });
  }

  int VisitDivision(const Expression& e, const Variables& vars) const {
    return Visit(get_first_argument(e), vars) -
           Visit(get_second_argument(e), vars);
  }

  int VisitPow(const Expression& e, const Variables& vars) const {
    const int exponent{
        static_cast<int>(get_constant_value(get_second_argument(e)))};
    return Visit(get_first_argument(e), vars) * exponent;
  }

  // Makes VisitPolynomial a friend of this class so that it can use private
  // methods.
  friend int drake::symbolic::VisitPolynomial<int>(const DegreeVisitor*,
                                                   const Expression&,
                                                   const Variables&);
};

int Degree(const Expression& e, const Variables& vars) {
  return DegreeVisitor().Visit(e, vars);
}

int Degree(const Expression& e) { return Degree(e, e.GetVariables()); }

// Expression GetMonomial(const unordered_map<Variable, int,
// hash_value<Variable>>&
//                            map_var_to_exponent) {
//   map<Expression, Expression> base_to_exponent_map;
//   for (const auto& p : map_var_to_exponent) {
//     DRAKE_DEMAND(p.second > 0);
//     base_to_exponent_map.emplace(Expression{p.first}, p.second);
//   }
//   return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
// }

Eigen::Matrix<Monomial, Eigen::Dynamic, 1> MonomialBasis(const Variables& vars,
                                                         const int degree) {
  return ComputeMonomialBasis<Eigen::Dynamic>(vars, degree);
}

// MonomialAsExpressionToCoefficientMap DecomposePolynomialIntoExpression(
//     const Expression& e, const Variables& vars) {
//   const auto& map_internal = DecomposePolynomialIntoMonomial(e, vars);
//   MonomialAsExpressionToCoefficientMap map;
//   map.reserve(map_internal.size());
//   for (const auto& p : map_internal) {
//     map.emplace(p.first.ToExpression(), p.second);
//   }
//   return map;
// }

// MonomialAsExpressionToCoefficientMap DecomposePolynomialIntoExpression(
//     const Expression& e) {
//   return DecomposePolynomialIntoExpression(e, e.GetVariables());
// }

}  // namespace symbolic
}  // namespace drake
