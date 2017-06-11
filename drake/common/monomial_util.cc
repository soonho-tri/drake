#include "drake/common/monomial_util.h"

#include <algorithm>
#include <map>
#include <memory>
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
using std::make_pair;
using std::map;
using std::ostream;
using std::ostringstream;
using std::out_of_range;
using std::pair;
using std::runtime_error;
using std::shared_ptr;
using std::unordered_map;

/**
 * Adds a term to the polynomial.
 * Find if the monomial in the new term exists in the polynomial or not. If it
 * does, then increment the corresponding coefficient. Otherwise add a new
 * pair (monomial, coefficient) to the map.
 * @param monomial The monomial in the new term.
 * @param coefficient The coefficient in the new term.
 * @param polynomial The polynomial that the new term is added to.
 */
void AddTermToPolynomial(const Monomial& monomial,
                         const Expression& coefficient,
                         MonomialToCoefficientMap* polynomial) {
  auto it = polynomial->find(monomial);
  if (it == polynomial->end()) {
    polynomial->emplace_hint(it, monomial, coefficient);
  } else {
    Expression new_coeff = it->second + coefficient;
    if (is_zero(new_coeff)) {
      polynomial->erase(it);
    } else {
      it->second = new_coeff;
    }
  }
}

/**
 * For a polynomial e = c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ), compute
 * the square of the polynomial.
 * Note that x and kᵢ can both be vectors, for example x = (x₀, x₁),
 * kᵢ = (1, 2), then pow(x, kᵢ) = x₀x₁²
 * @param map maps the monomial in the input polynomial to its coefficient.
 * @return maps the monomial in the output polynomial to its coefficient.
 */
MonomialToCoefficientMap PolynomialSqaure(const MonomialToCoefficientMap& map) {
  MonomialToCoefficientMap map_square;
  map_square.reserve(map.size() * (map.size() + 1) / 2);
  for (auto it1 = map.begin(); it1 != map.end(); ++it1) {
    for (auto it2 = it1; it2 != map.end(); ++it2) {
      Monomial new_monomial = it1->first * it2->first;
      Expression new_coeff = it1->second * it2->second;
      if (it1 != it2) {
        // Two cross terms.
        new_coeff *= 2;
      }
      AddTermToPolynomial(new_monomial, new_coeff, &map_square);
    }
  }
  return map_square;
}

class DecomposePolynomialVisitor {
 public:
  // `vars` is a const set of variables, that will not be changed. It stays
  // the same during visiting each type of symbolic expressions.
  MonomialToCoefficientMap Visit(const Expression& e,
                                 const Variables& vars) const {
    return VisitPolynomial<MonomialToCoefficientMap>(this, e, vars);
  }

 private:
  MonomialToCoefficientMap VisitVariable(const Expression& e,
                                         const Variables& vars) const {
    const Variable& var{get_variable(e)};
    Expression coeff{};
    int exponent{};
    if (vars.include(var)) {
      exponent = 1;
      coeff = 1;
    } else {
      exponent = 0;
      coeff = var;
    }
    return MonomialToCoefficientMap({{Monomial(var, exponent), coeff}});
  }

  MonomialToCoefficientMap VisitConstant(const Expression& e,
                                         const Variables&) const {
    const double v{get_constant_value(e)};
    if (v != 0) {
      return MonomialToCoefficientMap({{Monomial(), v}});
    }
    return MonomialToCoefficientMap();
  }

  MonomialToCoefficientMap VisitAddition(const Expression& e,
                                         const Variables& vars) const {
    MonomialToCoefficientMap map;
    const double e_constant{get_constant_in_addition(e)};
    if (e_constant != 0) {
      map.emplace(Monomial(), e_constant);
    }
    // For an expression 2*(3*x*y+4*x) + 4*y.
    // expr_to_coeff_map_[3*x*y+4*x] = 2
    // expr_to_coeff_map_[y] = 4
    for (const auto& p : get_expr_to_coeff_map_in_addition(e)) {
      // For expr_to_coeff_map_[3*x*y+4*x] = 2
      // map_p[x*y] = 3
      // map_p[x] = 4
      const auto& map_p = Visit(p.first, vars);
      for (const auto& map_p_pair : map_p) {
        const Monomial& p_monomial = map_p_pair.first;
        const Expression& p_coefficient = map_p_pair.second;
        // a * (b * monomial) = (a * b) * monomial.
        AddTermToPolynomial(p_monomial, p_coefficient * p.second, &map);
      }
    }
    return map;
  }

  MonomialToCoefficientMap VisitMultiplication(const Expression& e,
                                               const Variables& vars) const {
    MonomialToCoefficientMap map;
    // We iterate through base_to_exponent_map
    // Suppose e = pow(e₁, p₁) * pow(e₂, p₂) * ... * pow(eₖ, pₖ)
    // We first decompose the first k-1 products
    // pow(e₁, p₁) * pow(e₂, p₂) * ... * pow(eₖ₋₁, pₖ₋₁) as a polynomial
    // (c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ))
    // and the last term pow(eₖ, pₖ) as another polynomial
    // (d₀ + d₁ * pow(x, k₁) + ... + dₘ * pow(x, kₘ))
    // And then multiply the term cᵢ * pow(x, kᵢ) with the term dⱼ * pow(x,
    // kⱼ) to get each monomial in the product.
    // TODO(hongkai.dai):
    // Alternatively, we can do divide and conquer here.
    // for an expression e = pow(e₁, p₁) * pow(e₂, p₂) * ... * pow(eₖ, pₖ)
    // First decomposes the first ⌊k/2⌋ products, then decomposes the last
    // ⌈k/2⌉ products, so as to write e as the product of two polynomials
    // e = (c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ)) *
    //     (d₀ + d₁ * pow(x, k₁) + ... + dₘ * pow(x, kₘ))
    // Finally multiply the term cᵢ * pow(x, kᵢ) with the term dⱼ * pow(x, kⱼ)
    // to get each monomial in the product.
    // The divide and conquer approach requires splitting the map
    // base_to_exponent_map to two halves, which can be inefficient, so we do
    // not implement this approach.
    const auto& base_to_exponent_map =
        get_base_to_exponent_map_in_multiplication(e);
    for (const auto& p : base_to_exponent_map) {
      if (map.empty()) {
        map = Visit(pow(p.first, p.second), vars);
      } else {
        const auto& map_p = Visit(pow(p.first, p.second), vars);
        MonomialToCoefficientMap map_product;
        map_product.reserve(map.size() * map_p.size());
        // Now multiply each term in map, with each term in map_p.
        for (const auto& term_map : map) {
          for (const auto& term_map_p : map_p) {
            Monomial new_monomial = term_map.first * term_map_p.first;
            Expression new_coeff = term_map.second * term_map_p.second;
            AddTermToPolynomial(new_monomial, new_coeff, &map_product);
          }
        }
        map = std::move(map_product);
      }
    }
    // Finally multiply the constant coefficient.
    for (auto& p : map) {
      p.second *= get_constant_in_multiplication(e);
    }
    return map;
  }

  MonomialToCoefficientMap VisitPow(const Expression& e,
                                    const Variables& vars) const {
    // We use a divide and conquer approach here
    // pow(e, p) can be computed as pow(e, ⌊p/2⌋) * pow(e, ⌈p/2⌉)
    // We can decompose the first term as a polynomial
    // (c₀ + c₁ * pow(x, k₁) + ... + cₙ * pow(x, kₙ))
    // and the second term as another polynomial
    // (d₀ + d₁ * pow(x, k₁) + ... + dₘ * pow(x, kₘ))
    // We then multiply the term cᵢ * pow(x, kᵢ) with the term dⱼ * pow(x, kⱼ)
    // to get each monomial in the product.
    MonomialToCoefficientMap map;
    const Expression& first_arg{get_first_argument(e)};
    const Expression& second_arg{get_second_argument(e)};
    const int exponent{static_cast<int>(get_constant_value(second_arg))};
    if (exponent == 1) {
      return Visit(first_arg, vars);
    }
    if (exponent % 2 == 0) {
      // compute the square of a polynomial (c₀ + c₁ * pow(x, k₁) + ... + cₙ *
      // pow(x, kₙ)).
      const auto& map1 = Visit(pow(first_arg, exponent / 2), vars);
      map = PolynomialSqaure(map1);
    } else {
      // For expression pow(e, k) with odd exponent k, compute
      // e1 = pow(e, ⌊k/2⌋) first, and then compute the square of e1, finally
      // multiply the squared result with e.
      const auto& map1 = Visit(pow(first_arg, exponent / 2), vars);
      const auto& map1_square = PolynomialSqaure(map1);
      const auto& map2 = Visit(first_arg, vars);
      map.reserve(map1_square.size() * map2.size());
      for (const auto& p1 : map1_square) {
        for (const auto& p2 : map2) {
          Monomial new_monomial = p1.first * p2.first;
          Expression new_coeff = p1.second * p2.second;
          AddTermToPolynomial(new_monomial, new_coeff, &map);
        }
      }
    }
    return map;
  }

  MonomialToCoefficientMap VisitDivision(const Expression& e,
                                         const Variables& vars) const {
    const Expression& first_arg{get_first_argument(e)};
    const Expression& second_arg{get_second_argument(e)};

    // Currently we can only handle the case of a monomial as the divisor.
    const auto& map1 = Visit(first_arg, vars);
    const auto& map2 = Visit(second_arg, vars);
    if (map2.size() != 1) {
      throw std::runtime_error(
          "The divisor is not a monomial. The Div expression cannot be "
          "decomposed as a polynomial.");
    }
    const auto& divisor_monomial = map2.begin()->first;
    const auto& divisor_monomial_powers = divisor_monomial.get_powers();
    const Expression& divisor_coeff = map2.begin()->second;
    MonomialToCoefficientMap map;
    map.reserve(map1.size());
    for (const auto& p1 : map1) {
      // For each monomial in the dividend, compute the division from the
      // dividend monomial by the divisor monomial.
      const Monomial dividend_monomial(p1.first);
      std::map<Variable, int> division_monomial_powers =
          dividend_monomial.get_powers();
      for (const auto& p_divisor : divisor_monomial_powers) {
        // The variable in divisor has to appear in the dividend.
        auto it = division_monomial_powers.find(p_divisor.first);
        if (it == division_monomial_powers.end()) {
          throw std::runtime_error(
              "The variable in the divisor is not in the dividend.");
        } else {
          // xⁿ / xᵐ = xⁿ⁻ᵐ
          it->second -= p_divisor.second;
        }
      }
      Monomial division_monomial(division_monomial_powers);
      map.emplace(division_monomial, p1.second / divisor_coeff);
    }
    return map;
  }

  // Makes VisitPolynomial a friend of this class so that it can use private
  // methods.
  friend MonomialToCoefficientMap
  drake::symbolic::VisitPolynomial<MonomialToCoefficientMap>(
      const DecomposePolynomialVisitor*, const Expression&, const Variables&);
};

MonomialToCoefficientMap DecomposePolynomialIntoMonomial(
    const Expression& e, const Variables& vars) {
  DRAKE_DEMAND(e.is_polynomial());
  MonomialToCoefficientMap map = DecomposePolynomialVisitor().Visit(e, vars);
  // Now loops through the map to remove the term with zero coefficient.
  for (auto it = map.begin(); it != map.end();) {
    bool is_zero_term = false;
    DRAKE_DEMAND(it->second.is_polynomial());
    if (!is_constant(it->second)) {
      // If the coefficient it->second is a polynomial, then determine if it
      // is a zero polynomial, by decomposing it->second into monomials,
      // and check if the constant coefficient for each term is zero.
      MonomialToCoefficientMap coeff_map = DecomposePolynomialVisitor().Visit(
          it->second, it->second.GetVariables());
      is_zero_term = true;
      for (const auto& p : coeff_map) {
        DRAKE_DEMAND(is_constant(p.second));
        if (!is_zero(p.second)) {
          is_zero_term = false;
          break;
        }
      }
    } else {
      // If the coefficient is a constant, then it cannot be zero, since we
      // have deleted term with zero constant coefficient already.
      DRAKE_ASSERT(!is_zero(it->second));
    }
    if (is_zero_term) {
      it = map.erase(it);
    } else {
      ++it;
    }
  }
  return map;
}

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
      degree = std::max(degree, Visit(p.first, vars));
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

Expression GetMonomial(const unordered_map<Variable, int, hash_value<Variable>>&
                           map_var_to_exponent) {
  map<Expression, Expression> base_to_exponent_map;
  for (const auto& p : map_var_to_exponent) {
    DRAKE_DEMAND(p.second > 0);
    base_to_exponent_map.emplace(Expression{p.first}, p.second);
  }
  return ExpressionMulFactory{1.0, base_to_exponent_map}.GetExpression();
}

Eigen::Matrix<Monomial, Eigen::Dynamic, 1> MonomialBasis(const Variables& vars,
                                                         const int degree) {
  return ComputeMonomialBasis<Eigen::Dynamic>(vars, degree);
}

MonomialAsExpressionToCoefficientMap DecomposePolynomialIntoExpression(
    const Expression& e, const Variables& vars) {
  const auto& map_internal = DecomposePolynomialIntoMonomial(e, vars);
  MonomialAsExpressionToCoefficientMap map;
  map.reserve(map_internal.size());
  for (const auto& p : map_internal) {
    map.emplace(p.first.ToExpression(), p.second);
  }
  return map;
}

MonomialAsExpressionToCoefficientMap DecomposePolynomialIntoExpression(
    const Expression& e) {
  return DecomposePolynomialIntoExpression(e, e.GetVariables());
}

}  // namespace symbolic
}  // namespace drake
