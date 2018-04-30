#include "drake/common/symbolic_simplification.h"

#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace symbolic {

using std::function;
using std::map;
using std::runtime_error;

namespace {

// Implements a first-order unification algorithm.
//
// For more information, read Section 8.2. Syntactic unification of [1].
//
// [1] Baader, F., & Snyder, W. (2001). Unification Theory. Handbook of
//     Automated Reasoning.
//     URL: https://www.cs.bu.edu/~snyder/publications/UnifChapter.pdf
class UnificationVisitor {
 public:
  // Matches the expression @p e with the pattern @p p and tries to find a
  // substitution which will transform the pattern @p p into the expression @p e
  // when applied. Returns a substitution if found. Otherwise, returns a
  // `nullopt`.
  //
  // Consider the following example:
  //
  //     Pattern:    p = sin(x) * cos(y)
  //     Expression: e = sin(a + b) * cos(c).
  //
  // `Unify(p, e)` returns a substitution `{x ↦ (a + b), y ↦ c}`. Note that
  // applying this substitution to the pattern `p = sin(x)*cos(y)`, we have the
  // expression `e = sin(a+b)*cos(c)`.
  //
  // Consider another example:
  //
  //     Pattern:    p = sin(x * y)
  //     Expression: e = sin(a + b)
  //
  // In this case, there is no way to match `e` with `p` because `a + b` is not
  // matched with `x * y`.  Therefore, `Unify(p, e)` returns `nullopt`.
  optional<Substitution> Unify(const Pattern& p, const Expression& e) const {
    Substitution subst;
    if (Unify(p, e, &subst)) {
      return subst;
    } else {
      return nullopt;
    }
  }

 private:
  // It visits the pattern `p` and the expression `e` recursively and updates
  // the output parameter `subst`.
  //
  // Returns `false` if it fails to match `e` and `p`. Otherwise, returns true.
  bool Unify(const Pattern& p, const Expression& e,
             Substitution* const subst) const {
    return VisitExpression<bool>(this, p, e, subst);
  }

  bool VisitVariable(const Pattern& p, const Expression& e,
                     Substitution* const subst) const {
    // Case: `p` is a variable `v`.
    //
    // We need to update `subst` to include `v ↦ e`. It fails if `subst`
    // already includes an entry `v` but `subst[v]` is not `e`.
    const Variable& v{get_variable(p)};
    const auto it = subst->find(v);
    if (it != subst->end()) {
      return e.EqualTo(it->second);
    }
    subst->emplace(v, e);
    return true;
  }

  bool VisitConstant(const Pattern& p, const Expression& e,
                     Substitution* const) const {
    // Case: `p` is a constant `c`.
    //
    // `e` should be `c` for this unification to be successful.
    return is_constant(e) && (get_constant_value(e) == get_constant_value(p));
  }

  // Visits the pattern @p p and the expression @p e, updates the substitution
  // @p subst, and returns true if @p p and @p e are matched. Otherwise, returns
  // false without modifying the substitution @p subst.
  //
  // This code handles the case where @p p is an addition expression. If the
  // expression @p e is not an addition, the unification fails immediately.
  //
  //
  // Internal Representation of addition expressions
  // -----------------------------------------------
  //
  // An addition expression is represented as a pair of its constant coefficient
  // (: double) and an ordered map from a term (: Expression) to its coefficient
  // (: double). For example, `3 + 2x + 3x²` is represented as
  //
  //     constant coefficient: 3
  //     ordered map: {x ↦ 2, x² ↦ 3}.
  //
  // Note that there exists a zero-coefficient even if it is not explicitly
  // provided by a user. For example, `a + b` is represented internally as
  //
  //     constant coefficient: 0
  //     ordered map: {a ↦ 1, b ↦ 1}.
  //
  //
  // Unification (First attempt)
  // ---------------------------
  //
  // Let the pattern @p p and the expression @p e be the summations of terms as
  // follows:
  //
  //     p = c₀ + c₁t₁ + ... cₙtₙ.
  //     e = c'₀ + c'₁t'₁ + ... c'ₘt'ₘ.
  //
  // Now, we derive the condition for the pattern @p p and the expression @p e
  // to be matched.
  //
  // 1) We require that the expression should be longer than the pattern to be
  //    matched. That is,
  //
  //     n ≤ m.
  //
  // 2) The constant parts of the pattern and the expression should be the
  //    same. That is,
  //
  //     c₀ = c'₀.
  //
  // 3) Each summand of the pattern and the expression should be matched except
  //    for the last one. That is,
  //
  //     Unify(cᵢtᵢ, c'ᵢtᵢ) for all i ∈ [1, n-1].
  //
  // 4) Finally, the last summand in the pattern, cₙtₙ, and the rest of
  //    unmatched summands in the expression, c'ₙt'ₙ + ... + c'ₘt'ₘ should
  //    be matched. That is,
  //
  //     Unify(cₙtₙ, c'ₙt'ₙ + ... + c'ₘt'ₘ).
  //
  //
  // Problem of the first attempt and an extension to the algorithm
  // --------------------------------------------------------------
  //
  // Let's say that a user wants to match a pattern, p = a + b, and an
  // expression, 2 + x. There exists a substitution, {a ↦ 2, b ↦ x}, which
  // unifies the two. However, the above algorithm failes to find this
  // substitution. This is because the pattern, p = a + b, is represented as `p
  // = 0 + a + b`. As a result, the algorithm rejects (p, e) because e is not as
  // long as e.
  //
  // A solution to this problem is to do the case analysis:
  //
  //  1. Zero lead-coeff in Pattern + Zero lead-coeff in Expression.
  //      Pattern:    c₁t₁   + ... + cₙtₙ
  //      Expression: c'₁t'₁ + ... + c'ₘt'ₘ
  //
  //      => Unify(cᵢtᵢ, c'ᵢt'ᵢ) for all i ∈ [1, n-1] and match the rest.
  //
  //     Example:     p = a + b
  //                  e = x + y + z
  //             Result = match with {a ↦ x, b ↦ y + z}
  //
  //  2. Zero lead-coeff in Pattern + Non-zero lead-coeff in Expression.
  //      Pattern:          c₁t₁   + ... + cₙtₙ
  //      Expression: c'₀ + c'₁t'₁ + ... + c'ₘt'ₘ
  //
  //      => Unify(cᵢtᵢ, c'_{i-1}t'_{i-1}) for all i ∈ [1, n-1] and match the
  //         rest (where t'₀ = 1.0).
  //
  //     Example:     p = a + b
  //                  e = 1 + x + y + z
  //             Result = match with {a ↦ 1, b ↦ x + y + z}
  //
  //  3. Non-zero lead-coeff in Pattern + Zero lead-coeff in Expression.
  //      Pattern:    c₀ + c₁t₁   + ... + cₙtₙ
  //      Expression:      c'₁t'₁ + ... + c'ₘt'ₘ
  //
  //      => To unify p and e, c₀ should be matched with c'₁t'₁. This requires
  //         `c'₁t'₁` to be a constant, which is not the case by
  //         construction. Therefore, this case always fails.
  //
  //     Example:     p = 3 + a + b
  //                  e = x + y
  //             Result = fail
  //
  //  4. Non-zero lead-coeff in Pattern + Non-zero lead-coeff in Expression.
  //      Pattern:    c₀  + c₁t₁   + ... + cₙtₙ
  //      Expression: c'₀ + c'₁t'₁ + ... + c'ₘt'ₘ
  //
  //      => Check c₀ = c'₀, Unify(cᵢtᵢ, c'ᵢt'ᵢ) for all i ∈ [1, n-1], unify
  //         the rest.
  //
  //     Example:     p = 3 + a + b
  //                  e = 3 + x + y + z
  //             Result = match with {a ↦ x, b ↦ y + z}
  //
  // Note that Case 1 and Case 4 can be handled together (see
  // VisitAdditionAligned below), while we need a separate routine for the Case
  // 2 (see VisitAdditionSkewed below).
  //
  //
  // Incompleteness
  // --------------
  //
  // Note that our pattern-matching algorithm is incomplete as we perform the
  // left-to-right pattern-matching over the "ordered" summands of the pattern
  // and the expression. It is possible that our algorithm rejects a pair of a
  // pattern and an expression while there exists a substitution unifying the
  // two. Consider the following example:
  //
  //     p = t₁ + t₂
  //     e = t'₁ + t'₂
  //
  // Our algorithm checks if Unify(t₁, t'₁) and Unify(t₂, t'₂) hold. If this
  // condition does not hold, this algorithm rejects the pair (p, e). However,
  // it is possible that Unify(t₁, t'₂) and Unify(t₁, t'₂) hold instead. To be
  // complete, it needs to check all possible re-orderings of the pattern @p p
  // and the expression @p e, which is not tractable in practice. As a result,
  // we decide to make this algorithm incomplete.
  //
  bool VisitAddition(const Pattern& p, const Expression& e,
                     Substitution* const subst) const {
    if (!is_addition(e)) {
      return false;
    }
    const double c0_p{get_constant_in_addition(p)};
    const double c0_e{get_constant_in_addition(e)};
    const auto& map_p = get_expr_to_coeff_map_in_addition(p);
    const auto& map_e = get_expr_to_coeff_map_in_addition(e);
    if (c0_p == 0.0) {
      if (c0_e == 0.0) {
        return VisitAdditionAligned(c0_p, map_p, c0_e, map_e, subst);
      } else {
        return VisitAdditionSkewed(map_p, c0_e, map_e, subst);
      }
    } else {
      if (c0_e == 0.0) {
        return false;  // Case 3.
      } else {
        return VisitAdditionAligned(c0_p, map_p, c0_e, map_e, subst);
      }
    }
  }

  // Helper method for VisitAddition.
  //
  // We perform the left-to-right pattern-matching over p and e:
  //     p := c₀ + (c₁t₁ + ... + cₙtₙ)
  //     e := c'₀ + (c'₁t'₁ + ... + c'ₘt'ₘ)
  //
  // Note that this method takes care of the cases where either (c₀ = c'₀ = 0)
  // or (c₀ ≠ 0 and c'₀ ≠ 0). VisitAddition calls `VisitAdditionSkewed` for
  // other cases.
  //
  // The following conditions must be satisfied for this unification to
  // be successful:
  //
  //   1) c₀ == c'₀.
  //   2) n ≤ m.
  //   3) For all i ∈ [1, n - 1], Unify(cᵢtᵢ, c'ᵢt'ᵢ) holds.
  //   4) Unify(cₙtₙ, c'ₙt'ₙ + ... + c'ₘt'ₘ) holds.
  bool VisitAdditionAligned(const double c0_p,
                            const map<Expression, double>& map_p,
                            const double c0_e,
                            const map<Expression, double>& map_e,
                            Substitution* const subst) const {
    DRAKE_ASSERT((c0_p == 0.0 && c0_e == 0.0) || (c0_p != 0.0 && c0_e != 0.0));
    if (c0_p != c0_e) {
      return false;
    }
    const size_t n{map_p.size()};
    const size_t m{map_e.size()};
    if (!(n <= m)) {
      return false;
    }
    return VisitAdditionCheckPairs(map_p.begin(), map_e.begin(), n, m, subst);
  }

  // Helper method for VisitAddition.
  //
  // We perform the left-to-right pattern-matching over p and e:
  //     p := c₁t₁ + ...          + cₙtₙ
  //     e := c'₀  + c'₁t'₁ + ... + c'ₘt'ₘ
  //
  // Note that p does not have the constant term (no c₀) while e has a non-zero
  // constant term (c'₀ ≠ 0).
  //
  // The following conditions must be satisfied for this unification to be
  // successful:
  //
  //   1) n ≤ m + 1.
  //   2) Unify(c₁t₁, c'₀).
  //   3) For all i ∈ [2, n - 1], Unify(cᵢtᵢ, c'_{i-1}t'_{i-1}) holds.
  //   4) Unify(cₙtₙ, c'_{n-1}t'_{n-1} + ... + c'ₘt'ₘ) holds.
  bool VisitAdditionSkewed(const map<Expression, double>& map_p,
                           const double c0_e,
                           const map<Expression, double>& map_e,
                           Substitution* const subst) const {
    DRAKE_ASSERT(c0_e != 0.0);
    const size_t n{map_p.size()};
    const size_t m{map_e.size()};
    if (!(n <= m + 1)) {
      return false;
    }
    auto it_p = map_p.begin();
    if (!Unify(it_p->first * it_p->second, c0_e, subst)) {
      return false;
    }
    ++it_p;
    auto it_e = map_e.begin();
    return VisitAdditionCheckPairs(it_p, it_e, n - 1, m, subst);
  }

  // Helper method for VisitAdditionAligned and VisitAdditionSkewed.
  //
  // `it_p` and `n` represent a summation `p = c₁t₁ + ... + cₙtₙ` and
  // `it_e` and `m` represent another summation `e = c'₁t'₁ + ... + c'ₘt'ₘ`.
  // Note that we have a precondition that n is less than or equal to m.
  // This helpder method performs the following unifications and updates the
  // output parameter `subst`.
  //
  //  - For the first n - 1 pairs, Unify(cᵢtᵢ, c'ᵢt'ᵢ), for i ∈ [1, n-1].
  //  - Unify the last element of p with the rest of elements in e. That is,
  //    Unify(cₙtₙ, c'ₙt'ₙ + ... c'ₘt'ₘ).
  bool VisitAdditionCheckPairs(map<Expression, double>::const_iterator it_p,
                               map<Expression, double>::const_iterator it_e,
                               const int n, const int m,
                               Substitution* const subst) const {
    DRAKE_ASSERT(n <= m);
    int i = 1;
    for (; i < n; ++i, ++it_p, ++it_e) {
      // Check Unify(cᵢtᵢ, c'ᵢt'ᵢ) holds.
      const double ci_p{it_p->second};
      const double ci_e{it_e->second};
      const Expression& ti_p{it_p->first};
      const Expression& ti_e{it_e->first};
      if (!Unify(ci_p * ti_p, ci_e * ti_e, subst)) {
        return false;
      }
    }
    // Check Unify(cₙtₙ, c'ₙt'ₙ + ... + c'ₘt'ₘ) holds.
    const Expression last_term_p{it_p->first * it_p->second};
    Expression rest_of_e{0.0};
    for (; i <= m; ++i, ++it_e) {
      rest_of_e += it_e->first * it_e->second;
    }
    return Unify(last_term_p, rest_of_e, subst);
  }

  // Visits the pattern @p p and the expression @p e, updates the substitution
  // @p subst, and returns true if @p p and @p e are matched. Otherwise, returns
  // false without modifying the substitution @p subst.
  //
  // This code handles the case where @p p is a multiplication expression. If
  // the expression @p e is not a multiplication, the unification fails
  // immediately.
  //
  //
  // Internal Representation of multiplication expression
  // ----------------------------------------------------
  //
  // A multiplication expression is represented as a pair of its constant
  // factor (: double) and an ordered map from a base (: Expression) to its
  // exponent (: double). For example, `3 * x² * y³` is represented as
  //
  //     constant factor: 3
  //     ordered map: {x ↦ 2, y ↦ 3}.
  //
  // Note that there exists the constant factor "one" even if it is not
  // explicitly provided by a user. For example, `x * y` is represented
  // internally as
  //
  //     constant coefficient: 1
  //     ordered map: {x ↦ 1, y ↦ 1}.
  //
  //
  // Unification (First attempt)
  // ---------------------------
  //
  // Let the pattern @p p and the expression @p e be the products of factors as
  // follows:
  //
  //     p =  c * pow(b₁,t₁)    * ... * pow(bₙ, tₙ)
  //     e =  c' * pow(b'₁, t'₁) * ... * pow(b'ₘ, t'ₘ)
  //
  // Now, we derive the condition for the pattern @p p and the expression @p e
  // to be matched.
  //
  // 1) We require that the expression should be longer than the pattern to be
  //    matched. That is,
  //
  //     n ≤ m.
  //
  // 2) The constant factors of the pattern and the expression should be the
  //    same. That is,
  //
  //     c₀ = c'₀.
  //
  // 3) Each factor of the pattern and the expression should be matched except
  //    for the last one. That is,
  //
  //     Unify(pow(bᵢ, tᵢ), pow(b'ᵢ, tᵢ)) for all i ∈ [1, n-1].
  //
  // 4) Finally, the last factor in the pattern, pow(bₙ, tₙ), and the rest of
  //    unmatched factors in the expression, pow(b'ₙ, t'ₙ) * ... *
  //    pow(b'ₘ, t'ₘ) should be matched. That is,
  //
  //     Unify(pow(bₙ, tₙ), pow(b'ₙ, t'ₙ) * ... * pow(b'ₘ, t'ₘ)).
  //
  //
  // Problem of the first attempt and an extension to the algorithm
  // --------------------------------------------------------------
  //
  // Let's say that a user wants to match a pattern p = a * b and an expression
  // 2 * x. There exists a substitution, {a ↦ 2, b ↦ x}, which unifies the
  // two. However, the above algorithm failes to find this substitution. This is
  // because the pattern, p = a * b, is represented as `p = 1 * a * b`. As a
  // result, the algorithm rejects (p, e) because e is not as long as e.
  //
  // A solution to this problem is to do the case analysis:
  //
  // 1. Lead-coeff in Pattern = 1 && lead-coeff in Expression = 1
  //     Pattern:    pow(b₁,t₁)    * ... * pow(bₙ, tₙ)
  //     Expression: pow(b'₁, t'₁) * ... * pow(b'ₘ, t'ₘ)
  //
  //     => Unify(pow(bᵢ, tᵢ), pow(b'ᵢ, t'ᵢ)) for all i ∈ [1, n-1] and unify
  //        the rest.
  //
  //     Example:     p = a * b
  //                  e = x * y * z
  //             Result = match with {a ↦ x, b ↦ y * z}
  //
  // 2. Lead-coeff in Pattern = 1 && Lead-coeff in Expression ≠ 1.
  //
  //     Pattern:         pow(b₁,t₁)    * ... * pow(bₙ, tₙ)
  //     Expression: c' * pow(b'₁, t'₁) * ... * pow(b'ₘ, t'ₘ)
  //
  //     => Unify(pow(b₁, t₁), c'), Unify(pow(bᵢ, tᵢ), pow(b'_{i-1},
  //        t'_{i-1})) for all i ∈ [1, n-1], and unify the rest.
  //
  //     Example:     p = a * b
  //                  e = 2 * x * y * z
  //             Result = match with {a ↦ 2, b ↦ x * y * z}
  //
  // 3. Lead-coeff in Pattern ≠ 1 && Lead-coeff in Expression = 1.
  //
  //     Pattern:    c * pow(b₁,t₁)    * ... * pow(bₙ, tₙ)
  //     Expression:     pow(b'₁, t'₁) * ... * pow(b'ₘ, t'ₘ)
  //
  //     => To unify p and e, c should be matched with pow(b'₁, t'₁). This
  //        requires `pow(b'₁, t'₁)` to be a constant, which is not the case
  //        by construction. Therefore, this case always fails.
  //
  //     Example:     p = 2 * a * b
  //                  e = x * y * z
  //             Result = fail
  //
  // 4. Lead-coeff in Pattern ≠ 1 && Lead-coeff in Expression ≠ 1.
  //
  //     Pattern:    c  * pow(b₁,t₁)    * ... * pow(bₙ, tₙ)
  //     Expression: c' * pow(b'₁, t'₁) * ... * pow(b'ₘ, t'ₘ)
  //
  //     => Check c = c', Unify(pow(bᵢ, tᵢ), pow(b'ᵢ, t'ᵢ)) for all i ∈ [0,
  //        n-1], and unify the rest.
  //
  //     Example:     p = 3 * a * b
  //                  e = 3 * x * y * z
  //             Result = match with {a ↦ x, b ↦ * y * z}
  //
  // Note that Case 1 and Case 4 can be handled together (in
  // VisitMultiplicationAligned below), while we need a separate routine for
  // the Case 2 (in VisitMultiplicationSkewed below).
  //
  //
  // Incompleteness
  // --------------
  //
  // Note that our pattern-matching algorithm is incomplete as we perform the
  // left-to-right pattern-matching over the ordered factors of the pattern and
  // the expression. It is possible that our algorithm rejects a pair of a
  // pattern and an expression while there exists a substitution unifying the
  // two. Consider the following example:
  //
  //     p = t₁ * t₂
  //     e = t'₁ * t'₂
  //
  // Our algorithm checks if Unify(t₁, t'₁) and Unify(t₂, t'₂) hold. If this
  // condition does not hold, this algorithm rejects the pair (p, e). However,
  // it is possible that Unify(t₁, t'₂) and Unify(t₁, t'₂) hold instead. To be
  // complete, it needs to check all possible re-orderings of the pattern @p p
  // and the expression @p e, which is not tractable in practice. As a result,
  // we decide to make this algorithm incomplete.
  //
  bool VisitMultiplication(const Pattern& p, const Expression& e,
                           Substitution* const subst) const {
    const double c_p{get_constant_in_multiplication(p)};
    if (c_p < 0.0) {
      // Internally, an unary expression `-e` is represented as a multiplicative
      // expression `-1 * e`. The following line allows us to match expressions
      // with a pattern with unary minus.
      return Unify(-p, -e, subst);
    }

    if (!is_multiplication(e)) {
      return false;
    }
    const double c_e{get_constant_in_multiplication(e)};
    const auto& map_p = get_base_to_exponent_map_in_multiplication(p);
    const auto& map_e = get_base_to_exponent_map_in_multiplication(e);
    if (c_p == 1.0) {
      if (c_e == 1.0) {
        return VisitMultiplicationAligned(c_p, map_p, c_e, map_e, subst);
      } else {
        return VisitMultiplicationSkewed(map_p, c_e, map_e, subst);
      }
    } else {
      if (c_e == 1.0) {
        return false;  // Case 3.
      } else {
        return VisitMultiplicationAligned(c_p, map_p, c_e, map_e, subst);
      }
    }
  }

  // Helper method for VisitMultiplication.
  //
  // We perform left-to-right pattern-matching over p and e:
  //
  //     p := c  * (pow(b₁, t₁)   * ... * pow(bₙ, tₙ))
  //     e := c' * (pow(b'₁, t'₁) * ... * pow(b'ₘ, t'ₘ))
  //
  // Note that this method takes care of the cases where either (c = c' = 1) or
  // (c₀ ≠ 1 and c'₀ ≠ 1). VisitMultiplication calls `VisitMultiplicationSkewed`
  // for other cases.
  //
  // The following conditions must be satisfied for this unification to be
  // successful:
  //
  //  1) c == c'
  //  2) n ≤ m
  //  3) For all i ∈ [1, n - 1], Unify(pow(bᵢ, tᵢ), pow(b'ᵢ, t'ᵢ)) holds.
  //  4) Unify(pow(bₙ, tₙ), pow(b'ₙ, t'ₙ) * ... * pow(b'ₘ, t'ₘ)) holds.
  bool VisitMultiplicationAligned(const double c_p,
                                  const map<Expression, Expression>& map_p,
                                  const double c_e,
                                  const map<Expression, Expression>& map_e,
                                  Substitution* const subst) const {
    DRAKE_ASSERT((c_p == 1.0 && c_e == 1.0) || (c_p != 1.0 && c_e != 1.0));
    if (c_p != c_e) {
      return false;
    }
    const size_t n{map_p.size()};
    const size_t m{map_e.size()};
    if (!(n <= m)) {
      return false;
    }
    return VisitMultiplicationCheckPairs(map_p.begin(), map_e.begin(), n, m,
                                         subst);
  }

  // Helper method for VisitMultiplication.
  //
  // We perform left-to-right pattern-matching over p and e:
  //
  //     p := (pow(b₁, t₁) *    ...     * pow(bₙ, tₙ))
  //     e := c' * (pow(b'₁, t'₁) * ... * pow(b'ₘ, t'ₘ))
  //
  // Note that this method handles the case where c' ≠ 1.0.
  //
  // The following conditions must be satisfied for this unification to be
  // successful:
  //
  //  1) n ≤ m + 1.
  //  2) Unify(pow(b₁, t₁), c').
  //  3) For all i ∈ [2, n - 1],
  //     Unify(pow(bᵢ, tᵢ), pow(b'_{i-1}, t'_{i-1})) holds.
  //  4) Unify(pow(bₙ, tₙ), pow(b'ₙ, t'ₙ) * ... * pow(b'ₘ, t'ₘ)) holds.
  bool VisitMultiplicationSkewed(const map<Expression, Expression>& map_p,
                                 const double c_e,
                                 const map<Expression, Expression>& map_e,
                                 Substitution* const subst) const {
    DRAKE_ASSERT(c_e != 1.0);
    const size_t n{map_p.size()};
    const size_t m{map_e.size()};
    if (!(n <= m + 1)) {
      return false;
    }
    auto it_p = map_p.begin();
    if (!Unify(pow(it_p->first, it_p->second), c_e, subst)) {
      return false;
    }
    ++it_p;
    auto it_e = map_e.begin();
    return VisitMultiplicationCheckPairs(it_p, it_e, n - 1, m, subst);
  }

  // Helper method for VisitMultiplicationAligned and VisitMultiplicationSkewed.
  //
  // `it_p` and `n` represent a product `p = pow(b₁, t₁) * ... * pow(bₙ, tₙ)`
  // and `it_e` and `m` represent another product `e = pow(b'₁, t'₁) * pow(b'ₘ,
  // t'ₘ)`. Note that we have a precondition that n is less than or equal to m.
  // This helper method performs the following unifications and updates the
  // output parameter `subst`.
  //
  //  - For the first n - 1 pairs, Unify(pow(bᵢ, tᵢ), pow(b'ᵢ, t'ᵢ)),
  //    for i ∈ [1, n-1].
  //  - Unify the last element of p with the rest of elements in e. That is,
  //    Unify(pow(bₙ, tₙ), pow(b'ₙ, t'ₙ) * ... * pow(b'ₘ, t'ₘ)).
  bool VisitMultiplicationCheckPairs(
      map<Expression, Expression>::const_iterator it_p,
      map<Expression, Expression>::const_iterator it_e, const int n,
      const int m, Substitution* const subst) const {
    DRAKE_ASSERT(n >= 1);
    // Checks Unify(pow(bᵢ, tᵢ), pow(b'ᵢ, t'ᵢ)) holds.
    int i = 1;
    for (; i < n; ++i, ++it_p, ++it_e) {
      const Expression& bi_p{it_p->first};
      const Expression& bi_e{it_e->first};
      const Expression& ti_p{it_p->second};
      const Expression& ti_e{it_e->second};
      if (!Unify(pow(bi_p, ti_p), pow(bi_e, ti_e), subst)) {
        return false;
      }
    }
    // Checks Unify(pow(bₙ, tₙ), pow(b'ₙ, t'ₙ) * ... * pow(b'ₘ, t'ₘ)).
    const Expression last_term_p{pow(it_p->first, it_p->second)};
    Expression rest_of_e{1.0};
    for (; i <= m; ++i, ++it_e) {
      rest_of_e *= pow(it_e->first, it_e->second);
    }
    return Unify(last_term_p, rest_of_e, subst);
  }

  // Helper method to handle unary cases.
  bool VisitUnary(const function<bool(const Expression&)>& pred,
                  const Pattern& p, const Expression& e,
                  Substitution* const subst) const {
    return pred(e) && Unify(get_argument(p), get_argument(e), subst);
  }

  // Helper method to handle binary cases.
  bool VisitBinary(const function<bool(const Expression&)>& pred,
                   const Pattern& p, const Expression& e,
                   Substitution* const subst) const {
    return pred(e) &&
           Unify(get_first_argument(p), get_first_argument(e), subst) &&
           Unify(get_second_argument(p), get_second_argument(e), subst);
  }

  bool VisitPow(const Pattern& p, const Expression& e,
                Substitution* const subst) const {
    // Case p = pow(e1, e2). e should be a pow expression and its arguments
    // should be matched with e1 and e2.
    return VisitBinary(&is_pow, p, e, subst);
  }

  bool VisitDivision(const Pattern& p, const Expression& e,
                     Substitution* const subst) const {
    // Case p = e1 / e2. e should be a division and its arguments should be
    // matched with e1 and e2.
    return VisitBinary(&is_division, p, e, subst);
  }

  bool VisitAbs(const Pattern& p, const Expression& e,
                Substitution* const subst) const {
    // Case p = abs(e'). e should be an abs expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_abs, p, e, subst);
  }

  bool VisitLog(const Pattern& p, const Expression& e,
                Substitution* const subst) const {
    // Case p = log(e'). e should be a log expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_log, p, e, subst);
  }

  bool VisitExp(const Pattern& p, const Expression& e,
                Substitution* const subst) const {
    // Case p = exp(e'). e should be an exp expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_exp, p, e, subst);
  }

  bool VisitSqrt(const Pattern& p, const Expression& e,
                 Substitution* const subst) const {
    // Case p = sqrt(e'). e should be a square-root and its argument should be
    // matched with e'.
    return VisitUnary(&is_sqrt, p, e, subst);
  }

  bool VisitSin(const Pattern& p, const Expression& e,
                Substitution* const subst) const {
    // Case p = sin(e'). e should be a sin expression and its argument should be
    // matched with e'.
    return VisitUnary(&is_sin, p, e, subst);
  }

  bool VisitCos(const Pattern& p, const Expression& e,
                Substitution* const subst) const {
    // Case p = cos(e'). e should be a cos expression and its argument should be
    // matched with e'.
    return VisitUnary(&is_cos, p, e, subst);
  }

  bool VisitTan(const Pattern& p, const Expression& e,
                Substitution* const subst) const {
    // Case p = tan(e'). e should be a tan expression and its argument should be
    // matched with e'.
    return VisitUnary(&is_tan, p, e, subst);
  }

  bool VisitAsin(const Pattern& p, const Expression& e,
                 Substitution* const subst) const {
    // Case p = asin(e'). e should be an asin expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_asin, p, e, subst);
  }

  bool VisitAcos(const Pattern& p, const Expression& e,
                 Substitution* const subst) const {
    // Case p = acos(e'). e should be an acos expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_acos, p, e, subst);
  }

  bool VisitAtan(const Pattern& p, const Expression& e,
                 Substitution* const subst) const {
    // Case p = atan(e'). e should be an atan expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_atan, p, e, subst);
  }

  bool VisitAtan2(const Pattern& p, const Expression& e,
                  Substitution* const subst) const {
    // Case p = atan2(e1, e2). e should be an atan2 expression and its arguments
    // should be matched with e1, e2.
    return VisitBinary(&is_atan2, p, e, subst);
  }

  bool VisitSinh(const Pattern& p, const Expression& e,
                 Substitution* const subst) const {
    // Case p = sinh(e'). e should be a sinh expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_sinh, p, e, subst);
  }

  bool VisitCosh(const Pattern& p, const Expression& e,
                 Substitution* const subst) const {
    // Case p = cosh(e'). e should be a cosh expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_cosh, p, e, subst);
  }

  bool VisitTanh(const Pattern& p, const Expression& e,
                 Substitution* const subst) const {
    // Case p = tanh(e'). e should be a tanh expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_tanh, p, e, subst);
  }

  bool VisitMin(const Pattern& p, const Expression& e,
                Substitution* const subst) const {
    // Case p = min(e1, e2). e should be a min expression and its arguments
    // should be matched with e1 and e2.
    return VisitBinary(&is_min, p, e, subst);
  }

  bool VisitMax(const Pattern& p, const Expression& e,
                Substitution* const subst) const {
    // Case p = max(e1, e2). e should be a max expression and its arguments
    // should be matched with e1 and e2.
    return VisitBinary(&is_max, p, e, subst);
  }

  bool VisitCeil(const Pattern& p, const Expression& e,
                 Substitution* const subst) const {
    // Case p = ceil(e'). e should be a ceil expression and its argument should
    // be matched with e'.
    return VisitUnary(&is_ceil, p, e, subst);
  }

  bool VisitFloor(const Pattern& p, const Expression& e,
                  Substitution* const subst) const {
    // Case p = floor(e'). e should be a floor expression and its argument
    // should be matched with e'.
    return VisitUnary(&is_floor, p, e, subst);
  }

  bool VisitIfThenElse(const Pattern&, const Expression&,
                       Substitution* const) const {
    // TODO(soonho): Support this.
    throw runtime_error(
        "Unification algorithm does not support if-then-else-expressions, yet");
  }

  bool VisitUninterpretedFunction(const Pattern&, const Expression&,
                                  Substitution* const) const {
    // TODO(soonho): Support this.
    throw runtime_error(
        "Unification algorithm does not support uninterpreted functions, yet");
  }

  // Makes VisitExpression a friend of this class so that it can use private
  // methods.
  friend bool VisitExpression<bool>(const UnificationVisitor*, const Pattern&,
                                    const Expression&, Substitution* const&);
};

// Unifies the expression @p e with the pattern @p p.
optional<Substitution> Unify(const Pattern& p, const Expression& e) {
  return UnificationVisitor{}.Unify(p, e);
}
}  // namespace

Rewriter MakeRuleRewriter(const RewritingRule& rule) {
  return [rule](const Expression& e) {
    const optional<Substitution> subst{Unify(rule.lhs(), e)};
    if (subst) {
      return rule.rhs().Substitute(*subst);
    } else {
      return e;
    }
  };
}

}  // namespace symbolic
}  // namespace drake
