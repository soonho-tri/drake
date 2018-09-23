#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
// TODO(soonho-tri): Change to #error, when #6613 merged.
#warning Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <ostream>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
/**
 * Represents symbolic polynomial fractions. A function f(x) is a polynomial
 * fraction, if f(x) = p(x) / q(x), where both p(x) and q(x) are polynomials of
 * x. Note that polynomial fraction is closed under (+, -, x, /). One
 * application of polynomial fraction is in polynomial optimization, where we
 * represent (or approximate) functions using polynomial fractions, and then
 * convert the constraint f(x) = h(x) (where h(x) is a polynomial) to a
 * polynomial constraint p(x) - q(x) * h(x) = 0, or convert the inequality
 * constraint f(x) >= h(x) as p(x) - q(x) * h(x) >= 0 if we know q(x) > 0.
 *
 * We choose to create this new class instead of using symbolic::Expression.
 * If we were to use symbolic::Expression to represent polynomial fraction, then
 * it is quite hard to extract the numerator/denominator for a complicated
 * expression; for instance, from p1(x) / q1(x) + p2(x) / q2(x) + ... + pn(x) /
 * qn(x).
 */
class PolynomialFraction {
 public:
  /** Constructs a zero polynomial fraction 0 / 1. */
  PolynomialFraction();

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PolynomialFraction)

  /**
   * Constructs a polynomial fraction numerator / denominator.
   * @param numerator The numerator of the fraction.
   * @param denominator The denominator of the fraction.
   * @pre denominator cannot be structurally equal to 0.
   * @pre None of the indeterminates in the numerator can be decision variables
   * in the denominator; similarly none of the indeterminates in the denominator
   * can be decision variables in the numerator.
   * @throw logic_error if the precondition is not satisfied.
   */
  PolynomialFraction(const Polynomial& numerator,
                     const Polynomial& denominator);

  /**
   * Constructs a polynomial fraction p / 1. Note that we use 1 as the
   * denominator.
   * @param p The numerator of the fraction.
   */
  PolynomialFraction(const Polynomial& p);

  /**
   * Constructs a polynomial fraction c / 1. Note that we use 1 as the
   * denominator.
   * @param c The numerator of the fraction.
   */
  PolynomialFraction(double c);

  ~PolynomialFraction() = default;

  /// Getter for the numerator.
  const Polynomial& numerator() const { return numerator_; }

  /// Getter for the denominator.
  const Polynomial& denominator() const { return denominator_; }

  PolynomialFraction& operator+=(const PolynomialFraction& f);
  PolynomialFraction& operator+=(const Polynomial& p);
  PolynomialFraction& operator+=(double c);

  PolynomialFraction& operator-=(const PolynomialFraction& f);
  PolynomialFraction& operator-=(const Polynomial& p);
  PolynomialFraction& operator-=(double c);

  PolynomialFraction& operator*=(const PolynomialFraction& f);
  PolynomialFraction& operator*=(const Polynomial& p);
  PolynomialFraction& operator*=(double c);

  PolynomialFraction& operator/=(const PolynomialFraction& f);
  PolynomialFraction& operator/=(const Polynomial& p);
  PolynomialFraction& operator/=(double c);

  /**
   * Returns true if this polynomial fraction and f are structurally equal.
   */
  bool EqualTo(const PolynomialFraction& f) const;

  friend std::ostream& operator<<(std::ostream&, const PolynomialFraction& f);

 private:
  // Throws std::logic_error if an indeterminate of the denominator (numerator,
  // respectively) is a decision variable of the numerator (denominator).
  void CheckInvariant() const;
  Polynomial numerator_;
  Polynomial denominator_;
};

/**
 * Unary minus operation for polynomial fraction.
 * if f(x) = p(x) / q(x), then -f(x) = (-p(x)) / q(x)
 */
PolynomialFraction operator-(PolynomialFraction f);

PolynomialFraction operator+(PolynomialFraction f1,
                             const PolynomialFraction& f2);
PolynomialFraction operator+(PolynomialFraction f, const Polynomial& p);
PolynomialFraction operator+(const Polynomial& p, PolynomialFraction f);
PolynomialFraction operator+(PolynomialFraction f, double c);
PolynomialFraction operator+(double c, PolynomialFraction f);

PolynomialFraction operator-(PolynomialFraction f1,
                             const PolynomialFraction& f2);
PolynomialFraction operator-(PolynomialFraction f, const Polynomial& p);
PolynomialFraction operator-(const Polynomial& p, PolynomialFraction f);
PolynomialFraction operator-(PolynomialFraction f, double c);
PolynomialFraction operator-(double c, PolynomialFraction f);

PolynomialFraction operator*(PolynomialFraction f1,
                             const PolynomialFraction& f2);
PolynomialFraction operator*(PolynomialFraction f, const Polynomial& p);
PolynomialFraction operator*(const Polynomial& p, PolynomialFraction f);
PolynomialFraction operator*(PolynomialFraction f, double c);
PolynomialFraction operator*(double c, PolynomialFraction f);

PolynomialFraction operator/(PolynomialFraction f1,
                             const PolynomialFraction& f2);
PolynomialFraction operator/(PolynomialFraction f, const Polynomial& p);
PolynomialFraction operator/(const Polynomial& p, const PolynomialFraction& f);
PolynomialFraction operator/(PolynomialFraction f, double c);
PolynomialFraction operator/(double c, const PolynomialFraction& f);

/**
 * Returns the polynomial fraction @p f raised to @p n.
 * For a positive integer n, (f/g)ⁿ = fⁿ / gⁿ, (f/g)⁻ⁿ = gⁿ / fⁿ, (f/g)⁰ = 0 /
 * 1.
 */
PolynomialFraction pow(const PolynomialFraction& f, int n);

/**
 * Provides the following operations
 *  - Matrix<PolynomialFraction> * Matrix<Polynomial> =>
 * Matrix<PolynomialFraction>
 *  - Matrix<PolynomialFraction> * Matrix<double> => Matrix<PolynomialFraction>
 *  - Matrix<Polynomial> * Matrix<PolynomialFraction> =>
 * Matrix<PolynomialFraction>
 *  - Matrix<double> * Matrix<PolynomialFraction> => Matrix<PolynomialFraction>
 *
 * @note that these operator overloadings are necessary even after providing
 * Eigen::ScalarBinaryOpTraits. See
 * https://stackoverflow.com/questions/41494288/mixing-scalar-types-in-eigen
 * for more information
 */
#if defined(DRAKE_DOXYGEN_CXX)
template <typename MatrixL, typename MatrixR>
Eigen::Matrix<PolynomialFraction, MatrixL::RowsAtCompileTime,
              MatrixR::ColsAtCompileTime>
operator*(const Eigen::MatrixBase<MatrixL>& lhs,
          const Eigen::MatrixBase<MatrixR>& rhs);
#else
template <typename MatrixL, typename MatrixR>
typename std::enable_if<
    std::is_base_of<Eigen::MatrixBase<MatrixL>, MatrixL>::value &&
        std::is_base_of<Eigen::MatrixBase<MatrixR>, MatrixR>::value &&
        ((std::is_same<typename MatrixL::Scalar, PolynomialFraction>::value &&
          (std::is_same<typename MatrixR::Scalar, Polynomial>::value ||
           std::is_same<typename MatrixR::Scalar, double>::value)) ||
         (std::is_same<typename MatrixR::Scalar, PolynomialFraction>::value &&
          (std::is_same<typename MatrixL::Scalar, Polynomial>::value ||
           std::is_same<typename MatrixL::Scalar, double>::value))),
    Eigen::Matrix<PolynomialFraction, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>::type
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  return lhs.template cast<PolynomialFraction>() *
         rhs.template cast<PolynomialFraction>();
}
#endif
}  // namespace symbolic
}  // namespace drake

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {

// Defines Eigen traits needed for Matrix<drake::symbolic::PolynomialFraction>.
template <>
struct NumTraits<drake::symbolic::PolynomialFraction>
    : GenericNumTraits<drake::symbolic::PolynomialFraction> {
  static inline int digits10() { return 0; }
};

// Informs Eigen that BinaryOp(LhsType, RhsType) gets ResultType.
#define DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(LhsType, RhsType, BinaryOp,    \
                                               ResultType)                    \
  template <>                                                                 \
  struct ScalarBinaryOpTraits<LhsType, RhsType, BinaryOp<LhsType, RhsType>> { \
    enum { Defined = 1 };                                                     \
    typedef ResultType ReturnType;                                            \
  };

// Informs Eigen that LhsType op RhsType gets ResultType
// where op ∈ {+, -, *, /, conj_product}.
#define DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(  \
    LhsType, RhsType, ResultType)                                             \
  DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(LhsType, RhsType,                    \
                                         internal::scalar_sum_op, ResultType) \
  DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(                                     \
      LhsType, RhsType, internal::scalar_difference_op, ResultType)           \
  DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(                                     \
      LhsType, RhsType, internal::scalar_product_op, ResultType)              \
  DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS(                                     \
      LhsType, RhsType, internal::scalar_conj_product_op, ResultType)

// Informs Eigen that PolynomialFraction op Polynomial gets PolynomialFraction
// where op ∈ {+, -, *, /, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::PolynomialFraction, drake::symbolic::Polynomial,
    drake::symbolic::PolynomialFraction)

// Informs Eigen that Polynomial op PolynomialFraction gets PolynomialFraction
// where op ∈ {+, -, *, /, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::Polynomial, drake::symbolic::PolynomialFraction,
    drake::symbolic::PolynomialFraction)

// Informs Eigen that double op PolynomialFraction gets PolynomialFraction
// where op ∈ {+, -, *, /, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    double, drake::symbolic::PolynomialFraction,
    drake::symbolic::PolynomialFraction)

// Informs Eigen that PolynomialFraction op double gets PolynomialFraction
// where op ∈ {+, -, *, /, conj_product}.
DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS(
    drake::symbolic::PolynomialFraction, double,
    drake::symbolic::PolynomialFraction)
#undef DRAKE_SYMBOLIC_SCALAR_BINARY_OP_TRAITS
#undef DRAKE_SYMBOLIC_SCALAR_SUM_DIFF_PRODUCT_CONJ_PRODUCT_TRAITS
}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
