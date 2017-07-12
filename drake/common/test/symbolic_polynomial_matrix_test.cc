#include "drake/common/symbolic_polynomial.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/monomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_variable.h"
#include "drake/common/test/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;

class SymbolicPolynomialMatrixTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};

  const Monomial m_x_{var_x_};
  const Monomial m_y_{var_y_};
  const Monomial m_z_{var_z_};

  MatrixX<Polynomial> M_poly_{2, 2};
  MatrixX<Monomial> M_monomial_{2, 2};
  MatrixX<double> M_double_{2, 2};

  VectorX<Polynomial> v_poly_{2};
  VectorX<Monomial> v_monomial_{2};
  VectorX<double> v_double_{2};

  void SetUp() override {
    // clang-format off
    M_poly_ << Polynomial{},    (m_x_ + m_y_),      // [0        x + y]
              (2 + m_x_ * m_y_), Polynomial{m_z_};  // [2 + xy   z    ]

    M_monomial_ << Monomial{},   m_x_,                  // [1    x  ]
                  (m_x_ * m_y_), (m_x_ * m_x_ * m_z_);  // [xy   x²z]

    M_double_ << -2.0,  4.0,  // [-2.0   4.0]
                  5.0, -9.0;  // [ 5.0  -9.0]

    v_poly_ << (m_x_ + m_y_),             // [x+y    ]
               (3 + m_x_ * m_y_ * m_z_);  // [3 + xyz]

    v_monomial_ << (m_x_ * m_y_),         // [xy ]
                   (m_x_ * m_y_ * m_z_);  // [xyz]

    v_double_ << -1.1,  // [-1.1]
                  5.2;  // [ 5.2]
    // clang-format on
  }
};

// Compares m1 and m2 after expanding both of them.
::testing::AssertionResult CompareMatricesWithExpansion(
    const Eigen::Ref<const MatrixX<Expression>>& m1,
    const Eigen::Ref<const MatrixX<Expression>>& m2) {
  const MatrixX<Expression> m1_expanded{
      m1.unaryExpr([](const Expression& e) { return e.Expand(); })};
  const MatrixX<Expression> m2_expanded{
      m2.unaryExpr([](const Expression& e) { return e.Expand(); })};
  if (m1_expanded == m2_expanded) {
    return ::testing::AssertionSuccess()
           << "m1 and m2 are equal after expansion where m1 = \n"
           << m1 << "\n"
           << "and m2 = " << m2;
  } else {
    return ::testing::AssertionFailure()
           << "m1 and m2 are not equal after expansion where m1 = \n"
           << m1 << "\n"
           << "m2 = " << m2 << "\n"
           << "m1_expanded = \n"
           << m1_expanded << "\n"
           << "m2_expanded = \n"
           << m2_expanded;
  }
}

TEST_F(SymbolicPolynomialMatrixTest, PolynomialPolynomial) {
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_poly_ + M_poly_).cast<Expression>(),
      M_poly_.cast<Expression>() + M_poly_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_poly_ - M_poly_).cast<Expression>(),
      M_poly_.cast<Expression>() - M_poly_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_poly_ * M_poly_).cast<Expression>(),
      M_poly_.cast<Expression>() * M_poly_.cast<Expression>()));
  EXPECT_PRED2(
      ExprEqual, v_poly_.dot(v_poly_).ToExpression(),
      v_poly_.cast<Expression>().dot(v_poly_.cast<Expression>()).Expand());
}

TEST_F(SymbolicPolynomialMatrixTest, PolynomialMonomial) {
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_poly_ + M_monomial_).cast<Expression>(),
      M_poly_.cast<Expression>() + M_monomial_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_poly_ - M_monomial_).cast<Expression>(),
      M_poly_.cast<Expression>() - M_monomial_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_poly_ * M_monomial_).cast<Expression>(),
      M_poly_.cast<Expression>() * M_monomial_.cast<Expression>()));
  EXPECT_PRED2(
      ExprEqual, v_poly_.dot(v_monomial_).ToExpression(),
      v_poly_.cast<Expression>().dot(v_monomial_.cast<Expression>()).Expand());
}

TEST_F(SymbolicPolynomialMatrixTest, PolynomialDouble) {
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_poly_ + M_double_).cast<Expression>(),
      M_poly_.cast<Expression>() + M_double_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_poly_ - M_double_).cast<Expression>(),
      M_poly_.cast<Expression>() - M_double_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_poly_ * M_double_).cast<Expression>(),
      M_poly_.cast<Expression>() * M_double_.cast<Expression>()));
  EXPECT_PRED2(
      ExprEqual, v_poly_.dot(v_double_).ToExpression(),
      v_poly_.cast<Expression>().dot(v_double_.cast<Expression>()).Expand());
}

TEST_F(SymbolicPolynomialMatrixTest, MonomialPolynomial) {
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_monomial_ + M_poly_).cast<Expression>(),
      M_monomial_.cast<Expression>() + M_poly_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_monomial_ - M_poly_).cast<Expression>(),
      M_monomial_.cast<Expression>() - M_poly_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_monomial_ * M_poly_).cast<Expression>(),
      M_monomial_.cast<Expression>() * M_poly_.cast<Expression>()));
  EXPECT_PRED2(
      ExprEqual, v_monomial_.dot(v_poly_).ToExpression(),
      v_monomial_.cast<Expression>().dot(v_poly_.cast<Expression>()).Expand());
}

TEST_F(SymbolicPolynomialMatrixTest, MonomialMonomial) {
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_monomial_ + M_double_).cast<Expression>(),
      M_monomial_.cast<Expression>() + M_double_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_monomial_ - M_double_).cast<Expression>(),
      M_monomial_.cast<Expression>() - M_double_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_monomial_ * M_double_).cast<Expression>(),
      M_monomial_.cast<Expression>() * M_double_.cast<Expression>()));
  EXPECT_PRED2(ExprEqual, v_monomial_.dot(v_monomial_).ToExpression(),
               v_monomial_.cast<Expression>()
                   .dot(v_monomial_.cast<Expression>())
                   .Expand());
}

TEST_F(SymbolicPolynomialMatrixTest, MonomialDouble) {
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_monomial_ + M_double_).cast<Expression>(),
      M_monomial_.cast<Expression>() + M_double_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_monomial_ - M_double_).cast<Expression>(),
      M_monomial_.cast<Expression>() - M_double_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_monomial_ * M_double_).cast<Expression>(),
      M_monomial_.cast<Expression>() * M_double_.cast<Expression>()));
  EXPECT_PRED2(ExprEqual, v_monomial_.dot(v_double_).ToExpression(),
               v_monomial_.cast<Expression>()
                   .dot(v_double_.cast<Expression>())
                   .Expand());
}

TEST_F(SymbolicPolynomialMatrixTest, doublePolynomial) {
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_double_ + M_poly_).cast<Expression>(),
      M_double_.cast<Expression>() + M_poly_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_double_ - M_poly_).cast<Expression>(),
      M_double_.cast<Expression>() - M_poly_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_double_ * M_poly_).cast<Expression>(),
      M_double_.cast<Expression>() * M_poly_.cast<Expression>()));
  EXPECT_PRED2(
      ExprEqual, v_double_.dot(v_poly_).ToExpression(),
      v_double_.cast<Expression>().dot(v_poly_.cast<Expression>()).Expand());
}

TEST_F(SymbolicPolynomialMatrixTest, doubleMonomial) {
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_double_ + M_monomial_).cast<Expression>(),
      M_double_.cast<Expression>() + M_monomial_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_double_ - M_monomial_).cast<Expression>(),
      M_double_.cast<Expression>() - M_monomial_.cast<Expression>()));
  EXPECT_TRUE(CompareMatricesWithExpansion(
      (M_double_ * M_monomial_).cast<Expression>(),
      M_double_.cast<Expression>() * M_monomial_.cast<Expression>()));
  EXPECT_PRED2(ExprEqual, v_double_.dot(v_monomial_).ToExpression(),
               v_double_.cast<Expression>()
                   .dot(v_monomial_.cast<Expression>())
                   .Expand());
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
