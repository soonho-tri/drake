#include "drake/common/symbolic_polynomial.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using std::vector;

using test::ExprEqual;

class SymbolicPolynomialTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Variables var_xy_{var_x_, var_y_};
  const Variables var_xyz_{var_x_, var_y_, var_z_};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  const vector<Expression> exprs_{
      0,
      -1,
      3.14,
      x_,
      5 * x_,
      -3 * x_,
      y_,
      x_* y_,
      2 * x_* x_,
      2 * x_* x_,
      6 * x_* y_,
      3 * x_* x_* y_ + 4 * pow(y_, 3) * z_ + 2,
      y_*(3 * x_ * x_ + 4 * y_ * y_ * z_) + 2,
      6 * pow(x_, 3) * pow(y_, 2),
      2 * pow(x_, 3) * 3 * pow(y_, 2),
      pow(x_, 3) - 4 * x_* y_* y_ + 2 * x_* x_* y_ - 8 * pow(y_, 3),
      pow(x_ + 2 * y_, 2) * (x_ - 2 * y_),
      (x_ + 2 * y_) * (x_ * x_ - 4 * y_ * y_),
      (x_ * x_ + 4 * x_ * y_ + 4 * y_ * y_) * (x_ - 2 * y_),
      pow(x_ + y_ + 1, 4),
      pow(x_ + y_ + 1, 3),
      1 + x_* x_ + 2 * (y_ - 0.5 * x_ * x_ - 0.5)};
};

TEST_F(SymbolicPolynomialTest, DefaultConstructor) {
  const Polynomial p{};
  EXPECT_TRUE(p.indeterminates().empty());
  EXPECT_TRUE(p.monomial_to_coefficient_map().empty());
  EXPECT_PRED2(ExprEqual, p.ToExpression(), Expression{0.0});
}

TEST_F(SymbolicPolynomialTest, ConstructFromExpression) {
  // Expression -------------------> Polynomial
  //     |                               |
  //     | .Expand()                     | .ToExpression()
  //    \/                              \/
  // Expanded Expression     ==      Expression

  for (const Expression& e : exprs_) {
    const Polynomial p{e};
    const Expression expanded_expr{e.Expand()};
    const Expression expr_from_poly{p.ToExpression()};
    EXPECT_PRED2(ExprEqual, expanded_expr, expr_from_poly);
  }
}

TEST_F(SymbolicPolynomialTest, Addition) {
  //   (Polynomial(e₁) + Polynomial(e₂)).ToExpression()
  // = e₁.Expand() + e₂.Expand()
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      EXPECT_PRED2(ExprEqual, (Polynomial{e1} + Polynomial{e2}).ToExpression(),
                   e1.Expand() + e2.Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, Subtraction) {
  //   (Polynomial(e₁) - Polynomial(e₂)).ToExpression()
  // = e₁.Expand() - e₂.Expand()
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      EXPECT_PRED2(ExprEqual, (Polynomial{e1} - Polynomial{e2}).ToExpression(),
                   e1.Expand() - e2.Expand());
    }
  }
}

TEST_F(SymbolicPolynomialTest, Multiplication) {
  //   (Polynomial(e₁) * Polynomial(e₂)).ToExpression()
  // = (e₁.Expand() * e₂.Expand()).Expand()
  for (const Expression& e1 : exprs_) {
    for (const Expression& e2 : exprs_) {
      EXPECT_PRED2(ExprEqual, (Polynomial{e1} * Polynomial{e2}).ToExpression(),
                   (e1.Expand() * e2.Expand()).Expand());
    }
  }
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
