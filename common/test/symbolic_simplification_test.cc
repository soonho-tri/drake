#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;

// Provides common variables that are used by the following tests.
class SymbolicSimplificationTest : public ::testing::Test {
 protected:
  const Variable x_{"x", Variable::Type::CONTINUOUS};
  const Variable y_{"y", Variable::Type::CONTINUOUS};
  const Variable z_{"z", Variable::Type::CONTINUOUS};
};

TEST_F(SymbolicSimplificationTest, SinSquare) {
  const Expression e{sin(x_) * sin(x_)};
  const Expression result = SinSquare(e);
  EXPECT_PRED2(ExprEqual, result, 1 - cos(x_) * cos(x_));
}

TEST_F(SymbolicSimplificationTest, CosSquare) {
  const Expression e{cos(x_) * cos(x_)};
  const Expression result = CosSquare(e);
  EXPECT_PRED2(ExprEqual, result, 1 - sin(x_) * sin(x_));
}

TEST_F(SymbolicSimplificationTest, MakeTry) {
  const Pattern p{MakeTry(SinSquare, CosSquare)};
  const Expression e1{sin(x_) * sin(x_)};
  EXPECT_PRED2(ExprEqual, p(e1), 1 - cos(x_) * cos(x_));

  const Expression e2{cos(x_) * cos(x_)};
  EXPECT_PRED2(ExprEqual, p(e2), 1 - sin(x_) * sin(x_));
}

TEST_F(SymbolicSimplificationTest, MakeCongruence) {
  const Pattern p{MakeCongruence(CosSquare)};

  const Expression e1{cos(x_) * cos(x_) + sin(x_) * sin(x_)};
  EXPECT_PRED2(ExprEqual, p(e1), 1);

  const Expression e2{sin(x_) * cos(x_) * cos(x_) +
                      sin(x_) * sin(x_) * sin(x_)};
  EXPECT_PRED2(ExprEqual, p(e2).Expand(), sin(x_));
}

TEST_F(SymbolicSimplificationTest, RussExample) {
  const Variable b{"b"};
  const Variable l{"l"};
  const Variable m{"m"};
  const Variable mh{"mh"};
  const Variable qh{"qh"};
  const Variable qst{"qst"};

  // Original
  const Expression e =
      (0.5 * (mh * (2 * (pow(l, 2) * pow(sin(qst), 2)) +
                    2 * (pow(l, 2) * pow(cos(qst), 2)))) +
       0.5 * (m * (2 * (pow((l - b), 2) * pow(sin(qst), 2)) +
                   2 * (pow((l - b), 2) * pow(cos(qst), 2)))) +
       0.5 * (m * (2 * pow((-(l * sin(qst)) - (b * sin((qst + qh)))), 2) +
                   2 * pow(((l * cos(qst)) + (b * cos((qst + qh)))), 2))));

  // Output from sympy.trigsimp
  const Expression expected = (1.0 * l * l * mh + 1.0 * m * (b - l) * (b - l) +
                               1.0 * m * (b * b + 2 * b * l * cos(qh) + l * l))
                                  .Expand();

  const Pattern expander = [](const Expression& e) { return e.Expand(); };
  const Pattern simplifier = MakeFixpoint(MakeCongruence(
      MakeTry(SinSquare, SinSumOfAngle, CosSumOfAngle, expander)));

  EXPECT_PRED2(ExprEqual, simplifier(e), expected);
  std::cerr << simplifier(e) << std::endl;
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
