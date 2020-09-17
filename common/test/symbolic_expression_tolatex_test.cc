#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using std::runtime_error;
using std::string;

// Provides common variables that are used by the following tests.
class SymbolicExpressionToLatexTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};

  ::testing::AssertionResult Check(const Expression& e,
                                   const string& expected_str) {
    if (e.ToLatex() == expected_str) {
      return ::testing::AssertionSuccess();
    }
    return ::testing::AssertionFailure() << "\n"
                                         << "ToLatex  = " << e.ToLatex()
                                         << "\n"
                                            "Expected = "
                                         << expected_str << "\n";
  }
};

TEST_F(SymbolicExpressionToLatexTest, Variable) { EXPECT_TRUE(Check(x_, "x")); }

TEST_F(SymbolicExpressionToLatexTest, Constant) {
  EXPECT_TRUE(Check(Expression(3.14), "3.14"));
  EXPECT_TRUE(Check(Expression(-3.14), "-3.14"));
}

TEST_F(SymbolicExpressionToLatexTest, NaN) {
  EXPECT_TRUE(Check(Expression::NaN(), R"""(\mathrm{NaN})"""));
}

// TEST_F(SymbolicExpressionToLatexTest, Add) {
//   EXPECT_TRUE(Check(3 + x_ + 2 * y_ - 4 * z_, R"""(3 + x + 2 * y + -4 *
//   z)"""));
// }

// TEST_F(SymbolicExpressionToLatexTest, Mul) {
//   EXPECT_TRUE(Check(3 + x_ + 2 * y_ - 4 * z_, R"""(3 + x + 2 * y + -4 *
//   z)"""));
// }

TEST_F(SymbolicExpressionToLatexTest, Div) {
  EXPECT_TRUE(Check(x_ / y_, R"""(\frac{x}{y})"""));
}

TEST_F(SymbolicExpressionToLatexTest, Log) {
  EXPECT_TRUE(Check(log(x_), R"""(\ln(x))"""));
}

TEST_F(SymbolicExpressionToLatexTest, Abs) {
  EXPECT_TRUE(Check(abs(x_), R"""(\lvert{x}\rvert)"""));
}

TEST_F(SymbolicExpressionToLatexTest, Exp) {
  EXPECT_TRUE(Check(exp(x_), R"""(\exp(x))"""));
}

TEST_F(SymbolicExpressionToLatexTest, Sqrt) {
  EXPECT_TRUE(Check(sqrt(x_), R"""(\sqrt{x})"""));
}

TEST_F(SymbolicExpressionToLatexTest, Pow) {
  EXPECT_TRUE(Check(pow(x_, y_), R"""({x}^{y})"""));
}

TEST_F(SymbolicExpressionToLatexTest, Trig) {
  EXPECT_TRUE(Check(sin(x_), R"""(\sin(x))"""));
  EXPECT_TRUE(Check(cos(x_), R"""(\cos(x))"""));
  EXPECT_TRUE(Check(tan(x_), R"""(\tan(x))"""));

  EXPECT_TRUE(Check(asin(x_), R"""(\arcsin(x))"""));
  EXPECT_TRUE(Check(acos(x_), R"""(\arccos(x))"""));
  EXPECT_TRUE(Check(atan(x_), R"""(\arctan(x))"""));
  EXPECT_TRUE(Check(atan2(x_, y_), R"""(\mathrm{arctan2}(x, y))"""));

  EXPECT_TRUE(Check(sinh(x_), R"""(\sinh(x))"""));
  EXPECT_TRUE(Check(cosh(x_), R"""(\cosh(x))"""));
  EXPECT_TRUE(Check(tanh(x_), R"""(\tanh(x))"""));
}

TEST_F(SymbolicExpressionToLatexTest, MinMax) {
  EXPECT_TRUE(Check(min(x_, y_), R"""(\min(x, y))"""));
  EXPECT_TRUE(Check(max(x_, y_), R"""(\max(x, y))"""));
}

TEST_F(SymbolicExpressionToLatexTest, CeilFloor) {
  EXPECT_TRUE(Check(ceil(x_), R"""(\lceil{x}\rceil)"""));
  EXPECT_TRUE(Check(floor(x_), R"""(\lfloor{x}\rfloor)"""));
}

TEST_F(SymbolicExpressionToLatexTest, IfThenElse) {
  string dummy;
  EXPECT_THROW(dummy = if_then_else(x_ > y_, x_, y_).ToLatex(), runtime_error);
}

TEST_F(SymbolicExpressionToLatexTest, UninterpretedFunction) {
  EXPECT_TRUE(Check(uninterpreted_function("MyFunc", {x_, y_}),
                    R"""(\mathrm{MyFunc}(x, y))"""));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
