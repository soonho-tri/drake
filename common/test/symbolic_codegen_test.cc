#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

using std::cerr;
using std::endl;

namespace drake {
namespace symbolic {
namespace {

GTEST_TEST(SymbolicCodeGenTest, Scalar) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Expression e1{sin(x) + cos(2 * y) + 3 * x * y * x};
  const Expression e2{pow(2, y) + 3};

  TccContext context;

  // Generated Code
  // --------------
  // double f1(const double* p) {
  //   return (0 + (3 * (1 * pow(p[0], 2.000000) * p[1])) +
  //          sin(p[0]) + cos((2 * p[1])));
  // }
  context.AddCode(CodeGen("f1", {x, y}, e1));

  // Generated Code
  // --------------
  // double f2(const double* p) {
  //     return (3 + pow(2.000000, p[1]));
  // }
  context.AddCode(CodeGen("f2", {x, y}, e2));
  context.Compile();

  // a function pointer taking an array of double and returning a double value.
  using FuncType = double (*)(const double*);
  FuncType f1 = context.GetSymbol<FuncType>("f1");
  FuncType f2 = context.GetSymbol<FuncType>("f2");

  const Eigen::Vector2d param{1 /* x */, 2 /* y */};
  const Environment env = {{x, 1.0}, {y, 2.0}};

  EXPECT_EQ(f1(param.data()), e1.Evaluate(env));
  EXPECT_EQ(f2(param.data()), e2.Evaluate(env));
}

GTEST_TEST(SymbolicCodeGenTest, DenseMatrix) {
  const Variable x{"x"};
  const Variable y{"y"};
  Eigen::Matrix<Expression, 2, 2> m;
  m(0, 0) = x + y;
  m(0, 1) = x * x + 2 * y + y * y;
  m(1, 0) = 3 + x;
  m(1, 1) = 6 / y;

  TccContext context;

  // Generated Code
  // --------------
  // void f(const double* p, double* m) {
  //   m[0] = (0 + p[0] + p[1]);
  //   m[1] = (3 + p[0]);
  //   m[2] = (0 + (2 * p[1]) + pow(p[0], 2.000000) + pow(p[1], 2.000000));
  //   m[3] = (6.000000 / p[1]);
  // }
  context.AddCode(CodeGen("f", {x, y}, m));
  context.Compile();

  using FuncType = void (*)(const double*, double*);
  const Eigen::Vector2d param{1 /* x */, 2 /* y */};
  const Environment env = {{x, 1.0}, {y, 2.0}};
  FuncType f = context.GetSymbol<FuncType>("f");
  Eigen::Matrix<double, 2, 2> evaluated;
  f(param.data(), evaluated.data());

  EXPECT_EQ(evaluated(0, 0), m(0, 0).Evaluate(env));
  EXPECT_EQ(evaluated(0, 1), m(0, 1).Evaluate(env));
  EXPECT_EQ(evaluated(1, 0), m(1, 0).Evaluate(env));
  EXPECT_EQ(evaluated(1, 1), m(1, 1).Evaluate(env));
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
