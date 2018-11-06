#include <array>
#include <iostream>

#include <Eigen/Sparse>
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

GTEST_TEST(SymbolicCodeGenTest, SparseMatrix) {
  const Variable x{"x"};
  const Variable y{"y"};
  // | x+y 0   0   0 0|
  // |   0 0 x/2   0 0|
  // |   0 0   0 y*y 0|
  // |   0 0   0   0 0|
  Eigen::SparseMatrix<Expression> m(4, 4);
  m.insert(0, 0) = x + y;
  m.insert(2, 3) = y * y;
  m.insert(1, 2) = x / 2.0;

  TccContext context;
  context.AddCode(CodeGen("f", {x, y}, m));
  context.Compile();
  using FuncType = void (*)(const double*, int*, int*, double*);
  const Eigen::Vector2d param{1 /* x */, 2 /* y */};
  const Environment env = {{x, 1.0}, {y, 2.0}};
  FuncType f = context.GetSymbol<FuncType>("f");

  std::array<int, 3> cols;
  std::array<int, 3> rows;
  std::array<double, 3> values;
  f(param.data(), cols.data(), rows.data(), values.data());

  EXPECT_EQ(cols[0], 0);
  EXPECT_EQ(rows[0], 0);
  EXPECT_EQ(values[0], 1 + 2);

  EXPECT_EQ(cols[1], 2);
  EXPECT_EQ(rows[1], 1);
  EXPECT_EQ(values[1], 1.0 / 2);

  EXPECT_EQ(cols[2], 3);
  EXPECT_EQ(rows[2], 2);
  EXPECT_EQ(values[2], 2.0 * 2.0);
}

GTEST_TEST(SymbolicCodeGenTest, SparseMatrix2) {
  const Variable x{"x"};
  const Variable y{"y"};
  // | x+y 0   0   0 0|
  // |   0 0 x/2   0 0|
  // |   0 0   0 y*y 0|
  // |   0 0   0   0 0|
  Eigen::SparseMatrix<Expression> m(4, 4);
  m.insert(0, 0) = x + y;
  m.insert(2, 3) = y * y;
  m.insert(1, 2) = x / 2.0;
  m.makeCompressed();

  TccContext context;
  context.AddCode(CodeGen2("f", {x, y}, m));
  context.Compile();
  using FuncType = void (*)(const double*, double*);
  const Eigen::Vector2d param{1 /* x */, 2 /* y */};
  const Environment env = {{x, 1.0}, {y, 2.0}};
  FuncType f = context.GetSymbol<FuncType>("f");

  std::array<double, 3 /* nonzeros */> values;
  f(param.data(), values.data());

  Eigen::Map<Eigen::SparseMatrix<double>> map_sp(
      m.rows(), m.cols(), m.nonZeros(), m.outerIndexPtr(), m.innerIndexPtr(),
      values.data());

  Eigen::SparseMatrix<double> m_double = map_sp.eval();
  Eigen::SparseMatrix<double> m_double_expected(4, 4);
  m_double_expected.insert(0, 0) = 1 + 2;
  m_double_expected.insert(2, 3) = 2 * 2;
  m_double_expected.insert(1, 2) = 1 / 2.0;
  EXPECT_EQ((m_double - m_double_expected).norm(), 0.0);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
