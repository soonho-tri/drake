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

  // Generated Code
  //
  // void f(const double* p, int* cols, int* rows, double* values) {
  //     cols[0] = 0;
  //     rows[0] = 0;
  //     values[0] = (0 + p[0] + p[1]);
  //     cols[1] = 2;
  //     rows[1] = 1;
  //     values[1] = (p[0] / 2.000000);
  //     cols[2] = 3;
  //     rows[2] = 2;
  //     values[2] = pow(p[1], 2.000000);
  // }
  using FuncType = void (*)(const double*, int*, int*, double*);
  const Eigen::Vector2d param{1 /* x */, 2 /* y */};
  const Environment env = {{x, 1.0}, {y, 2.0}};
  FuncType f = context.GetSymbol<FuncType>("f");

  std::array<int, 3> cols;
  std::array<int, 3> rows;
  std::array<double, 3> values;
  f(param.data(), cols.data(), rows.data(), values.data());

  // Construct an evaluated matrix.
  Eigen::SparseMatrix<double> m_double(m.rows(), m.cols());
  for (int i = 0; i < 3; ++i) {
    m_double.insert(rows[i], cols[i]) = values[i];
  }
  m_double.makeCompressed();

  // Construct the expected one.
  Eigen::SparseMatrix<double> m_double_expected(m.rows(), m.cols());
  m_double_expected.insert(0, 0) = m.coeffRef(0, 0).Evaluate(env);
  m_double_expected.insert(2, 3) = m.coeffRef(2, 3).Evaluate(env);
  m_double_expected.insert(1, 2) = m.coeffRef(1, 2).Evaluate(env);

  // Make sure the two are the same.
  EXPECT_EQ((m_double - m_double_expected).norm(), 0.0);
}

GTEST_TEST(SymbolicCodeGenTest, SparseMatrix2) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};
  // | x  0  0  0  z  0|
  // | 0  0  y  0  0  x|
  // | 0  0  0  y  0  y|
  Eigen::SparseMatrix<Expression> m(3, 6);
  m.insert(0, 0) = x;
  m.insert(0, 4) = z;
  m.insert(1, 2) = y;
  m.insert(1, 5) = x;
  m.insert(2, 3) = y;
  m.insert(2, 5) = y;
  m.makeCompressed();

  TccContext context;
  context.AddCode(CodeGen2("f", {x, y, z}, m));
  context.Compile();
  // Generated Code
  //
  // void f(const double* p, int* outerindices, int* innerindices,
  //        double* values) {
  //   outerindices[0] = 0;
  //   outerindices[1] = 1;
  //   outerindices[2] = 1;
  //   outerindices[3] = 2;
  //   outerindices[4] = 3;
  //   outerindices[5] = 4;
  //   outerindices[6] = 6;
  //
  //   innerindices[0] = 0;
  //   innerindices[1] = 1;
  //   innerindices[2] = 2;
  //   innerindices[3] = 0;
  //   innerindices[4] = 1;
  //   innerindices[5] = 2;
  //
  //   values[0] = pow(p[0], 2.000000);
  //   values[1] = p[1];
  //   values[2] = p[1];
  //   values[3] = p[2];
  //   values[4] = (0 + p[0] + p[1]);
  //   values[5] = (p[1] / 2.000000);
  // }

  using FuncType = void (*)(const double*, int*, int*, double*);
  FuncType f = context.GetSymbol<FuncType>("f");
  const Eigen::Vector3d param{1 /* x */, 2 /* y */, 3 /* z */};
  const Environment env = {{x, 1.0}, {y, 2.0}, {z, 3.0}};
  std::array<int, 6 + 1 /* cols + 1 */> outerindices;
  std::array<int, 6 /* nonzeros */> innerindices;
  std::array<double, 6 /* nonzeros */> values;
  f(param.data(), outerindices.data(), innerindices.data(), values.data());

  // Construct an evaluated matrix using Eigen::Map.
  Eigen::Map<Eigen::SparseMatrix<double>> map_sp(
      m.rows(), m.cols(), m.nonZeros(), outerindices.data(),
      innerindices.data(), values.data());
  Eigen::SparseMatrix<double> m_double = map_sp.eval();

  // Construct the expected one.
  Eigen::SparseMatrix<double> m_double_expected(m.rows(), m.cols());
  m_double_expected.insert(0, 0) = m.coeffRef(0, 0).Evaluate(env);
  m_double_expected.insert(0, 4) = m.coeffRef(0, 4).Evaluate(env);
  m_double_expected.insert(1, 2) = m.coeffRef(1, 2).Evaluate(env);
  m_double_expected.insert(1, 5) = m.coeffRef(1, 5).Evaluate(env);
  m_double_expected.insert(2, 3) = m.coeffRef(2, 3).Evaluate(env);
  m_double_expected.insert(2, 5) = m.coeffRef(2, 5).Evaluate(env);

  // Make sure the two are the same.
  EXPECT_EQ((m_double - m_double_expected).norm(), 0.0);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
