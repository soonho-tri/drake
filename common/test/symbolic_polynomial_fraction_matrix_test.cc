#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
using test::PolyFractionEqual;

class SymbolicPolynomialFractionMatrixTest : public ::testing::Test {
 protected:
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Variables var_xyz_{{var_x_, var_y_, var_z_}};

  const Variable var_a_{"a"};
  const Variable var_b_{"b"};
  const Variable var_c_{"c"};
  const Variables var_abc_{{var_a_, var_b_, var_c_}};

  const Polynomial p1_{var_x_ * var_x_};
  const Polynomial p2_{var_x_ * var_y_ + 2 * var_y_ + 1};
  const Polynomial p3_{var_x_ * var_z_ + var_a_ * var_x_ * var_x_, var_xyz_};
  const Polynomial p4_{var_a_ * var_x_ * var_y_ + var_b_ * var_x_ * var_z_,
                       var_xyz_};
  const Polynomial p5_{var_a_ * var_x_ + var_b_ * 2 * var_z_ * var_y_,
                       var_xyz_};
  const Polynomial p6_{
      3 * var_a_ * var_x_ * var_x_ + var_b_ * 2 * var_z_ * var_y_, var_xyz_};

  MatrixX<PolynomialFraction> M_poly_fraction_dynamic_{2, 2};
  Matrix2<PolynomialFraction> M_poly_fraction_static_;
  VectorX<PolynomialFraction> v_poly_fraction_dynamic_{2};
  Vector2<PolynomialFraction> v_poly_fraction_static_;

  MatrixX<Polynomial> M_poly_dynamic_{2, 2};
  Matrix2<Polynomial> M_poly_static_;
  VectorX<Polynomial> v_poly_dynamic_{2};
  Vector2<Polynomial> v_poly_static_;

  Eigen::MatrixXd M_double_dynamic_{2, 2};
  Eigen::Matrix2d M_double_static_;
  Eigen::VectorXd v_double_dynamic_{2};
  Eigen::Vector2d v_double_static_;

  void SetUp() override {
    M_poly_fraction_dynamic_ << PolynomialFraction(p1_, p2_),
        PolynomialFraction(p1_, p3_), PolynomialFraction(p2_, p3_),
        PolynomialFraction(p1_, p2_ + p3_);
    M_poly_fraction_static_ = M_poly_fraction_dynamic_;
    v_poly_fraction_dynamic_ << PolynomialFraction(p2_, p4_),
        PolynomialFraction(p3_, p2_ + p4_);
    v_poly_fraction_static_ = v_poly_fraction_dynamic_;

    M_poly_dynamic_ << p1_, p2_, p5_, p6_;
    M_poly_static_ = M_poly_dynamic_;
    v_poly_dynamic_ << p1_, p3_;
    v_poly_static_ = v_poly_dynamic_;

    M_double_dynamic_ << 1, 2, 3, 4;
    M_double_static_ = M_double_dynamic_;
    v_double_dynamic_ << 1, 2;
    v_double_static_ = v_double_dynamic_;
  }
};

template <typename Derived1, typename Derived2>
typename std::enable_if<
    std::is_same<typename Derived1::Scalar, PolynomialFraction>::value &&
    std::is_same<typename Derived2::Scalar, PolynomialFraction>::value>::type
CompareMatrixWithPolynomialFraction(const Eigen::MatrixBase<Derived1>& m1,
                                    const Eigen::MatrixBase<Derived2>& m2) {
  EXPECT_EQ(m1.rows(), m2.rows());
  EXPECT_EQ(m1.cols(), m2.cols());
  for (int i = 0; i < m1.rows(); ++i) {
    for (int j = 0; j < m1.cols(); ++j) {
      EXPECT_PRED2(test::PolyEqualAfterExpansion, m1(i, j).numerator(),
                   m2(i, j).numerator());
      EXPECT_PRED2(test::PolyEqualAfterExpansion, m1(i, j).denominator(),
                   m2(i, j).denominator());
    }
  }
}

template <typename Derived1, typename Derived2>
void CheckAddition(const Eigen::MatrixBase<Derived1>& m1,
                   const Eigen::MatrixBase<Derived2>& m2) {
  DRAKE_DEMAND(m1.rows() == m2.rows());
  DRAKE_DEMAND(m1.cols() == m2.cols());
  MatrixX<PolynomialFraction> m1_add_m2_expected(m1.rows(), m1.cols());
  for (int i = 0; i < m1.rows(); ++i) {
    for (int j = 0; j < m1.cols(); ++j) {
      m1_add_m2_expected(i, j) = m1(i, j) + m2(i, j);
    }
  }
  auto m1_add_m2 = m1 + m2;
  static_assert(std::is_same<typename decltype(m1_add_m2)::Scalar,
                             PolynomialFraction>::value,
                "m1 + m2 should have scalar type PolynomialFraction.");
  CompareMatrixWithPolynomialFraction(m1_add_m2, m1_add_m2_expected);
  CompareMatrixWithPolynomialFraction(m2 + m1, m1_add_m2_expected);
}

template <typename Derived1, typename Derived2>
void CheckSubtraction(const Eigen::MatrixBase<Derived1>& m1,
                      const Eigen::MatrixBase<Derived2>& m2) {
  DRAKE_DEMAND(m1.rows() == m2.rows());
  DRAKE_DEMAND(m1.cols() == m2.cols());
  MatrixX<PolynomialFraction> m1_minus_m2_expected(m1.rows(), m1.cols());
  for (int i = 0; i < m1.rows(); ++i) {
    for (int j = 0; j < m1.cols(); ++j) {
      m1_minus_m2_expected(i, j) = m1(i, j) - m2(i, j);
    }
  }
  auto m1_minus_m2 = m1 - m2;
  static_assert(std::is_same<typename decltype(m1_minus_m2)::Scalar,
                             PolynomialFraction>::value,
                "m1 - m2 should have scalar type PolynomialFraction.");
  CompareMatrixWithPolynomialFraction(m1_minus_m2, m1_minus_m2_expected);
  CompareMatrixWithPolynomialFraction(m2 - m1, -m1_minus_m2_expected);
}

template <typename Derived1, typename Derived2>
void CheckProduct(const Eigen::MatrixBase<Derived1>& m1,
                  const Eigen::MatrixBase<Derived2>& m2) {
  DRAKE_DEMAND(m1.cols() == m2.rows());
  MatrixX<PolynomialFraction> m1_times_m2_expected(m1.rows(), m2.cols());
  for (int i = 0; i < m1.rows(); ++i) {
    for (int j = 0; j < m2.cols(); ++j) {
      for (int k = 0; k < m1.cols(); ++k) {
        m1_times_m2_expected(i, j) += m1(i, k) * m2(k, j);
      }
    }
  }
  auto m1_times_m2 = m1 * m2;

  static_assert(std::is_same<typename decltype(m1_times_m2)::Scalar,
                             PolynomialFraction>::value,
                "m1 * m2 should have scalar type PolynomialFraction.");
  CompareMatrixWithPolynomialFraction(m1_times_m2, m1_times_m2_expected);
}

template <typename Derived1, typename Derived2>
void CheckMatrixMatrixBinaryOperations(const Eigen::MatrixBase<Derived1>& m1,
                                       const Eigen::MatrixBase<Derived2>& m2) {
  CheckAddition(m1, m2);
  CheckSubtraction(m1, m2);
  CheckProduct(m1, m2);
  CheckProduct(m2, m1);
}

template <typename Derived1, typename Derived2>
typename std::enable_if<is_eigen_vector<Derived2>::value>::type
CheckMatrixVectorBinaryOperations(const Eigen::MatrixBase<Derived1>& m1,
                                  const Eigen::MatrixBase<Derived2>& m2) {
  // CheckProduct(m1, m2);
}

template <typename Derived1, typename Derived2>
typename std::enable_if<is_eigen_vector<Derived1>::value &&
                        is_eigen_vector<Derived2>::value>::type
CheckVectorVectorBinaryOperations(const Eigen::MatrixBase<Derived1>& m1,
                                  const Eigen::MatrixBase<Derived2>& m2) {
  CheckAddition(m1, m2);
  CheckSubtraction(m1, m2);
  // CheckConjugateProdocut(m1, m2);
}

TEST_F(SymbolicPolynomialFractionMatrixTest,
       PolynomialFractionOpPolynomialFraction) {
  Matrix2<PolynomialFraction> M2;
  M2 << PolynomialFraction(p2_, p3_ + 2 * p4_),
      PolynomialFraction(p1_ + p2_, 2 * p3_),
      PolynomialFraction(p1_, p4_ - p5_), PolynomialFraction(p2_, p3_ * p6_);

  CheckMatrixMatrixBinaryOperations(M_poly_fraction_static_, M2);
  CheckMatrixMatrixBinaryOperations(M_poly_fraction_dynamic_, M2);
  Vector2<PolynomialFraction> v2(PolynomialFraction(p1_, p4_ + 2 * p5_),
                                 PolynomialFraction(p3_, p2_ - p1_));
  CheckVectorVectorBinaryOperations(v_poly_fraction_static_, v2);
  CheckVectorVectorBinaryOperations(v_poly_fraction_dynamic_, v2);

  CheckMatrixVectorBinaryOperations(M_poly_fraction_static_,
                                    v_poly_fraction_static_);
  CheckMatrixVectorBinaryOperations(M_poly_fraction_dynamic_,
                                    v_poly_fraction_static_);
  CheckMatrixVectorBinaryOperations(M_poly_fraction_static_,
                                    v_poly_fraction_dynamic_);
  CheckMatrixVectorBinaryOperations(M_poly_fraction_dynamic_,
                                    v_poly_fraction_dynamic_);
}

TEST_F(SymbolicPolynomialFractionMatrixTest, PolynomialFractionOpPolynomial) {
  CheckMatrixMatrixBinaryOperations(M_poly_fraction_static_, M_poly_static_);
  CheckMatrixMatrixBinaryOperations(M_poly_fraction_static_, M_poly_dynamic_);
  CheckMatrixMatrixBinaryOperations(M_poly_fraction_dynamic_, M_poly_static_);
  // The 3 lines above are fine. But when we have the next line, we run into
  // compiler issue.
  CheckMatrixMatrixBinaryOperations(M_poly_fraction_dynamic_, M_poly_dynamic_);

  /*CheckVectorVectorBinaryOperations(v_poly_fraction_static_, v_poly_static_);
  CheckVectorVectorBinaryOperations(v_poly_fraction_static_, v_poly_dynamic_);
  CheckVectorVectorBinaryOperations(v_poly_fraction_dynamic_, v_poly_static_);
  CheckVectorVectorBinaryOperations(v_poly_fraction_dynamic_, v_poly_dynamic_);

  CheckMatrixVectorBinaryOperations(M_poly_fraction_static_, v_poly_static_);
  CheckMatrixVectorBinaryOperations(M_poly_fraction_static_, v_poly_dynamic_);
  CheckMatrixVectorBinaryOperations(M_poly_fraction_dynamic_, v_poly_static_);
  CheckMatrixVectorBinaryOperations(M_poly_fraction_dynamic_,
  v_poly_dynamic_);*/
}

TEST_F(SymbolicPolynomialFractionMatrixTest, PolynomialFractionOpDouble) {
  /*CheckMatrixMatrixBinaryOperations(M_poly_fraction_static_,
  M_double_static_);
  CheckMatrixMatrixBinaryOperations(M_poly_fraction_static_, M_double_dynamic_);
  CheckMatrixMatrixBinaryOperations(M_poly_fraction_dynamic_, M_double_static_);
  CheckMatrixMatrixBinaryOperations(M_poly_fraction_dynamic_,
                                    M_double_dynamic_);

  CheckVectorVectorBinaryOperations(v_poly_fraction_static_, v_double_static_);
  CheckVectorVectorBinaryOperations(v_poly_fraction_static_, v_double_dynamic_);
  CheckVectorVectorBinaryOperations(v_poly_fraction_dynamic_, v_double_static_);
  CheckVectorVectorBinaryOperations(v_poly_fraction_dynamic_,
                                    v_double_dynamic_);

  CheckMatrixVectorBinaryOperations(M_poly_fraction_static_, v_double_static_);
  CheckMatrixVectorBinaryOperations(M_poly_fraction_static_, v_double_dynamic_);
  CheckMatrixVectorBinaryOperations(M_poly_fraction_dynamic_, v_double_static_);
  CheckMatrixVectorBinaryOperations(M_poly_fraction_dynamic_,
                                    v_double_dynamic_);*/
}
}  // namespace symbolic
}  // namespace drake
