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
      EXPECT_PRED2(PolyFractionEqual, m1(i, j), m2(i, j));
    }
  }
}
TEST_F(SymbolicPolynomialFractionMatrixTest, Addition) {
  // matrix + matrix.
  Matrix2<PolynomialFraction> M2;
  M2 << PolynomialFraction(p2_, p3_ + 2 * p4_),
      PolynomialFraction(p1_ + p2_, 2 * p3_),
      PolynomialFraction(p1_, p4_ - p5_), PolynomialFraction(p2_, p3_ * p6_);
  Matrix2<PolynomialFraction> M_plus_M2_expected;
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      M_plus_M2_expected(i, j) = M_poly_fraction_static_(i, j) + M2(i, j);
    }
  }
  CompareMatrixWithPolynomialFraction(M_poly_fraction_static_ + M2,
                                      M_plus_M2_expected);
  CompareMatrixWithPolynomialFraction(M_poly_fraction_dynamic_ + M2,
                                      M_plus_M2_expected);
}
}  // namespace symbolic
}  // namespace drake
