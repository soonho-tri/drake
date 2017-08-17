#include "drake/common/eigen_types.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::ArrayXXd;
using Eigen::Array33d;
using Eigen::ArrayXd;
using Eigen::Array3d;

namespace drake {
namespace {

template <typename Derived>
void CheckTraits(bool is_vector) {
  std::string info = "Type: " + NiceTypeName::Get<Derived>() + "\n";
  EXPECT_TRUE(is_eigen_type<Derived>::value) << info;
  EXPECT_TRUE((is_eigen_scalar_same<Derived, double>::value)) << info;
  EXPECT_EQ(is_vector, (is_eigen_vector_of<Derived, double>::value)) << info;
  EXPECT_EQ(!is_vector, (is_eigen_nonvector_of<Derived, double>::value))
      << info;
}

// Test traits within eigen_types.h
GTEST_TEST(EigenTypesTest, TraitsPositive) {
  // MatrixBase<>
  CheckTraits<MatrixXd>(false);
  CheckTraits<Matrix3d>(false);
  CheckTraits<VectorXd>(true);
  CheckTraits<Vector3d>(true);
  // ArrayBase<>
  CheckTraits<ArrayXXd>(false);
  CheckTraits<Array33d>(false);
  CheckTraits<ArrayXd>(true);
  CheckTraits<Array3d>(true);
}

// SFINAE check.
bool IsEigen(...) {
  return false;
}
template <typename T,
          typename Cond = typename std::enable_if<
              is_eigen_type<T>::value>::type>
bool IsEigen(const T&) {
  return true;
}

// SFINAE check with edge case described below.
bool IsEigenOfDouble(...) {
  return false;
}
template <typename T,
          typename Cond = typename std::enable_if<
              is_eigen_scalar_same<T, double>::value>::type>
bool IsEigenOfDouble(const T&) {
  return true;
}

// Ensure that we do not capture non-eigen things.
template <typename Derived>
class NonEigenBase {};
class NonEigen : public NonEigenBase<NonEigen> {
 public:
  typedef double Scalar;
};

GTEST_TEST(EigenTypesTest, TraitsSFINAE) {
  EXPECT_TRUE(IsEigen(MatrixXd()));
  EXPECT_TRUE(IsEigenOfDouble(MatrixXd()));
  EXPECT_TRUE(IsEigen(ArrayXXd()));
  EXPECT_TRUE(IsEigenOfDouble(ArrayXXd()));
  EXPECT_FALSE(IsEigen(NonEigen()));
  EXPECT_FALSE(IsEigenOfDouble(NonEigen()));
  EXPECT_FALSE(IsEigen(1));
  // TODO(eric.cousineau): See if there is a way to short-circuit conditionals.
  // Presently, the following code will throw an error, even in SFINAE.
  // EXPECT_FALSE(IsEigenOfDouble(1));
  // EXPECT_FALSE((is_eigen_vector_of<std::string, double>::value));
}

// Sets M(i, j) = c.
void set(EigenPtr<MatrixXd> M, const int i, const int j, const double c) {
  (*M)(i, j) = c;
}

// Returns M(i, j).
double get(const EigenPtr<const MatrixXd> M, const int i, const int j) {
  return M->coeff(i, j);
}

GTEST_TEST(EigenTypesTest, EigenPtr) {
  Eigen::MatrixXd M1 = Eigen::MatrixXd::Zero(3, 3);
  const Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(3, 3);

  // Tests set.
  set(&M1, 0, 0, 1);       // Sets M1(0,0) = 1
  EXPECT_EQ(M1(0, 0), 1);  // Checks M1(0, 0) = 1

  // Tests get.
  EXPECT_EQ(get(&M1, 0, 0), 1);  // Checks M1(0, 0) = 1
  EXPECT_EQ(get(&M2, 0, 0), 0);  // Checks M2(0, 0) = 1

  // Shows how to use EigenPtr with .block(). Here we introduce `tmp` to avoid
  // taking the address of temporary object.
  auto tmp = M1.block(1, 1, 2, 2);
  set(&tmp, 0, 0, 1);  // tmp(0, 0) = 1. That is, M1(1, 1) = 1.
  EXPECT_EQ(get(&M1, 1, 1), 1);
}
}  // namespace
}  // namespace drake
