#include "drake/multibody/rational_forward_kinematics/rational_forward_kinematics.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"

namespace drake {
namespace multibody {
namespace {
using symbolic::RationalFunction;
using symbolic::Polynomial;
using symbolic::test::PolyEqualAfterExpansion;

void CheckReplaceCosAndSinWithRationalFunction(
    const symbolic::Expression& e, const VectorX<symbolic::Variable>& cos_delta,
    const VectorX<symbolic::Variable>& sin_delta,
    const VectorX<symbolic::Variable>& t_angle, const symbolic::Variables& t,
    const symbolic::RationalFunction& e_rational_expected) {
  symbolic::RationalFunction e_rational;
  ReplaceCosAndSinWithRationalFunction(e, cos_delta, sin_delta, t_angle, t,
                                       &e_rational);
  EXPECT_PRED2(PolyEqualAfterExpansion, e_rational.numerator(),
               e_rational_expected.numerator());
  EXPECT_PRED2(PolyEqualAfterExpansion, e_rational.denominator(),
               e_rational_expected.denominator());
}

GTEST_TEST(RationalForwardKinematics, ReplaceCosAndSinWithRationalFunction) {
  VectorX<symbolic::Variable> cos_delta(3);
  VectorX<symbolic::Variable> sin_delta(3);
  VectorX<symbolic::Variable> t_angle(3);
  for (int i = 0; i < 3; ++i) {
    cos_delta(i) =
        symbolic::Variable("cos(delta_q(" + std::to_string(i) + "))");
    sin_delta(i) =
        symbolic::Variable("sin(delta_q(" + std::to_string(i) + "))");
    t_angle(i) = symbolic::Variable("t_angle(" + std::to_string(i) + ")");
  }
  symbolic::Variable a("a");
  symbolic::Variable b("b");

  symbolic::Variables t(t_angle);

  // test cos(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      cos_delta(0), cos_delta, sin_delta, t_angle, t,
      symbolic::RationalFunction(Polynomial(1 - t_angle(0) * t_angle(0)),
                                 Polynomial(1 + t_angle(0) * t_angle(0))));
  // test sin(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      sin_delta(0), cos_delta, sin_delta, t_angle, t,
      symbolic::RationalFunction(Polynomial(2 * t_angle(0)),
                                 Polynomial(1 + t_angle(0) * t_angle(0))));
  // test 1.
  CheckReplaceCosAndSinWithRationalFunction(1, cos_delta, sin_delta, t_angle, t,
                                            symbolic::RationalFunction(1));

  // test a + b
  CheckReplaceCosAndSinWithRationalFunction(
      a + b, cos_delta, sin_delta, t_angle, t,
      RationalFunction(Polynomial(a + b, t)));

  // test 1 + cos(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      1 + cos_delta(0), cos_delta, sin_delta, t_angle, t,
      symbolic::RationalFunction(Polynomial(2),
                                 Polynomial(1 + t_angle(0) * t_angle(0))));

  // test a + b*cos(delta_q(0)) + sin(delta_q(1))
  CheckReplaceCosAndSinWithRationalFunction(
      a + b * cos_delta(0) + sin_delta(1), cos_delta, sin_delta, t_angle, t,
      RationalFunction(
          Polynomial(a * (1 + t_angle(0) * t_angle(0)) *
                             (1 + t_angle(1) * t_angle(1)) +
                         b * (1 - t_angle(0) * t_angle(0)) *
                             (1 + t_angle(1) * t_angle(1)) +
                         2 * t_angle(1) * (1 + t_angle(0) * t_angle(0)),
                     t),
          Polynomial((1 + t_angle(0) * t_angle(0)) *
                     (1 + t_angle(1) * t_angle(1)))));

  // test a + b * cos(delta_q(0) * sin(delta_q(1)) + sin(delta_q(0))
  CheckReplaceCosAndSinWithRationalFunction(
      a + b * cos_delta(0) * sin_delta(1) + sin_delta(0), cos_delta, sin_delta,
      t_angle, t,
      RationalFunction(
          Polynomial(a * (1 + t_angle(0) * t_angle(0)) *
                             (1 + t_angle(1) * t_angle(1)) +
                         b * (1 - t_angle(0) * t_angle(0)) * 2 * t_angle(1) +
                         2 * t_angle(0) * (1 + t_angle(1) * t_angle(1)),
                     t),
          Polynomial((1 + t_angle(0) * t_angle(0)) *
                     (1 + t_angle(1) * t_angle(1)))));

  // test a + b * cos(delta_q(0)) * sin(delta_q(1)) + sin(delta_q(0)) *
  // cos(delta_q(2))
  CheckReplaceCosAndSinWithRationalFunction(
      a + b * cos_delta(0) * sin_delta(1) + sin_delta(0) * cos_delta(2),
      cos_delta, sin_delta, t_angle, t,
      RationalFunction(
          Polynomial(a * (1 + t_angle(0) * t_angle(0)) *
                             (1 + t_angle(1) * t_angle(1)) *
                             (1 + t_angle(2) * t_angle(2)) +
                         b * (1 - t_angle(0) * t_angle(0)) * 2 * t_angle(1) *
                             (1 + t_angle(2) * t_angle(2)) +
                         2 * t_angle(0) * (1 + t_angle(1) * t_angle(1)) *
                             (1 - t_angle(2) * t_angle(2)),
                     t),
          Polynomial((1 + t_angle(0) * t_angle(0)) *
                     (1 + t_angle(1) * t_angle(1)) *
                     (1 + t_angle(2) * t_angle(2)))));
}

std::unique_ptr<multibody_plant::MultibodyPlant<double>> ConstructIiwaPlant(
  const std::string& iiwa_sdf_name) {
const std::string file_path =
    "drake/manipulation/models/iiwa_description/sdf/" + iiwa_sdf_name;
auto plant = std::make_unique<multibody_plant::MultibodyPlant<double>>(0);
parsing::AddModelFromSdfFile(FindResourceOrThrow(file_path), plant.get());
plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));
plant->Finalize();
return plant;
}

void CheckRationalForwardKinematics(
  const RationalForwardKinematics& rational_forward_kinematics,
  const Eigen::Ref<const Eigen::VectorXd>& q_val,
  const Eigen::Ref<const Eigen::VectorXd>& q_star_val,
  const Eigen::Ref<const Eigen::VectorXd>& t_val) {
DRAKE_DEMAND(q_star_val.rows() ==
             rational_forward_kinematics.q_star().rows());
DRAKE_DEMAND(t_val.rows() == rational_forward_kinematics.t().rows());
auto context = rational_forward_kinematics.tree().CreateDefaultContext();

auto mbt_context = dynamic_cast<MultibodyTreeContext<double>*>(context.get());

mbt_context->get_mutable_positions() = q_val;

std::vector<Eigen::Isometry3d> X_WB_expected;

rational_forward_kinematics.tree().CalcAllBodyPosesInWorld(*mbt_context,
                                                           &X_WB_expected);

symbolic::Environment env;
for (int i = 0; i < q_star_val.rows(); ++i) {
  env.insert(rational_forward_kinematics.q_star()(i), q_star_val(i));
}
for (int i = 0; i < t_val.rows(); ++i) {
  env.insert(rational_forward_kinematics.t()(i), t_val(i));
}

for (int i = 1; i < rational_forward_kinematics.tree().num_bodies(); ++i) {
  Matrix3<double> R_WB_i;
  Vector3<double> p_WB_i;
  for (int m = 0; m < 3; ++m) {
    for (int n = 0; n < 3; ++n) {
      R_WB_i(m, n) =
          rational_forward_kinematics.R_WB(i)(m, n).numerator().Evaluate(
              env) /
          rational_forward_kinematics.R_WB(i)(m, n).denominator().Evaluate(
              env);
    }
    p_WB_i(m) =
        rational_forward_kinematics.p_WB(i)(m).numerator().Evaluate(env) /
        rational_forward_kinematics.p_WB(i)(m).denominator().Evaluate(env);
  }

  const double tol{1E-12};
  EXPECT_TRUE(CompareMatrices(R_WB_i, X_WB_expected[i].linear(), tol));
  EXPECT_TRUE(CompareMatrices(p_WB_i, X_WB_expected[i].translation(), tol));
}
}

GTEST_TEST(RationalForwardKinematicsTest, Iiwa) {
auto iiwa_plant = ConstructIiwaPlant("iiwa14_no_collision.sdf");
RationalForwardKinematics rational_forward_kinematics(iiwa_plant->tree());
EXPECT_EQ(rational_forward_kinematics.t().rows(), 7);

CheckRationalForwardKinematics(
    rational_forward_kinematics, Eigen::VectorXd::Zero(7),
    Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
