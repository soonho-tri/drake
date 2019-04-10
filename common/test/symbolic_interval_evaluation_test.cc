#include "drake/common/symbolic_interval_evaluation.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/hash.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;
using test::ExprLess;
using test::ExprNotEqual;
using test::ExprNotLess;
using test::FormulaEqual;

GTEST_TEST(SymbolicIntervalEvaluation, Test1) {
  const Variable x{"x"};
  const Variable y{"y"};
  const Variable z{"z"};

  // env = {x: [0, 1],
  //        y: [1, 3],
  //        z: [2, 4]}
  IntervalEnvironment env;
  env.emplace(x, Interval{0, 1});
  env.emplace(y, Interval{1, 3});
  env.emplace(z, Interval{2, 4});

  // e = x² + sin(y + z).
  const Expression e{x * x + sin(y + z)};
  const Interval intv{Evaluate(e, env)};

  // intv is approximately [-1, 1.65699].
  EXPECT_NEAR(intv.lb(), -1, 1e-5);
  EXPECT_NEAR(intv.ub(), 1.65699, 1e-5);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
