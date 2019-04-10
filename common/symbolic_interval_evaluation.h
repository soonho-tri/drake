#pragma once

#include <unordered_map>

#include "drake/common/drake_interval.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

/// Mapping from a symbolic variable to an interval.
using IntervalEnvironment = std::unordered_map<Variable, Interval>;

/// Evaluates a symbolic expression @p e using an interval environment @p env
/// which maps a symbolic variable into an interval.
Interval Evaluate(const Expression& e, const IntervalEnvironment& env);

}  // namespace symbolic
}  // namespace drake
