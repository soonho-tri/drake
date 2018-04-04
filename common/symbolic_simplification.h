#pragma once

#include <functional>

#include "drake/common/drake_optional.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

/// If the expression @p e is matched with sin²(X), then returns 1 - cos²(X).
/// Otherwise returns @p e.
Expression SinSquare(const Expression& e);

/// If the expression @p e is matched with cos²(X), then returns 1 - sin²(X).
/// Otherwise returns @p e.
Expression CosSquare(const Expression& e);

/// If the expression @p e is matched with sin(X + Y), then returns sin(X)cos(Y)
/// + cos(X)sin(Y). Otherwise returns @p e.
Expression SinSumOfAngle(const Expression& e);

/// If the expression @p e is matched with cos(X + Y), then returns cos(X)cos(Y)
/// - sin(X)sin(Y). Otherwise returns @p e.
Expression CosSumOfAngle(const Expression& e);

using Pattern = std::function<Expression(const Expression&)>;
Pattern MakeCongruence(const Pattern& pattern);

Pattern MakeTry(const Pattern& pattern1, const Pattern& pattern2);

template <typename... Rest>
Pattern MakeTry(const Pattern& pattern1, const Pattern& pattern2,
                Rest... rest) {
  return MakeTry(MakeTry(pattern1, pattern2), rest...);
}

Pattern MakeFixpoint(const Pattern& pattern);

}  // namespace symbolic
}  // namespace drake
