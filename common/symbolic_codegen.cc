#include "drake/common/symbolic_codegen.h"

#include <sstream>
#include <stdexcept>

namespace drake {
namespace symbolic {

using std::ostringstream;
using std::runtime_error;
using std::string;
using std::to_string;

CodeGenVisitor::CodeGenVisitor(const IdToIndexMap& id_to_idx_map)
    : id_to_idx_map_{id_to_idx_map} {}

string CodeGenVisitor::CodeGen(const Expression& e) const {
  return VisitExpression<string>(this, e);
}

string CodeGenVisitor::VisitVariable(const Expression& e) const {
  const Variable& v{get_variable(e)};
  const auto it{id_to_idx_map_.find(v.get_id())};
  if (it == id_to_idx_map_.end()) {
    throw runtime_error("Variable index is not found.");
  }
  return "p[" + to_string(it->second) + "]";
}

string CodeGenVisitor::VisitConstant(const Expression& e) const {
  return to_string(get_constant_value(e));
}

string CodeGenVisitor::VisitAddition(const Expression& e) const {
  const double c{get_constant_in_addition(e)};
  const auto& expr_to_coeff_map{get_expr_to_coeff_map_in_addition(e)};
  ostringstream oss;
  oss << "(" << c;
  for (const auto& item : expr_to_coeff_map) {
    const Expression& e_i{item.first};
    const double c_i{item.second};
    oss << " + ";
    if (c_i == 1.0) {
      oss << CodeGen(e_i);
    } else {
      oss << "(" << c_i << " * " << CodeGen(e_i) << ")";
    }
  }
  oss << ")";
  return oss.str();
}

string CodeGenVisitor::VisitMultiplication(const Expression& e) const {
  const double c{get_constant_in_multiplication(e)};
  const auto& base_to_exponent_map{
      get_base_to_exponent_map_in_multiplication(e)};
  ostringstream oss;
  oss << "(" << c;
  for (const auto& item : base_to_exponent_map) {
    const Expression& e_1{item.first};
    const Expression& e_2{item.second};
    oss << " * ";
    if (is_one(e_2)) {
      oss << CodeGen(e_1);
    } else {
      oss << "pow(" << CodeGen(e_1) << ", " << CodeGen(e_2) << ")";
    }
  }
  oss << ")";
  return oss.str();
}

// Helper method to handle unary cases.
string CodeGenVisitor::VisitUnary(const string& f, const Expression& e) const {
  return f + "(" + CodeGen(get_argument(e)) + ")";
}

// Helper method to handle binary cases.
string CodeGenVisitor::VisitBinary(const string& f, const Expression& e) const {
  return f + "(" + CodeGen(get_first_argument(e)) + ", " +
         CodeGen(get_second_argument(e)) + ")";
}

string CodeGenVisitor::VisitPow(const Expression& e) const {
  return VisitBinary("pow", e);
}

string CodeGenVisitor::VisitDivision(const Expression& e) const {
  return "(" + CodeGen(get_first_argument(e)) + " / " +
         CodeGen(get_second_argument(e)) + ")";
}

string CodeGenVisitor::VisitAbs(const Expression& e) const {
  return VisitUnary("abs", e);
}

string CodeGenVisitor::VisitLog(const Expression& e) const {
  return VisitUnary("log", e);
}

string CodeGenVisitor::VisitExp(const Expression& e) const {
  return VisitUnary("exp", e);
}

string CodeGenVisitor::VisitSqrt(const Expression& e) const {
  return VisitUnary("sqrt", e);
}

string CodeGenVisitor::VisitSin(const Expression& e) const {
  return VisitUnary("sin", e);
}

string CodeGenVisitor::VisitCos(const Expression& e) const {
  return VisitUnary("cos", e);
}

string CodeGenVisitor::VisitTan(const Expression& e) const {
  return VisitUnary("tan", e);
}

string CodeGenVisitor::VisitAsin(const Expression& e) const {
  return VisitUnary("asin", e);
}

string CodeGenVisitor::VisitAcos(const Expression& e) const {
  return VisitUnary("acos", e);
}

string CodeGenVisitor::VisitAtan(const Expression& e) const {
  return VisitUnary("atan", e);
}

string CodeGenVisitor::VisitAtan2(const Expression& e) const {
  return VisitBinary("atan2", e);
}

string CodeGenVisitor::VisitSinh(const Expression& e) const {
  return VisitUnary("sinh", e);
}

string CodeGenVisitor::VisitCosh(const Expression& e) const {
  return VisitUnary("cosh", e);
}

string CodeGenVisitor::VisitTanh(const Expression& e) const {
  return VisitUnary("tanh", e);
}

string CodeGenVisitor::VisitMin(const Expression& e) const {
  return VisitBinary("min", e);
}

string CodeGenVisitor::VisitMax(const Expression& e) const {
  return VisitBinary("max", e);
}

string CodeGenVisitor::VisitCeil(const Expression& e) const {
  return VisitUnary("ceil", e);
}

string CodeGenVisitor::VisitFloor(const Expression& e) const {
  return VisitUnary("floor", e);
}

string CodeGenVisitor::VisitIfThenElse(const Expression&) const {
  throw runtime_error("TODO");
}

string CodeGenVisitor::VisitUninterpretedFunction(const Expression&) const {
  throw runtime_error("TODO");
}

TccContext::TccContext() : s_{tcc_new()} {
  if (s_ == nullptr) {
    throw runtime_error("Fail to create a TccContext.");
  }
  tcc_set_options(s_, "-w");              // Suppress all warnings
  tcc_set_options(s_, "-D__OPTIMIZE__");  // Turn on optimization
  tcc_set_output_type(s_, TCC_OUTPUT_MEMORY);
}
TccContext::~TccContext() { tcc_delete(s_); }

void TccContext::AddCode(const string& code) { code_ += code; }

void TccContext::Compile() {
  code_ = "#include <math.h>\n" + code_;
  if (tcc_compile_string(s_, code_.c_str()) == -1) {
    throw runtime_error("Fail to compile the code:\n" + code_);
  }
  if (tcc_relocate(s_, reinterpret_cast<void*>(1) /* TCC_RELOCATE_AUTO */) <
      0) {
    throw runtime_error("Fail to relocate.");
  }
}

}  // namespace symbolic
}  // namespace drake
