#include "drake/common/symbolic_codegen.h"

#include <stdexcept>
#include <unordered_map>

#include <fmt/format.h>
#include <inja/inja.hpp>
#include <nlohmann/json.hpp>

namespace drake {
namespace symbolic {

using std::ostream;
using std::ostringstream;
using std::runtime_error;
using std::string;
using std::to_string;
using std::unordered_map;
using std::vector;

namespace {

// Visitor class for code generation.
class CodeGenVisitor {
 public:
  using IdToIndexMap = unordered_map<Variable::Id, vector<Variable>::size_type>;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CodeGenVisitor)

  // Constructs an instance of this visitor class using the vector of
  // variables, @p parameters. This visitor will map a symbolic variable `var`
  // into `p[n]` where `n` is the index of the variable `var` in the given @p
  /// parameters.
  explicit CodeGenVisitor(const vector<Variable>& parameters);

  // Generates C expression for the expression @p e.
  string CodeGen(const Expression& e) const;

 private:
  string VisitVariable(const Expression& e) const;
  string VisitConstant(const Expression& e) const;
  string VisitAddition(const Expression& e) const;
  string VisitMultiplication(const Expression& e) const;
  // Helper method to handle unary cases.
  string VisitUnary(const string& f, const Expression& e) const;
  // Helper method to handle binary cases.
  string VisitBinary(const string& f, const Expression& e) const;
  string VisitPow(const Expression& e) const;
  string VisitDivision(const Expression& e) const;
  string VisitAbs(const Expression& e) const;
  string VisitLog(const Expression& e) const;
  string VisitExp(const Expression& e) const;
  string VisitSqrt(const Expression& e) const;
  string VisitSin(const Expression& e) const;
  string VisitCos(const Expression& e) const;
  string VisitTan(const Expression& e) const;
  string VisitAsin(const Expression& e) const;
  string VisitAcos(const Expression& e) const;
  string VisitAtan(const Expression& e) const;
  string VisitAtan2(const Expression& e) const;
  string VisitSinh(const Expression& e) const;
  string VisitCosh(const Expression& e) const;
  string VisitTanh(const Expression& e) const;
  string VisitMin(const Expression& e) const;
  string VisitMax(const Expression& e) const;
  string VisitCeil(const Expression& e) const;
  string VisitFloor(const Expression& e) const;
  string VisitIfThenElse(const Expression& e) const;
  string VisitUninterpretedFunction(const Expression& e) const;
  // Makes VisitExpression a friend of this class so that it can use private
  // methods.
  friend string VisitExpression<string>(const CodeGenVisitor*,
                                        const Expression&);

  IdToIndexMap id_to_idx_map_;
};

CodeGenVisitor::CodeGenVisitor(const vector<Variable>& parameters) {
  for (vector<Variable>::size_type i = 0; i < parameters.size(); ++i) {
    id_to_idx_map_.emplace(parameters[i].get_id(), i);
  }
}

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
  return VisitUnary("fabs", e);
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
  return VisitBinary("fmin", e);
}

string CodeGenVisitor::VisitMax(const Expression& e) const {
  return VisitBinary("fmax", e);
}

string CodeGenVisitor::VisitCeil(const Expression& e) const {
  return VisitUnary("ceil", e);
}

string CodeGenVisitor::VisitFloor(const Expression& e) const {
  return VisitUnary("floor", e);
}

string CodeGenVisitor::VisitIfThenElse(const Expression&) const {
  throw runtime_error("Codegen does not support if-then-else expressions.");
}

string CodeGenVisitor::VisitUninterpretedFunction(const Expression&) const {
  throw runtime_error("Codegen does not support uninterpreted functions.");
}

}  // namespace

string CodeGen(const string& function_name, const vector<Variable>& parameters,
               const Expression& e) {
  const string templ{
      R"(double {{ function_name }}(const double* p) {
  return {{ code }};
}
typedef struct {
  /* p: input, vector */
  struct {
    int size;
  } p;
} {{ function_name }}_meta_t;
aaa {{ function_name }}_meta_t {{ function_name }}_meta() { return {{{{ {{ parameters_size }} }}; }
)"};
  const nlohmann::json json_data{
      {"function_name", function_name},
      {"code", CodeGenVisitor{parameters}.CodeGen(e)},
      {"parameters_size", parameters.size()},
  };
  return inja::render(templ, json_data);
}

namespace internal {
void CodeGenDenseData(const string& function_name,
                      const vector<Variable>& parameters,
                      const Expression* const data, const int size,
                      ostream* const os) {
  const string templ{
      R"(void {{ function_name }}(const double* {{ parameter_name }}, double* {{ matrix_name }}) {
## for item in items
  {{ matrix_name }}[{{ item.idx }}] = {{ item.code }};
## endfor
}
)"};
  nlohmann::json json_data{
      {"function_name", function_name},
      {"parameter_name", ""},  // Update it below if used in the body.
      {"matrix_name", "m"},
      {"items", nlohmann::json::array() /* empty */},
  };
  const CodeGenVisitor visitor{parameters};
  bool include_parameter{false};
  for (int i = 0; i < size; ++i) {
    if (!include_parameter && !data[i].GetVariables().empty()) {
      include_parameter = true;
    }
    json_data["items"].push_back(
        {{"idx", i}, {"code", visitor.CodeGen(data[i])}});
  }
  if (include_parameter) {
    json_data["parameter_name"] = "p";
  }
  inja::render_to(*os, templ, json_data);
}  // namespace internal

void CodeGenDenseMeta(const string& function_name, const int parameter_size,
                      const int rows, const int cols, ostream* const os) {
  // <function_name>_meta_t type.
  (*os) << "typedef struct {\n"
           "    /* p: input, vector */\n"
           "    struct { int size; } p;\n"
           "    /* m: output, matrix */\n"
           "    struct { int rows; int cols; } m;\n"
           "} "
        << function_name << "_meta_t;\n";
  // <function_name>_meta().
  (*os) << function_name << "_meta_t " << function_name << "_meta() { return {{"
        << parameter_size << "}, {" << rows << ", " << cols << "}}; }\n";
}

void CodeGenSparseData(const string& function_name,
                       const vector<Variable>& parameters,
                       const int outer_index_size, const int non_zeros,
                       const int* const outer_index_ptr,
                       const int* const inner_index_ptr,
                       const Expression* const value_ptr, ostream* const os) {
  // Print header.
  (*os) << fmt::format(
      "void {}(const double* p, int* outer_indices, int* "
      "inner_indices, double* values) {{\n",
      function_name);

  for (int i = 0; i < outer_index_size; ++i) {
    (*os) << fmt::format("    outer_indices[{0}] = {1};\n", i,
                         outer_index_ptr[i]);
  }
  for (int i = 0; i < non_zeros; ++i) {
    (*os) << fmt::format("    inner_indices[{0}] = {1};\n", i,
                         inner_index_ptr[i]);
  }
  const CodeGenVisitor visitor{parameters};
  for (int i = 0; i < non_zeros; ++i) {
    (*os) << fmt::format("    values[{0}] = {1};\n", i,
                         visitor.CodeGen(value_ptr[i]));
  }
  // Print footer.
  (*os) << "}\n";
}

void CodeGenSparseMeta(const string& function_name, const int parameter_size,
                       const int rows, const int cols, const int non_zeros,
                       const int outer_indices, const int inner_indices,
                       ostream* const os) {
  // <function_name>_meta_t type.
  (*os) << "typedef struct {\n"
           "    /* p: input, vector */\n"
           "    struct { int size; } p;\n"
           "    /* m: output, matrix */\n"
           "    struct {\n"
           "        int rows;\n"
           "        int cols;\n"
           "        int non_zeros;\n"
           "        int outer_indices;\n"
           "        int inner_indices;\n"
           "    } m;\n"
           "} "
        << function_name << "_meta_t;\n";
  // <function_name>_meta().
  (*os) << fmt::format(
      "{0}_meta_t {1}_meta() {{ return {{{{{2}}}, {{{3}, {4}, {5}, {6}, "
      "{7}}}}}; }}\n",
      function_name, function_name, parameter_size, rows, cols, non_zeros,
      outer_indices, inner_indices);
}

}  // namespace internal

std::string CodeGen(
    const std::string& function_name, const std::vector<Variable>& parameters,
    const Eigen::Ref<const Eigen::SparseMatrix<Expression>>& M) {
  DRAKE_ASSERT(M.isCompressed());
  ostringstream oss;
  internal::CodeGenSparseData(function_name, parameters, M.cols() + 1,
                              M.nonZeros(), M.outerIndexPtr(),
                              M.innerIndexPtr(), M.valuePtr(), &oss);
  internal::CodeGenSparseMeta(function_name, parameters.size(), M.rows(),
                              M.cols(), M.nonZeros(), M.cols() + 1,
                              M.nonZeros(), &oss);
  return oss.str();
}

}  // namespace symbolic
}  // namespace drake
