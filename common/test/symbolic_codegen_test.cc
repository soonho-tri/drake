#include <iostream>

#include <clang/Basic/DiagnosticOptions.h>
#include <clang/Driver/Compilation.h>
#include <clang/Driver/Driver.h>
#include <clang/Frontend/TextDiagnosticPrinter.h>
#include <fmt/format.h>
#include <gtest/gtest.h>
#include <llvm/Support/Host.h>
#include <llvm/Support/Program.h>
#include <llvm/Support/raw_ostream.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {
namespace {

using std::string;

// Helper function to combine the function name @p function_name and an
// expression @p e with the proper function header and footer.
string MakeFunctionCode(const string& function_name, int n, const string& e) {
  // Note that fmtlib requires to escape "{'" and "}" using "{{" and "}}".
  return fmt::format(
      R"""(double {}(const double* p) {{
    return {};
}}
int {}_in() {{
    return {};
}}
)""",
      function_name, e, function_name, n);
}

class SymbolicCodeGenTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
};

TEST_F(SymbolicCodeGenTest, Variable) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, x_), MakeFunctionCode("f", 2, "p[0]"));
  EXPECT_EQ(CodeGen("f", {x_, y_, z_}, z_), MakeFunctionCode("f", 3, "p[2]"));
}

TEST_F(SymbolicCodeGenTest, Constant) {
  EXPECT_EQ(CodeGen("f", {}, 3.141592), MakeFunctionCode("f", 0, "3.141592"));
}

TEST_F(SymbolicCodeGenTest, Addition) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, 2.0 + 3.0 * x_ - 7.0 * y_),
            MakeFunctionCode("f", 2, "(2 + (3 * p[0]) + (-7 * p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Multiplication) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, 2.0 * 3.0 * x_ * x_ * -7.0 * y_ * y_ * y_),
            MakeFunctionCode(
                "f", 2, "(-42 * pow(p[0], 2.000000) * pow(p[1], 3.000000))"));
}

TEST_F(SymbolicCodeGenTest, Pow) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, pow(2 + x_, 3 * y_)),
            MakeFunctionCode("f", 2, "pow((2 + p[0]), (3 * p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Division) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, (2 + x_) / (3 * y_)),
            MakeFunctionCode("f", 2, "((2 + p[0]) / (3 * p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Abs) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, abs(2 + x_)),
            MakeFunctionCode("f", 2, "fabs((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Log) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, log(2 + x_)),
            MakeFunctionCode("f", 2, "log((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Exp) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, exp(2 + x_)),
            MakeFunctionCode("f", 2, "exp((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Sqrt) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, sqrt(2 + x_)),
            MakeFunctionCode("f", 2, "sqrt((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Sin) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, sin(2 + x_)),
            MakeFunctionCode("f", 2, "sin((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Cos) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, cos(2 + x_)),
            MakeFunctionCode("f", 2, "cos((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Tan) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, tan(2 + x_)),
            MakeFunctionCode("f", 2, "tan((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Asin) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, asin(2 + x_)),
            MakeFunctionCode("f", 2, "asin((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Acos) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, acos(2 + x_)),
            MakeFunctionCode("f", 2, "acos((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Atan) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, atan(2 + x_)),
            MakeFunctionCode("f", 2, "atan((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Atan2) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, atan2(2 + x_, 3 + y_)),
            MakeFunctionCode("f", 2, "atan2((2 + p[0]), (3 + p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Sinh) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, sinh(2 + x_)),
            MakeFunctionCode("f", 2, "sinh((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Cosh) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, cosh(2 + x_)),
            MakeFunctionCode("f", 2, "cosh((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Tanh) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, tanh(2 + x_)),
            MakeFunctionCode("f", 2, "tanh((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Min) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, min(2 + x_, 3 + y_)),
            MakeFunctionCode("f", 2, "fmin((2 + p[0]), (3 + p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Max) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, max(2 + x_, 3 + y_)),
            MakeFunctionCode("f", 2, "fmax((2 + p[0]), (3 + p[1]))"));
}

TEST_F(SymbolicCodeGenTest, Ceil) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, ceil(2 + x_)),
            MakeFunctionCode("f", 2, "ceil((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, Floor) {
  EXPECT_EQ(CodeGen("f", {x_, y_}, floor(2 + x_)),
            MakeFunctionCode("f", 2, "floor((2 + p[0]))"));
}

TEST_F(SymbolicCodeGenTest, IfThenElse) {
  const Expression e{if_then_else(x_ > y_, x_, y_)};
  EXPECT_THROW(CodeGen("f", {x_, y_}, e), std::runtime_error);
}

TEST_F(SymbolicCodeGenTest, UninterpretedFunction) {
  const Expression e{uninterpreted_function("uf", {x_, y_})};
  EXPECT_THROW(CodeGen("f", {x_, y_}, e), std::runtime_error);
}

TEST_F(SymbolicCodeGenTest, Libclang) {
  // Path to the C file
  string inputPath = "getinmemory.c";

  // Path to the executable
  string outputPath = "getinmemory";

  // Path to clang (e.g. /usr/local/bin/clang)
  auto clangPath = llvm::sys::findProgramByName("clang");

  // Arguments to pass to the clang driver:
  // clang getinmemory.c -lcurl -v
  std::vector<const char*> args;
  args.push_back(clangPath->c_str());
  args.push_back(inputPath.c_str());
  args.push_back("-l");
  args.push_back("curl");
  args.push_back("-v");  // verbose

  clang::DiagnosticOptions options;

  // The clang driver needs a DiagnosticsEngine so it can report problems
  clang::TextDiagnosticPrinter* DiagClient =
      new clang::TextDiagnosticPrinter(llvm::errs(), &options);
  clang::IntrusiveRefCntPtr<clang::DiagnosticIDs> DiagID(
      new clang::DiagnosticIDs());
  clang::DiagnosticsEngine Diags(DiagID, DiagClient);

  // // Create the clang driver
  // clang::driver::Driver TheDriver(args[0],
  // llvm::sys::getDefaultTargetTriple(),
  //                                 outputPath, true, Diags);

  // // If you want to build C++ instead of C
  // //  TheDriver.CCCIsCXX = true;

  // // Create the set of actions to perform
  // clang::OwningPtr<clang::driver::Compilation> C(
  //     TheDriver.BuildCompilation(args));

  // // Print the set of actions
  // TheDriver.PrintActions(*C);

  // // Carry out the actions
  // int Res = 0;
  // const clang::driver::Command* FailingCommand = 0;
  // if (C) Res = TheDriver.ExecuteCompilation(*C, FailingCommand);

  // // Report problems
  // if (Res < 0) TheDriver.generateCompilationDiagnostics(*C, FailingCommand);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
