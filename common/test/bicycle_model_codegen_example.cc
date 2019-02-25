#include <iostream>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <inja/inja.hpp>

#include "drake/common/symbolic.h"
#include "drake/common/symbolic_codegen_util.h"

namespace drake {
namespace symbolic {
namespace {

using std::cerr;
using std::endl;
using std::string;

class SymbolicCodeGenTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
};

TEST_F(SymbolicCodeGenTest, InjaTest) {
  using json = nlohmann::json;

  inja::Environment env;
  const string template_filename{"common/test/codegen.h.template"};
  inja::Template templ{env.parse_template(template_filename)};

  OrderedDocumentedSymbolDict parameters{
      "Parameters",
      "Vector of vehicle parameters",
      {{"l_f", "m", "Distance from mass center to front contact "},
       {" l_r ", "m", " Distance from mass center to read contact"},
       {" cp_x ", "m", "Distance in x direction from mass center to control"},
       {"cp_y", "m", "Distance in y direction from mass center to control"},
       {"m", "kg", "Total mass of vehicle"},
       {"I_zz", "kg*m^2", "Central moment of inertia about z axis"},
       {"g", "m*s^-2", "Gravitational acceleration"},
       {"C_alpha_f", "N*rad^-1", "Front tire lateral slip coefficient"},
       {"mu_f", "", "Front tire friction coefficient"},
       {"C_alpha_r", "N*rad^-1", "Rear tire lateral slip coefficient"},
       {"mu_r", "", "Rear tire friction coefficient"}}};

  OrderedDocumentedSymbolDict states{
      "States",
      "Vector of vehicle states",
      {{"x_mc", "m",
        "Distance in x direction from inertial origin to mass center"},
       {"y_mc", "m",
        "Distance in y direction from inertial origin to mass center"},
       {"psi", "rad", "Heading measured from +x axis"},
       {"v_x_mc", "m*s^-1",
        "Inertial velocity of mass center in body fixed x direction"},
       {"v_y_mc", "m*s^-1",
        "Inertial velocity of mass center in body fixed y direction"},
       {"r", "rad^-1", "Heading rate"}}};

  OrderedDocumentedSymbolDict inputs{
      "Inputs",
      "Vector of vehicle inputs",
      {{"delta", "rad", "Front tire angle"},
       {"F_total", "kg*m*s^-2", "Total force acting on mass center"}}};

  OrderedDocumentedSymbolDict outputs{
      "Outputs",
      "Vehicle model output vector",
      {{"psi", "rad", "Heading measured from x axis"},
       {"r", "rad^-1", "Heading rate"},
       {"x_f", "m",
        "Distance in x direction from inertial origin to front axle"},
       {"y_f", "m",
        "Distance in y direction from inertial origin to front axle"},
       {"v_x_f", "m*s^-1",
        "Inertial velocity of front axle in body fixed x direction"},
       {"v_y_f", "m*s^-1",
        "Inertial velocity of front axle in body fixed y direction"},
       {"a_x_f", "m*s^-2",
        "Inertial acceleration of front axle in body fixed x direction"},
       {"a_y_f", "m*s^-2",
        "Inertial acceleration of front axle in body fixed y direction"},
       {"alpha_f", "rad", "Side slip angle of front axle"},
       {"x_mc", "m",
        "Distance in x direction from inertial origin to mass center"},
       {"y_mc", "m",
        "Distance in y direction from inertial origin to mass center"},
       {"v_x_mc", "m*s^-1",
        "Inertial velocity of mass center in body fixed x direction"},
       {"v_y_mc", "m*s^-1",
        "Inertial velocity of mass center in body fixed y direction"},
       {"a_x_mc", "m*s^-2",
        "Inertial acceleration of mass center in body fixed x direction"},
       {"a_y_mc", "m*s^-2",
        "Inertial acceleration of mass center in body fixed y direction"},
       {"alpha_mc", "rad", "Side slip angle of mass center"},
       {"x_r", "m",
        "Distance in x direction from inertial origin to rear axle"},
       {"y_r", "m",
        "Distance in y direction from inertial origin to rear axle"},
       {"v_x_r", "m*s^-1",
        "Inertial velocity of rear axle in body fixed x direction"},
       {"v_y_r", "m*s^-1",
        "Inertial velocity of rear axle in body fixed y direction"},
       {"a_x_r", "m*s^-2",
        "Inertial acceleration of rear axle in body fixed x direction"},
       {"a_y_r", "m*s^-2",
        "Inertial acceleration of rear axle in body fixed y direction"},
       {"alpha_r", "rad", "Side slip angle of rear axle"},
       {"x_cp", "m",
        "Distance in x direction from inertial origin to control point"},
       {"y_cp", "m",
        "Distance in y direction from inertial origin to control point"},
       {"v_x_cp", "m*s^-1",
        "Inertial velocity of control point in body fixed x direction"},
       {"v_y_cp", "m*s^-1",
        "Inertial velocity of control point in body fixed y direction"},
       {"a_x_cp", "m*s^-2",
        "Inertial acceleration of control point in body fixed x direction"},
       {"a_y_cp", "m*s^-2",
        "Inertial acceleration of control point in body fixed y direction"}}};

  constexpr int num_coordinates = 3;

  json data;
  data["namespace"] = {"drake", "test"};
  data["real_type"] = "double";
  data["vectors"] = {parameters, states, inputs, outputs};

  cerr << data.dump(4) << endl;

  const string output{env.render(templ, data)};
  cerr << output << endl;
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
