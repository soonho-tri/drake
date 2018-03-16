#include <fstream>

#include <Eigen/Core>

#include "drake/common/eigen_types.h"
#include "drake/solvers/dreal_solver.h"
#include "drake/systems/rendering/dev/symbolic_render.h"

namespace drake {
namespace systems {
namespace rendering {
namespace {

using solvers::DrealSolver;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Formula;
using symbolic::Substitution;
using symbolic::Variable;

Matrix4<Expression> LookAt(const Vector3<Expression>& pos,
                           const Vector3<Expression>& look,
                           const Vector3<Expression>& up) {
  Matrix4<Expression> mat;
  // Initialize first three columns of viewing matrix
  const Vector3<Expression> dir = Normalize(look - pos);
  const Vector3<Expression> left = Normalize(Normalize(up).cross(dir));
  const Vector3<Expression> new_up = dir.cross(left);
  mat.block<3, 1>(0, 0) = left;
  mat.block<3, 1>(0, 1) = new_up;
  mat.block<3, 1>(0, 2) = dir;
  // Initialize fourth column of viewing matrix
  mat.block<3, 1>(0, 3) = pos;
  mat(3, 3) = 1.0;
  return mat;
}

void PrintImage(const Formula& f, const Variable& sx, const Variable& sy,
                const int img_width, const int img_height) {
  Environment env;
  sensors::ImageGrey8U img(img_width, img_height);
  for (int yi = 0; yi < img_height; ++yi) {
    for (int xi = 0; xi < img_width; ++xi) {
      env[sx] = (xi + 0.5) / img_width;
      env[sy] = (yi + 0.5) / img_height;
      double val = f.Evaluate(env);
      *img.at(xi, yi) = uint8_t(std::min(val, 1.0) * 255.0);
    }
  }
  std::fstream fs("./image.ppm", std::fstream::out);
  fs << "P3\n" << img_width << " " << img_height << "\n" << 255 << "\n";
  for (int yi = 0; yi < img_height; ++yi) {
    for (int xi = 0; xi < img_width; ++xi) {
      int val = static_cast<int>(*img.at(xi, yi));
      fs << val << " " << val << " " << val << " ";
    }
  }
  fs.close();
}

int Main() {
  // Camera parameters
  const Variable px{"px"}, py{"py"}, pz{"pz"};
  const Variable lx{"lx"}, ly{"ly"}, lz{"lz"};
  const Variable ux{"ux"}, uy{"uy"}, uz{"uz"};
  // Triangle parameters
  const Variable v0_x{"v0_x"}, v0_y{"v0_y"}, v0_z{"v0_z"};
  const Variable v1_x{"v1_x"}, v1_y{"v1_y"}, v1_z{"v1_z"};
  const Variable v2_x{"v2_x"}, v2_y{"v2_y"}, v2_z{"v2_z"};
  // Screen space parameters
  const Variable sx{"sx"}, sy{"sy"};
  Vector3<Expression> pos(px, py, pz);
  Vector3<Expression> look(lx, ly, lz);
  Vector3<Expression> up(ux, uy, uz);
  Vector3<Expression> tri_v0(v0_x, v0_y, v0_z);
  Vector3<Expression> tri_v1(v1_x, v1_y, v1_z);
  Vector3<Expression> tri_v2(v2_x, v2_y, v2_z);
  Matrix4<Expression> cam_to_world = LookAt(pos, look, up);
  Expression x = sx;
  Expression y = sy;
  int img_width = 64;
  int img_height = 48;
  Expression aspect_ratio{static_cast<double>(img_width) /
                          static_cast<double>(img_height)};

  Expression hit_triangle =
      RenderTriangle(cam_to_world, tri_v0, tri_v1, tri_v2, x, y, aspect_ratio);

  // clang-format off
  Environment env1{{px, 0.0},
                   {py, 0.0},
                   {pz, -3.0},
                   {lx, 0.0},
                   {ly, 0.0},
                   {lz, 0.0},
                   {ux, 0.0},
                   {uy, 1.0},
                   {uz, 0.0}};
  // clang-format off
  Environment env2{{sx, 0.5}, {sy, 0.5}};

  // Fill in constant values
  Expression hit_triangle_x_y = hit_triangle.EvaluatePartial(env1);
  Expression hit_triangle_center = hit_triangle_x_y.EvaluatePartial(env2);
  const double interval = 0.5;

  // clang-format off
  Formula f0x{-1.0 - interval <= v0_x && v0_x <= -1.0 + interval};
  Formula f0y{-1.0 - interval <= v0_y && v0_y <= -1.0 + interval};
  Formula f0z{ 0.0 - interval <= v0_z && v0_z <=  0.0 + interval};

  Formula f1x{ 1.0 - interval <= v1_x && v1_x <=  1.0 + interval};
  Formula f1y{-1.0 - interval <= v1_y && v1_y <= -1.0 + interval};
  Formula f1z{ 0.0 - interval <= v1_z && v1_z <=  0.0 + interval};

  Formula f2x{ 0.0 - interval <= v2_x && v2_x <=  0.0 + interval};
  Formula f2y{ 1.0 - interval <= v2_y && v2_y <=  1.0 + interval};
  Formula f2z{ 0.0 - interval <= v2_z && v2_z <=  0.0 + interval};
  // clang-format on

  Formula hit{hit_triangle_center == 1.0};
  // In the following we explicitly check if the lhs is equal to 0.0 instead of
  // negating `hit` to avoid any numerical issues (`≠ 1.0` vs `= 0.0`).
  Formula no_hit{hit_triangle_center == 0.0};

  std::cerr << no_hit << std::endl;

  auto result = DrealSolver::CheckSatisfiability(
      f0x && f0y && f0z && f1x && f1y && f1z && f2x && f2y && f2z && no_hit,
      1e-3);

  if (result) {
    // We find an instance of triangle vertices where the ray will miss
    std::cout << "SAT" << std::endl;
    Substitution subst;
    const DrealSolver::IntervalBox& solution{*result};
    for (const auto& p : solution) {
      std::cerr << p.first << " = " << p.second << std::endl;
      subst.emplace(p.first, p.second.mid());
    }
    std::cerr << hit.Substitute(subst) << std::endl << std::endl;
    std::cerr << hit_triangle_x_y.Substitute(subst) << std::endl;
    PrintImage(hit_triangle_x_y.Substitute(subst) == 1.0, sx, sy, 64, 48);
  } else {
    std::cout << "UNSAT" << std::endl;
  }
  return 0;
}

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) { return drake::systems::rendering::Main(); }
