#include <fstream>

#include "drake/systems/sensors/dev/symbolic_render.h"

namespace drake {
namespace systems {
namespace sensors {

using std::fstream;

using symbolic::Expression;
using symbolic::Variable;
using symbolic::Environment;

inline Vector3<Expression> normalize(const Vector3<Expression>& vec) {
  return vec / vec.norm();
}

Matrix4<Expression> look_at(const Vector3<Expression>& pos,
                            const Vector3<Expression>& look,
                            const Vector3<Expression>& up) {
  Expression m[4][4];
  // Initialize fourth column of viewing matrix
  m[0][3] = pos[0];
  m[1][3] = pos[1];
  m[2][3] = pos[2];
  m[3][3] = Expression{1.0};

  // Initialize first three columns of viewing matrix
  const Vector3<Expression> dir = normalize(look - pos);
  const Vector3<Expression> left = normalize(normalize(up).cross(dir));
  const Vector3<Expression> new_up = dir.cross(left);
  m[0][0] = left[0];
  m[1][0] = left[1];
  m[2][0] = left[2];
  m[3][0] = Expression{0.0};
  m[0][1] = new_up[0];
  m[1][1] = new_up[1];
  m[2][1] = new_up[2];
  m[3][1] = Expression{0.0};
  m[0][2] = dir[0];
  m[1][2] = dir[1];
  m[2][2] = dir[2];
  m[3][2] = Expression{0.0};

  Matrix4<Expression> mat;
  mat << m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3],
      m[2][0], m[2][1], m[2][2], m[2][3], m[3][0], m[3][1], m[3][2], m[3][3];
  return mat;
}

int main() {
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
  const Vector3<Expression> pos(px, py, pz);
  const Vector3<Expression> look(lx, ly, lz);
  const Vector3<Expression> up(ux, uy, uz);
  const Vector3<Expression> tri_v0(Expression{-1.0}, Expression{-1.0},
                                   Expression{0.0});
  const Vector3<Expression> tri_v1(Expression{1.0}, Expression{-1.0},
                                   Expression{0.0});
  const Vector3<Expression> tri_v2(Expression{0.0}, Expression{1.0},
                                   Expression{0.0});
  const Matrix4<Expression> cam_to_world = look_at(pos, look, up);
  const Expression x = sx;
  const Expression y = sy;
  const int img_width = 64;
  const int img_height = 48;
  const Expression aspect_ratio{static_cast<double>(img_width) /
                                static_cast<double>(img_height)};

  Environment env{{px, 0.0},   {py, 0.0},    {pz, -3.0},   {lx, 0.0},
                  {ly, 0.0},   {lz, 0.0},    {ux, 0.0},    {uy, 1.0},
                  {uz, 0.0},   {v0_x, -1.0}, {v0_y, -1.0}, {v0_z, 0.0},
                  {v1_x, 1.0}, {v1_y, -1.0}, {v1_z, 0.0},  {v2_x, 0.0},
                  {v2_y, 1.0}, {v2_z, 0.0}};

  const Expression hit_triangle =
      render(cam_to_world, tri_v0, tri_v1, tri_v2, x, y, aspect_ratio);

  ImageGrey8U img(img_width, img_height);
  for (int yi = 0; yi < img_height; yi++) {
    for (int xi = 0; xi < img_width; xi++) {
      env[sx] = (xi + 0.5) / img_width;
      env[sy] = (yi + 0.5) / img_height;
      const double val = hit_triangle.Evaluate(env);
      *img.at(xi, yi) = uint8_t(std::min(val, 1.0) * 255.0);
    }
  }

  fstream fs("./image.ppm", std::fstream::out);
  fs << "P3\n" << img_width << " " << img_height << "\n" << 255 << "\n";
  for (int yi = 0; yi < img_height; ++yi) {
    for (int xi = 0; xi < img_width; ++xi) {
      int val = static_cast<int>(*img.at(xi, yi));
      fs << val << " " << val << " " << val << " ";
    }
  }
  fs.close();
  return 0;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) { return drake::systems::sensors::main(); }
