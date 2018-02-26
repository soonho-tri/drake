#include "drake/systems/rendering/dev/symbolic_render.h"

namespace drake {
namespace systems {
namespace rendering {

using symbolic::Expression;

namespace {

Matrix4<Expression> Scale(const Vector3<Expression>& scale_vec) {
  Matrix4<Expression> m;
  // clang-format off
  m << scale_vec[0],          0.0,          0.0, 0.0,
       0.0,          scale_vec[1],          0.0, 0.0,
       0.0,                   0.0, scale_vec[2], 0.0,
       0.0,                   0.0,          0.0, 1.0;
  // clang-format on
  return m;
}

Matrix4<Expression> Translate(const Vector3<Expression>& translate_vec) {
  // clang-format off
  Matrix4<Expression> m;
  m << 1.0, 0.0, 0.0, translate_vec[0],
       0.0, 1.0, 0.0, translate_vec[1],
       0.0, 0.0, 1.0, translate_vec[2],
       0.0, 0.0, 0.0,              1.0;
  // clang-format on
  return m;
}

Matrix4<Expression> Perspective(const Expression& fov, const Expression& z_near,
                                const Expression& z_far) {
  const Expression recip{1.0 / (z_far - z_near)};
  const Expression cot{1.0 / tan(fov / 2.0)};
  Matrix4<Expression> m;
  // clang-format off
  m << cot,   0.0,           0.0,             0.0,
         0.0, cot,           0.0,             0.0,
         0.0, 0.0, z_far * recip, -z_near * recip,
         0.0, 0.0,           1.0,             0.0;
  // clang-format on
  return m;
}

Vector3<Expression> XfmPoint(const Matrix4<Expression>& xform,
                             const Vector3<Expression>& pt) {
  const Vector4<Expression> tpt{xform(0, 0) * pt[0] + xform(0, 1) * pt[1] +
                                    xform(0, 2) * pt[2] + xform(0, 3),
                                xform(1, 0) * pt[0] + xform(1, 1) * pt[1] +
                                    xform(1, 2) * pt[2] + xform(1, 3),
                                xform(2, 0) * pt[0] + xform(2, 1) * pt[1] +
                                    xform(2, 2) * pt[2] + xform(2, 3),
                                xform(3, 0) * pt[0] + xform(3, 1) * pt[1] +
                                    xform(3, 2) * pt[2] + xform(3, 3)};
  const Expression invW{1.0 / tpt[3]};
  return Vector3<Expression>{tpt[0] * invW, tpt[1] * invW, tpt[2] * invW};
}

Vector3<Expression> XfmVector(const Matrix4<Expression>& xform,
                              const Vector3<Expression>& vec) {
  return Vector3<Expression>{
      xform(0, 0) * vec[0] + xform(0, 1) * vec[1] + xform(0, 2) * vec[2],
      xform(1, 0) * vec[0] + xform(1, 1) * vec[1] + xform(1, 2) * vec[2],
      xform(2, 0) * vec[0] + xform(2, 1) * vec[1] + xform(2, 2) * vec[2]};
}

}  // namespace

Vector3<Expression> Normalize(const Vector3<Expression>& vec) {
  return vec / vec.norm();
}

Expression RenderTriangle(const Matrix4<Expression>& cam_to_world,
                          const Vector3<Expression>& tri_v0,
                          const Vector3<Expression>& tri_v1,
                          const Vector3<Expression>& tri_v2,
                          const Expression& x, const Expression& y,
                          const Expression& aspect_ratio,
                          const Expression& z_near, const Expression& z_far,
                          const Expression& fov_y) {
  const Matrix4<Expression> cam_to_sample =
      Scale(Vector3<Expression>(-0.5, -0.5 * aspect_ratio, 1.0)) *
      Translate(Vector3<Expression>(-1.0, -1.0 / aspect_ratio, 0.0)) *
      Perspective(fov_y, z_near, z_far);
  const Matrix4<Expression> sample_to_cam{cam_to_sample.inverse()};
  const Vector3<Expression> tri_e1{tri_v1 - tri_v0};
  const Vector3<Expression> tri_e2{tri_v2 - tri_v0};

  // the origin of ray is the position of camera
  const Vector3<Expression> ray_org{
      XfmPoint(cam_to_world, Vector3<Expression>::Zero())};
  // for ray direction, transform (x, y, 0) to camera space and normalize
  Vector3<Expression> ray_dir{
      Normalize(XfmPoint(sample_to_cam, Vector3<Expression>(x, y, 0.0)))};
  // Vector3<Expression> ray_dir{
  //     XfmPoint(sample_to_cam, Vector3<Expression>(x, y, 0.0)).normalized()};
  // and transform to world space
  ray_dir = XfmVector(cam_to_world, ray_dir);
  // intersect the ray with objects (for now it's a single triangle)
  const Vector3<Expression> s1{ray_dir.cross(tri_e2)};
  const Expression divisor{s1.dot(tri_e1)};
  // TODO(tzumao): deal with the case divisor == 0 (ray is parallel with
  // triangle plane)
  const Expression inv_divisor{1.0 / divisor};
  const Vector3<Expression> s{ray_org - tri_v0};
  const Vector3<Expression> s2{s.cross(tri_e1)};
  // Barycentric coordinates
  const Expression b0{s.dot(s1) * inv_divisor};
  const Expression b1{ray_dir.dot(s2) * inv_divisor};
  // If hit, return 1, otherwise return
  const Expression mask{if_then_else(b0 >= 0.0 && b1 >= 0.0 && b0 + b1 <= 1.0,
                                     1.0, /* hit */
                                     0.0 /* miss */)};

  return mask;
}

sensors::ImageExpr RenderTriangle(
    const Matrix4<Expression>& cam_to_world, const Vector3<Expression>& tri_v0,
    const Vector3<Expression>& tri_v1, const Vector3<Expression>& tri_v2,
    const Expression& z_near, const Expression& z_far, const Expression& fov_y,
    const int img_width, const int img_height) {
  const Expression aspect_ratio{static_cast<double>(img_width) / img_height};
  sensors::ImageExpr img(img_width, img_height);
    for (int yi = 0; yi < img_height; ++yi) {
      for (int xi = 0; xi < img_width; ++xi) {
        const Expression x{(xi + 0.5) / img_width};
        const Expression y{(yi + 0.5) / img_height};
        *(img.at(xi, yi)) =
            RenderTriangle(cam_to_world, tri_v0, tri_v1, tri_v2, x, y,
                           aspect_ratio, z_near, z_far, fov_y);
    }
  }
  return img;
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake
