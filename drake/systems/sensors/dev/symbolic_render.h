#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

drake::symbolic::Expression render(
    const Matrix4<drake::symbolic::Expression>& cam_to_world,
    const Vector3<drake::symbolic::Expression>& tri_v0,
    const Vector3<drake::symbolic::Expression>& tri_v1,
    const Vector3<drake::symbolic::Expression>& tri_v2,
    const drake::symbolic::Expression& x, const drake::symbolic::Expression& y,
    const drake::symbolic::Expression& aspect_ratio,
    const drake::symbolic::Expression& z_near = 0.5,
    const drake::symbolic::Expression& z_far = 5.0,
    const drake::symbolic::Expression& fov_y = M_PI_4);

ImageExpr render(const Matrix4<drake::symbolic::Expression>& cam_to_world,
                 const Vector3<drake::symbolic::Expression>& tri_v0,
                 const Vector3<drake::symbolic::Expression>& tri_v1,
                 const Vector3<drake::symbolic::Expression>& tri_v2,
                 const drake::symbolic::Expression& z_near,
                 const drake::symbolic::Expression& z_far,
                 const drake::symbolic::Expression& fov_y, int img_width,
                 int img_height);

}  // namespace sensors
}  // namespace systems
}  // namespace drake
