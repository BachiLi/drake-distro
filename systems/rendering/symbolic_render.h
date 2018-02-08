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

using namespace drake::symbolic;

namespace drake {
namespace systems {
namespace sensors {

Expression render(const Matrix4<Expression>& cam_to_world,
                  const Vector3<Expression>& tri_v0,
                  const Vector3<Expression>& tri_v1,
                  const Vector3<Expression>& tri_v2,
                  Expression x,
                  Expression y,
                  Expression aspect_ratio,
                  Expression z_near = Expression{0.5},
                  Expression z_far = Expression{5.0},
                  Expression fov_y = Expression{M_PI_4});

ImageExpr render(const Matrix4<Expression>& cam_to_world,
                 const Vector3<Expression>& tri_v0,
                 const Vector3<Expression>& tri_v1,
                 const Vector3<Expression>& tri_v2,
                 Expression z_near,
                 Expression z_far,
                 Expression fov_y,
                 int img_width,
                 int img_height);

}  // namespace sensors
}  // namespace systems
}  // namespace drake