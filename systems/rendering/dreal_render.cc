#include "drake/systems/rendering/symbolic_render.h"
#include "drake/solvers/dreal_solver.h"

#include <fstream>

namespace drake {
namespace systems {

using namespace sensors;
using namespace solvers;

inline Vector3<Expression> normalize(const Vector3<Expression> &vec) {
  return vec / vec.norm();
}

Matrix4<Expression> look_at(const Vector3<Expression> &pos,
                            const Vector3<Expression> &look,
                            const Vector3<Expression> &up) {
  Expression m[4][4];
  // Initialize fourth column of viewing matrix
  m[0][3] = pos[0];
  m[1][3] = pos[1];
  m[2][3] = pos[2];
  m[3][3] = Expression{1.0};

  // Initialize first three columns of viewing matrix
  Vector3<Expression> dir = normalize(look - pos);
  Vector3<Expression> left = normalize(normalize(up).cross(dir));
  Vector3<Expression> new_up = dir.cross(left);
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
  mat << m[0][0], m[0][1], m[0][2], m[0][3],
         m[1][0], m[1][1], m[1][2], m[1][3],
         m[2][0], m[2][1], m[2][2], m[2][3],
         m[3][0], m[3][1], m[3][2], m[3][3];
  return mat;
}

int main() {
  // Camera parameters
  Variable px{"px"}, py{"py"}, pz{"pz"};
  Variable lx{"lx"}, ly{"ly"}, lz{"lz"};
  Variable ux{"ux"}, uy{"uy"}, uz{"uz"};
  // Triangle parameters
  Variable v0_x{"v0_x"}, v0_y{"v0_y"}, v0_z{"v0_z"};
  Variable v1_x{"v1_x"}, v1_y{"v1_y"}, v1_z{"v1_z"};
  Variable v2_x{"v2_x"}, v2_y{"v2_y"}, v2_z{"v2_z"};
  // Screen space parameters
  Variable sx{"sx"}, sy{"sy"};
  Vector3<Expression> pos(px, py, pz);
  Vector3<Expression> look(lx, ly, lz);
  Vector3<Expression> up(ux, uy, uz);
  Vector3<Expression> tri_v0(v0_x, v0_y, v0_z);
  Vector3<Expression> tri_v1(v1_x, v1_y, v1_z);
  Vector3<Expression> tri_v2(v2_x, v2_y, v2_z);
  Matrix4<Expression> cam_to_world = look_at(pos, look, up);
  Expression x = sx;
  Expression y = sy;
  int img_width = 64;
  int img_height = 48;
  Expression aspect_ratio{double(img_width) / double(img_height)};

  Expression hit_triangle =
      render(cam_to_world, tri_v0, tri_v1, tri_v2, x, y, aspect_ratio);

  Environment env{{sx, 0.5}, {sy, 0.5},
                  {px, 0.0}, {py, 0.0}, {pz, -3.0},
                  {lx, 0.0}, {ly, 0.0}, {lz,  0.0},
                  {ux, 0.0}, {uy, 1.0}, {uz,  0.0}};
  // Fill in constant values
  Expression hit_triangle_folded = hit_triangle.EvaluatePartial(env);
  double interval = 0.1;
  Formula f0x{-1.0 - interval <= v0_x && v0_x <= -1.0 + interval};
  Formula f0y{-1.0 - interval <= v0_y && v0_y <= -1.0 + interval};
  Formula f0z{ 0.0 - interval <= v0_z && v0_z <=  0.0 + interval};

  Formula f1x{ 1.0 - interval <= v1_x && v1_x <=  1.0 + interval};
  Formula f1y{-1.0 - interval <= v1_y && v1_y <= -1.0 + interval};
  Formula f1z{ 0.0 - interval <= v1_z && v0_z <=  0.0 + interval};

  Formula f2x{ 0.0 - interval <= v2_x && v2_x <=  0.0 + interval};
  Formula f2y{ 1.0 - interval <= v2_y && v2_y <=  1.0 + interval};
  Formula f2z{ 0.0 - interval <= v2_z && v2_z <=  0.0 + interval};

  Formula no_hit{hit_triangle_folded == 0.0};

  auto result = DrealSolver::CheckSatisfiability(
    f0x && f0y && f0z &&
    f1x && f1y && f1z &&
    f2x && f2y && f2z && no_hit, 1e-3);

  if (result) {
    // We find an instance of triangle vertices where the ray will miss
    std::cout << "SAT" << std::endl;
    const DrealSolver::IntervalBox& solution{*result};
    DrealSolver::Interval v0xi = solution.at(v0_x);
    DrealSolver::Interval v0yi = solution.at(v0_y);
    DrealSolver::Interval v0zi = solution.at(v0_z);
    DrealSolver::Interval v1xi = solution.at(v1_x);
    DrealSolver::Interval v1yi = solution.at(v1_y);
    DrealSolver::Interval v1zi = solution.at(v1_z);
    DrealSolver::Interval v2xi = solution.at(v2_x);
    DrealSolver::Interval v2yi = solution.at(v2_y);
    DrealSolver::Interval v2zi = solution.at(v2_z);

    std::cout << "v0x.mid(): " << v0xi.mid() << ", v0x.diam():" << v0xi.diam() << std::endl;
    std::cout << "v0y.mid(): " << v0yi.mid() << ", v0y.diam():" << v0yi.diam() << std::endl;
    std::cout << "v0z.mid(): " << v0zi.mid() << ", v0z.diam():" << v0zi.diam() << std::endl;
    std::cout << "v1x.mid(): " << v1xi.mid() << ", v1x.diam():" << v1xi.diam() << std::endl;
    std::cout << "v1y.mid(): " << v1yi.mid() << ", v1y.diam():" << v1yi.diam() << std::endl;
    std::cout << "v1z.mid(): " << v1zi.mid() << ", v1z.diam():" << v1zi.diam() << std::endl;
    std::cout << "v2x.mid(): " << v2xi.mid() << ", v2x.diam():" << v2xi.diam() << std::endl;
    std::cout << "v2y.mid(): " << v2yi.mid() << ", v2y.diam():" << v2yi.diam() << std::endl;
    std::cout << "v2z.mid(): " << v2zi.mid() << ", v2z.diam():" << v2zi.diam() << std::endl;
  } else {
    std::cout << "UNSAT" << std::endl;
  }

  return 0;
}

}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::systems::main();
}
