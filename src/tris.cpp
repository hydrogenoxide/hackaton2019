#include "tris.h"

Tris::Tris() {
  v1 = Eigen::Vector2d(0.0, 0.0);
  v2 = Eigen::Vector2d(0.0, 0.0);
  v3 = Eigen::Vector2d(0.0, 0.0);
}

Tris::Tris(Eigen::Vector2d vv1, Eigen::Vector2d vv2, Eigen::Vector2d vv3) {
  v1 = vv1;
  v2 = vv2;
  v3 = vv3;
}

std::vector<Eigen::Vector2d> Tris::get_vertecies() {
  std::vector<Eigen::Vector2d> t;
  t.push_back(v1);
  t.push_back(v2);
  t.push_back(v3);
  return t;
}
