#pragma once

#include <Eigen/Core>
#include <vector>

class Tris {
private:
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  Eigen::Vector2d v3;

public:
  Tris();
  Tris(Eigen::Vector2d vv1, Eigen::Vector2d vv2, Eigen::Vector2d vv3);
  std::vector<Eigen::Vector2d> get_vertecies();
};
