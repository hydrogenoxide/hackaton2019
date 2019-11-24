#include "stl.h"
#include "string.h"
#include <fstream>
#include <iostream>
#include <vector>

std::string Stl::tris_to_ascii(Tris tris) {
  std::vector<Eigen::Vector2d> vertices = tris.get_vertecies();
  if (vertices.size() != 3) {
    return "";
  }
  Eigen::Vector2d v1 = vertices.at(0);
  Eigen::Vector2d v2 = vertices.at(1);
  Eigen::Vector2d v3 = vertices.at(2);

  std::ostringstream oss;
  oss << "facet normal 0 0 1" << std::endl;
  oss << "\touter loop" << std::endl;
  oss << "\t\tvertex " << (v1.x()) << " " << (v1.y()) << " "
      << "0" << std::endl;
  oss << "\t\tvertex " << (v2.x()) << " " << (v2.y()) << " "
      << "0" << std::endl;
  oss << "\t\tvertex " << (v3.x()) << " " << (v3.y()) << " "
      << "0" << std::endl;
  oss << "\tendloop" << std::endl;
  oss << "endfacet" << std::endl;

  return oss.str();
}

Stl::stl_facet Stl::tris_to_facet(Tris tris) {
  stl_facet facet;
  facet.normal[0] = (float)0.0;
  facet.normal[1] = (float)0.0;
  facet.normal[2] = (float)1.0;

  std::vector<Eigen::Vector2d> vertices = tris.get_vertecies();

  Eigen::Vector2d v1 = vertices.at(0);
  Eigen::Vector2d v2 = vertices.at(1);
  Eigen::Vector2d v3 = vertices.at(2);

  facet.v1[0] = (float)v1.x();
  facet.v1[1] = (float)v1.y();
  facet.v1[2] = (float)0.0;

  facet.v2[0] = (float)v2.x();
  facet.v2[1] = (float)v2.y();
  facet.v3[2] = (float)0.0;

  facet.v3[0] = (float)v3.x();
  facet.v3[1] = (float)v3.y();
  facet.v3[2] = (float)0.0;

  return facet;
}

Stl::Stl(std::vector<Tris> tris, std::string name) {
  facets = tris;
  name = name;
}

std::string Stl::to_ascii() {
  std::ostringstream oss;
  oss << "solid " << name << std::endl;
  for (Tris t : facets) {
    oss << tris_to_ascii(t);
  }
  oss << "endsolid " << name << std::endl;
  return oss.str();
}

void Stl::save_binary(std::string filename) {
  std::ofstream file;
  file.open(filename, std::ios::out | std::ios::binary);
  file.imbue(std::locale::classic());

  char header[80];
  std::fill(header, header + sizeof(header), 0);
  unsigned int num_facets = facets.size();
  file.write(header, sizeof(header));
  file.write((char *)&num_facets, sizeof(num_facets));

  for (Tris t : facets) {
    Stl::stl_facet f = tris_to_facet(t);

    file.write((char *)&f, sizeof(Stl::stl_facet));
  }
}
