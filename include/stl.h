#pragma once

#include "tris.h"
#include <fstream>
#include <sstream>

class Stl {
private:
  struct stl_facet {
    float normal[3];
    float v1[3];
    float v2[3];
    float v3[3];
    unsigned int abc[2] = {0};
  };

  std::string name;
  std::vector<Tris> facets;

  std::string tris_to_ascii(Tris tris);
  stl_facet tris_to_facet(Tris tris);

public:
  Stl(std::vector<Tris> tris, std::string name);
  std::string to_ascii();
  void save_binary(std::string filename);
};
