#include <iostream>
#include "triangulation.h"



int main() {
  std::string path = "data/opendrive/Roundabout8Course.xodr";

  try {
    SimpleMesh mesh = convert_xodr(path);
    mesh.save_to_file("road", SimpleMesh::STL_ASCII);
  } catch (std::exception &ex) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
