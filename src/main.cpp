#include "triangulation.h"
#include <iostream>

int main() {
  std::string path = "data/opendrive/sample1.1.xodr";

  try {
    SimpleMesh mesh = convert_xodr(path);
    mesh.save_to_file("road", SimpleMesh::STL_ASCII);
  } catch (std::exception &ex) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
