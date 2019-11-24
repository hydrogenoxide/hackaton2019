#include "triangulation.h"
#include <iostream>

int main(int argc, char *argv[]) {

  if (argc < 2) {
    std::cout << "No file to convert provided." << std::endl;
    std::cout << "Usage:" << std::endl;
    std::cout << argv[0] << " <OpenROAD file>" << std::endl;
    return EXIT_FAILURE;
  }

  // std::string path = "data/opendrive/sample1.1.xodr";
  char *p = argv[1];
  std::string path = p;

  try {
    SimpleMesh mesh = convert_xodr(path);
    mesh.save_to_file("road", SimpleMesh::STL_ASCII);
  } catch (std::exception &ex) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
