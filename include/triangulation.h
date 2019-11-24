#pragma once

#include "tris.h"
#include "xodr/xodr_map.h"
#include <vector>

using namespace aid::xodr;

struct XodrFileInfo {
  const char *name;
  const char *path;
};

class SimpleMesh {
private:
  std::vector<Tris> surface;

public:
  enum FileFormat {
    STL_ASCII,
    STL_BINARY,
  };

  SimpleMesh(std::vector<Tris> mesh);
  size_t save_to_file(std::string filename, FileFormat format);
};

SimpleMesh convert_xodr(XodrMap *xodrMap);
SimpleMesh convert_xodr(std::string path);
