#include <iostream>
#include "xodr/xodr_map.h"



struct XodrFileInfo
{
  const char* name;
  const char* path;
};

static const XodrFileInfo xodrFiles[] = {
					 {"Crossing8Course", "data/opendrive/Crossing8Course.xodr"},
					 {"CulDeSac", "data/opendrive/CulDeSac.xodr"},
					 {"Roundabout8Course", "data/opendrive/Roundabout8Course.xodr"},
					 {"sample1.1", "data/opendrive/sample1.1.xodr",}
};

using namespace aid::xodr;

static bool showLaneType(LaneType laneType)
{
  return laneType == LaneType::DRIVING ||
    laneType == LaneType::SIDEWALK ||
    laneType == LaneType::BORDER;
}

std::string tris(double v1[], double v2[], double v3[]) {
  std::cout << "facet normal 0 0 1" << std::endl;
	    std::cout << "\touter loop" << std::endl;
	    std::cout << "\t\tvertex" << v1[0] << " " << v1[1] << " " << "0" << std::endl;
	    std::cout << "\t\tvertex" << v2[0] << " " << v2[1] << " " << "0" << std::endl;
	    std::cout << "\t\tvertex" << v3[0] << " " << v3[1] << " " << "0" << std::endl;
	    std::cout << "\t\tvertex" << std::endl;
	    std::cout << "\t\tvertex" << std::endl;
	    std::cout << "\tendloop" << std::endl;
	    std::cout << "end facet" << std::endl;
}

int get_poligon() {
  
}

void foo(XodrMap* xodrMap) {
  for (const Road& road : xodrMap->roads()) {
    for (LaneSection laneSection : road.laneSections()){
      for (LaneSection::Lane lane : laneSection.lanes()){
        for (LaneSection::WidthPoly3 wpoly : lane.widthPoly3s()) {

	    double step_size = (laneSection.endS() - laneSection.startS()) / 20;
	    

        }
      }
    }
  }

  void bar(XodrMap* xodrMap) {
    for (const Road& road : xodrMap->roads()) {
      ReferenceLine rline = road.referenceLine();
    
    }
  }

  int main()
  {
    const char* path = xodrFiles[1].path;

    std::cout << "Loading xodr file: " << path << std::endl;

    XodrParseResult<XodrMap> fromFileRes = XodrMap::fromFile(path);

    if (!fromFileRes.hasFatalErrors())
      {
	XodrMap* xodrMap(new XodrMap(std::move(fromFileRes.value())));
	bar(xodrMap);
      }
    else
      {
	std::cout << "Errors: " << std::endl;
	for (const auto& err : fromFileRes.errors())
	  {
	    std::cout << err.description() << std::endl;
	  }

	std::cerr << "Failed to load xodr file " << path << std::endl;
      }
  

    return 0;
  }
