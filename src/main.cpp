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

void tris(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d v3) {
  std::cout << "facet normal 0 0 1" << std::endl;
  std::cout << "\touter loop" << std::endl;
  std::cout << "\t\tvertex" << v1.x() << " " << v1.y() << " " << "0" << std::endl;
  std::cout << "\t\tvertex" << v2.x() << " " << v2.y() << " " << "0" << std::endl;
  std::cout << "\t\tvertex" << v3.x() << " " << v3.y() << " " << "0" << std::endl;
  std::cout << "\t\tvertex" << std::endl;
  std::cout << "\t\tvertex" << std::endl;
  std::cout << "\tendloop" << std::endl;
  std::cout << "end facet" << std::endl;
}

int get_poligon(double s, std::vector<LaneSection::WidthPoly3>* polygons) {
  double commulative_offset = 0;

  for (int i = 0; i < polygons->size(); i++) {
    commulative_offset += polygons->at(i).sOffset();
    if (commulative_offset >= s) {
      return i;
    }
  }
  return -1;
}

void convert_stl(XodrMap* xodrMap) {

  size_t number_steps = 20;

  
  for (const Road& road : xodrMap->roads()) {
    for (LaneSection laneSection : road.laneSections()){
      
      double step_size = (laneSection.endS() - laneSection.startS()) / number_steps;
      double start = laneSection.startS();
      
      for (LaneSection::Lane lane : laneSection.lanes()){
	// woosh
	if (lane.id().operator int() != 1) {
	  continue;
	}

	std::vector<LaneSection::WidthPoly3> polygons = lane.widthPoly3s();

	double s = start;
	int poly_idx = get_poligon(s, &polygons);
	if (poly_idx < 0) {
	  std::cerr << "Something went horribly wrong at the beginning :(" << std::endl;
	  exit(-1);
	}
	LaneSection::WidthPoly3 poly = polygons.at(poly_idx);
	double width = poly.poly3().eval(s - poly.sOffset());

	// only true for lane with id 1
	double t1 = 0;
	double t2 = width;

	// that's wrong
	ReferenceLine::PointAndTangentDir ptd = road.referenceLine().eval(s);

	Eigen::Vector2d v1 = ptd.pointWithTCoord(t1);
	Eigen::Vector2d v2 = ptd.pointWithTCoord(t2);

        for (int i = 1; i < number_steps; i++) {
          s = start + i * step_size;
          poly_idx = get_poligon(s, &polygons);
	  if (poly_idx < 0) {
	    std::cerr << "Something went horribly wrong :(" << std::endl;
	    exit(-1);
	  }
          poly = polygons.at(poly_idx);
          width = poly.poly3().eval(s - poly.sOffset());

          t1 = 0;
          t2 = width;

          Eigen::Vector2d v3 = ptd.pointWithTCoord(t1);
          Eigen::Vector2d v4 = ptd.pointWithTCoord(t2);

          // pay attention to order because of orientation!
          tris(v3, v2, v1);
          tris(v2, v3, v4);

          v1 = v3;
          v2 = v4;
        }
      }
    }
  }
}

int main() {
  const char* path = xodrFiles[1].path;

  std::cout << "Loading xodr file: " << path << std::endl;

  XodrParseResult<XodrMap> fromFileRes = XodrMap::fromFile(path);

  if (!fromFileRes.hasFatalErrors())
    {
      XodrMap* xodrMap(new XodrMap(std::move(fromFileRes.value())));
      std::cout << "solid road" << std::endl;
      convert_stl(xodrMap);
      std::cout << "endsolid road" << std::endl;
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
