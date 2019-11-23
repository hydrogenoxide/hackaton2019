#include <iostream>
#include <sstream>
#include <fstream>
#include "string.h"
#include "xodr/xodr_map.h"
#include "visible_lanes.h"
#include "stl.h"
#include "tris.h"


struct XodrFileInfo
{
  const char* name;
  const char* path;
};

static const XodrFileInfo xodrFiles[] = {
					 {"Crossing8Course", "data/opendrive/Crossing8Course.xodr"},
					 {"Roundabout8Course", "data/opendrive/Roundabout8Course.xodr"},
					 {"sample1.1", "data/opendrive/sample1.1.xodr",}
};

using namespace aid::xodr;

int get_poligon(double s_section, std::vector<LaneSection::WidthPoly3>* polygons) {
  double commulative_offset = 0;

  for (int i = 0; i < polygons->size(); i++) {
    commulative_offset += polygons->at(i).sOffset();

    if (s_section < commulative_offset) {
      return i - 1;
    }
  }
  return polygons->size() - 1;
}

std::vector<Eigen::Vector2d> get_one_vertex_row(LaneSection* laneSection,
						ReferenceLine* ref_line,
						double s, double start){
  std::vector<Eigen::Vector2d> vertex_row(visible_lanes(laneSection).size() + 1);

  // point and tangent direction at the current s we are looking at
  ReferenceLine::PointAndTangentDir ptd = ref_line->eval(s);

  int offset = num_visible_lanes_right(laneSection);

  vertex_row.at(offset) = ptd.pointWithTCoord(0);

  double accumulated_width = 0;
  // iterate over lanes with negative id
  for (int i = 1; i <= num_visible_lanes_right(laneSection); i++) {
    LaneID lid = LaneID(-i);
    LaneSection::Lane lane = laneSection->laneById(lid);
    std::vector<LaneSection::WidthPoly3> polygons = lane.widthPoly3s();

    int poly_idx = get_poligon(s - start, &polygons);
    LaneSection::WidthPoly3 poly = polygons.at(poly_idx);
    double width = poly.poly3().eval(s - poly.sOffset());
    accumulated_width -= width;
    vertex_row.at(offset - i) = ptd.pointWithTCoord(accumulated_width);
  }

  accumulated_width = 0;
  // iterate over lanes with positive id
  for (int i = 1; i <= num_visible_lanes_left(laneSection); i++) {
    LaneID lid = LaneID(i);
    LaneSection::Lane lane = laneSection->laneById(lid);
    std::vector<LaneSection::WidthPoly3> polygons = lane.widthPoly3s();

    int poly_idx = get_poligon(s - start, &polygons);
    LaneSection::WidthPoly3 poly = polygons.at(poly_idx);
    double width = poly.poly3().eval(s - poly.sOffset());
    accumulated_width += width;
    vertex_row.at(offset + i) = ptd.pointWithTCoord(accumulated_width);
  }
  return vertex_row;
}

std::vector<Tris> create_mesh(std::vector<Eigen::Vector2d>* vertices_behind,
					    std::vector<Eigen::Vector2d>* vertices_next){
  std::vector<Tris> mesh(2 * (vertices_next->size() - 1));

  for (int i = 0; i < vertices_next->size() - 1; ++i) {
    Tris tris1(vertices_next->at(i), vertices_behind->at(i), vertices_behind->at(i + 1));
    Tris tris2(vertices_next->at(i), vertices_behind->at(i + 1), vertices_next->at(i + 1));
    mesh.push_back(tris1);
    mesh.push_back(tris2);
  }
  return mesh;
}

std::vector<Tris> convert_stl(XodrMap* xodrMap) {

  size_t number_steps = 100;

  std::vector<Tris> mesh;

  for (const Road& road : xodrMap->roads()) {


    ReferenceLine ref_line = road.referenceLine();

    for (LaneSection laneSection : road.laneSections()){

      double step_size = (laneSection.endS() - laneSection.startS()) / number_steps;
      double start = laneSection.startS();
      double s = start;


      std::vector<Eigen::Vector2d> vertices_behind =
	get_one_vertex_row(&laneSection, &ref_line, s, start);



      for (int i = 1; i <= number_steps; i++) {
	s += step_size;
	if (s > laneSection.endS()) {
	  // The variable s becomes to big because of inaccuracies of floating point.
	  // Set it to the end to fix the bug :)
	  s = laneSection.endS();
	}
	std::vector<Eigen::Vector2d> vertices_next =
	  get_one_vertex_row(&laneSection, &ref_line, s, start);

	std::vector<Tris> new_tris = create_mesh(&vertices_behind, &vertices_next);
	mesh.reserve(mesh.size() + new_tris.size());
	mesh.insert(mesh.end(), new_tris.begin(), new_tris.end());
	vertices_behind = vertices_next;

      }
    }
  }
  return mesh;
}

int main() {
  const char* path = xodrFiles[0].path;

  // std::cout << "Loading xodr file: " << path << std::endl;

  XodrParseResult<XodrMap> fromFileRes = XodrMap::fromFile(path);

  if (!fromFileRes.hasFatalErrors())
    {
      XodrMap* xodrMap(new XodrMap(std::move(fromFileRes.value())));

      std::vector<Tris> mesh = convert_stl(xodrMap);

      Stl stl(mesh, "road");
      stl.save_binary("stl_export.stl");
      std::cout << stl.to_ascii() << std::endl;
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
