#include <iostream>
#include "xodr/xodr_map.h"


class Stl_bin_file{
  struct stl_facet {
    float normal[3];
    float v1[3];
    float v2[3];
    float v3[3];
    unsigned int abc[2] = {0};
  };

private:
  std::vector<stl_facet> facets;
  unsigned int num_facets;

  Stl_bin_file(std::vector<Eigen::Vector2d[3]>) {
    // TODO: implement
  }
};

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

void tris(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d v3) {
  std::cout << "facet normal 0 0 1" << std::endl;
  std::cout << "\touter loop" << std::endl;
  std::cout << "\t\tvertex " << (v1.x()) << " " << (v1.y()) << " " << "0" << std::endl;
  std::cout << "\t\tvertex " << (v2.x()) << " " << (v2.y()) << " " << "0" << std::endl;
  std::cout << "\t\tvertex " << (v3.x()) << " " << (v3.y()) << " " << "0" << std::endl;
  std::cout << "\tendloop" << std::endl;
  std::cout << "endfacet" << std::endl;
}

int get_poligon(double s_section, std::vector<LaneSection::WidthPoly3>* polygons) {
  double commulative_offset = 0;

  // std::cout << "get_poligon" << std::endl;
  // std::cout << "s " << s << std::endl;

  for (int i = 0; i < polygons->size(); i++) {
    commulative_offset += polygons->at(i).sOffset();
    // std::cout << "Round " << i << std::endl;
    // std::cout << "com offset " << commulative_offset << std::endl;
    // std::cout << "current poly offset " << polygons->at(i).sOffset() << std::endl;
    if (s_section < commulative_offset) {
      // std::cout << "return index  " << i << std::endl;
      return i - 1;
    }
  }
  return polygons->size() - 1;
}

std::vector<Eigen::Vector2d> get_one_vertex_row(LaneSection* laneSection,
						ReferenceLine* ref_line,
						double s, double start){
  std::vector<Eigen::Vector2d> vertex_row;
  vertex_row.reserve(laneSection->lanes().size());

  // point and tangent direction at the current s we are looking at
  ReferenceLine::PointAndTangentDir ptd = ref_line->eval(s);

  int offset = laneSection->numRightLanes();

  vertex_row.at(offset) = ptd.pointWithTCoord(0);

  double accumulated_width = 0;
  // iterate over lanes with negative id
  for (int i = 1; i <= laneSection->numRightLanes(); i++) {
    LaneID lid = LaneID(-i);
    LaneSection::Lane lane = laneSection->laneById(lid);
    std::vector<LaneSection::WidthPoly3> polygons = lane.widthPoly3s();

    int poly_idx = get_poligon(s - start, &polygons);
    LaneSection::WidthPoly3 poly = polygons.at(poly_idx);
    double width = poly.poly3().eval(s - poly.sOffset());
    accumulated_width += width;
    vertex_row.at(offset - i) = ptd.pointWithTCoord(accumulated_width);
  }

  accumulated_width = 0;
  // iterate over lanes with positive id
  for (int i = 1; i <= laneSection->numLeftLanes(); i++) {
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

std::vector<Eigen::Vector2d[3]> create_mesh(std::vector<Eigen::Vector2d>* vertices_behind,
					    std::vector<Eigen::Vector2d>* vertices_next){
  std::vector<Eigen::Vector2d[3]> mesh;
  mesh.reserve(2 * (vertices_next->size() - 1));

  for (int i = 0; i < vertices_next->size(); ++i) {
    Eigen::Vector2d tris1[3] = {vertices_next->at(i), vertices_behind->at(i), vertices_behind->at(i + 1)};
    Eigen::Vector2d tris2[3] = {vertices_next->at(i), vertices_behind->at(i + 1), vertices_next->at(i + 1)};
    mesh.push_back(tris1);
    mesh.push_back(tris2);
  }
  return mesh;
}

void convert_stl(XodrMap* xodrMap) {

  size_t number_steps = 20;


  for (const Road& road : xodrMap->roads()) {
    // std::cerr << "New Road" << std::endl;


    ReferenceLine ref_line = road.referenceLine();

    for (LaneSection laneSection : road.laneSections()){

      double step_size = (laneSection.endS() - laneSection.startS()) / number_steps;
      double start = laneSection.startS();
      double s = start;


      std::vector<Eigen::Vector2d> vertices_behind =
	get_one_vertex_row(&laneSection, &ref_line, s, start);

      std::vector<std::array<Eigen::Vector2d, 3>> mesh;

      for (int i = 1; i < number_steps; i++) {
	s += i * step_size;
	std::vector<Eigen::Vector2d> vertices_next =
	  get_one_vertex_row(&laneSection, &ref_line, s, start);

	std::vector<Eigen::Vector2d[3]> new_tris = create_mesh(&vertices_behind, &vertices_next);
	mesh.reserve(mesh.size() + new_tris.size());
	mesh.insert(mesh.end(), new_tris.begin(), new_tris.end());
	vertices_behind = vertices_next;

      }

    }
  }
}

int main() {
  const char* path = xodrFiles[1].path;

  // std::cout << "Loading xodr file: " << path << std::endl;

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
