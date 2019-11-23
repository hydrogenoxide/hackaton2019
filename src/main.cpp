#include <iostream>
#include <sstream>
#include <fstream>
#include "string.h"
#include "xodr/xodr_map.h"
#include "visible_lanes.h"

class Tris {
private:
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  Eigen::Vector2d v3;

public:

  Tris(){
    v1 = Eigen::Vector2d(0.0, 0.0);
    v2 = Eigen::Vector2d(0.0, 0.0);
    v3 = Eigen::Vector2d(0.0, 0.0);
  }

  Tris(Eigen::Vector2d vv1,
       Eigen::Vector2d vv2,
       Eigen::Vector2d vv3) {
    v1 = vv1;
    v2 = vv2;
    v3 = vv3;
  }

  std::vector<Eigen::Vector2d> get_vertecies() {
    std::vector<Eigen::Vector2d> t;
    t.push_back(v1);
    t.push_back(v2);
    t.push_back(v3);
    return t;
  }

};

class Stl{
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

  std::string tris_to_ascii(Tris tris) {
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
    oss << "\t\tvertex " << (v1.x()) << " " << (v1.y()) << " " << "0" << std::endl;
    oss << "\t\tvertex " << (v2.x()) << " " << (v2.y()) << " " << "0" << std::endl;
    oss << "\t\tvertex " << (v3.x()) << " " << (v3.y()) << " " << "0" << std::endl;
    oss << "\tendloop" << std::endl;
    oss << "endfacet" << std::endl;

    return oss.str();
  }

  stl_facet tris_to_facet(Tris tris) {
    stl_facet facet;
    facet.normal[0] = (float) 0.0;
    facet.normal[1] = (float) 0.0;
    facet.normal[2] = (float) 1.0;

    std::vector<Eigen::Vector2d> vertices = tris.get_vertecies();

    Eigen::Vector2d v1 = vertices.at(0);
    Eigen::Vector2d v2 = vertices.at(1);
    Eigen::Vector2d v3 = vertices.at(2);

    facet.v1[0] = (float) v1.x();
    facet.v1[1] = (float) v1.y();
    facet.v1[2] = (float) 0.0;

    facet.v2[0] = (float) v2.x();
    facet.v2[1] = (float) v2.y();
    facet.v3[2] = (float) 0.0;

    facet.v3[0] = (float) v3.x();
    facet.v3[1] = (float) v3.y();
    facet.v3[2] = (float) 0.0;

    return facet;
  }

public:
  Stl(std::vector<Tris> tris, std::string name) {
    facets = tris;
    name = name;
  }

  std::string to_ascii() {
    std::ostringstream oss;
    oss << "solid " << name << std::endl;
    for (Tris t : facets) {
      oss << tris_to_ascii(t);
    }
    oss << "endsolid " << name << std::endl;
    return oss.str();
  }

  void save_binary(std::string filename) {
    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::binary);
    file.imbue(std::locale::classic());

    char header[80];
    std::fill(header, header + sizeof( header ), 0 );
    unsigned int num_facets = facets.size();
    file.write(header, sizeof(header));
    file.write((char*)&num_facets, sizeof(num_facets));

    for (Tris t : facets) {
      Stl::stl_facet f = tris_to_facet(t);

      file.write((char*)&f, sizeof(Stl::stl_facet));

    }

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
