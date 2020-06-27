#include "visual/file_types/stl/stl_parser.hpp"
#include "spatial/triangle.hpp"
#include "spatial/vector.hpp"
#include "utils/timer.hpp"

#include <iostream>

namespace rbt::visual {

void STLParser::open_file(const std::string& file_path) {
  this->file = std::ifstream(file_path, std::ios::in | std::ios::binary);

  if(!this->file.is_open()) {
    std::cout << "Error: Couldn't find the file" << std::endl;
    throw;
  }
}

void STLParser::read_header() {
  this->file.read(&(this->header[0]), STLParser::STL_HEADER_SIZE_IN_BYTES);

  std::cout << "STL header: " << header << std::endl;
}

int STLParser::get_number_of_triangles() {
  int num_triangles = 0;
  this->file.read(reinterpret_cast<char*>(&num_triangles), 4);

  std::cout << "STL number of facets: " << num_triangles << std::endl;

  return num_triangles;
}

void STLParser::parse(const std::string& file_path, std::vector<Triangle>& triangles) {
  Timer t("Parse STL");

  this->open_file(file_path);

  this->read_header();

  // ASCII parsing is not implemented.
  if(!this->is_binary()) return;

  const auto number_of_triangles = this->get_number_of_triangles();

  // Read the raw data into an array of Facets
  Facet* facets = new Facet[number_of_triangles];
  file.read(reinterpret_cast<char*>(facets), number_of_triangles * sizeof(Facet));

  triangles.reserve(number_of_triangles);
  for(int i = 0; i < number_of_triangles; ++i) {
    triangles.push_back(Triangle({
      Vector3({ facets[i].a[0], facets[i].a[1], facets[i].a[2] }),
      Vector3({ facets[i].b[0], facets[i].b[1], facets[i].b[2] }),
      Vector3({ facets[i].c[0], facets[i].c[1], facets[i].c[2] }),
    }));
  }
}

}
