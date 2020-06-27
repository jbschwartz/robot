#ifndef __STL_PARSER_H__
#define __STL_PARSER_H__

#include <fstream>
#include <string>
#include <vector>

#include "typedefs.hpp"

namespace rbt {
  class Triangle;
}

namespace rbt::visual {

// A class for reading in STL files into an internal representation of a mesh.
class STLParser {
public:
  // Open and parse the given file and populate the provided vector of triangles.
  void parse(const std::string& file_path, std::vector<Triangle>& triangles);

private:
  // The layout of each facet in the STL file.
  // See more: https://en.wikipedia.org/wiki/STL_(file_format)
  struct Facet {
    Real normal[3];
    Real a[3];
    Real b[3];
    Real c[3];
    uint16_t attribute;
  }  __attribute__ ((packed));

  // The number of bytes comprising the header at the top of an STL file.
  static const int STL_HEADER_SIZE_IN_BYTES = 80;

  // The input filestream for the current file being parsed.
  std::ifstream file;

  // The header string for the file currenting being parsed. Initialized to the appropriate size.
  std::string header = std::string(STLParser::STL_HEADER_SIZE_IN_BYTES, ' ');

  // Open the file and check for success. Throw an exception on failure.
  void open_file(const std::string& file_path);

  // Read the header of the STL file.
  void read_header();

  // Returns true if the STL file is believed to be binary.
  // STL files are considered to be binary if they do not start with the word "solid".
  inline bool is_binary() { return this->header.rfind("solid", 0) != 0; };

  // Returns the number of triangles in the STL file.
  int get_number_of_triangles();
};

}

#endif /* __STL_PARSER_H__ */
