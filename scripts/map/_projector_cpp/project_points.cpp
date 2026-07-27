// Small CLI that reuses Autoware's own projection code
// (autoware::map_projection_loader::load_info_from_yaml +
//  autoware::geography_utils::project_forward) to convert lat/lon points
// into the Autoware "map" frame, exactly as lanelet2_map_loader does.
//
// This exists so check_map.py can verify a lanelet2 map's bounding box in
// map-frame metres without reimplementing any projection math by hand: the
// same yaml parser and the same projector classes Autoware itself uses at
// launch time are linked in here.
//
// Usage:
//   project_points <map_projector_info.yaml>
// Input (stdin): one "lat lon" pair per line, until EOF.
// Output (stdout): one "PROJECTOR_TYPE <type>" line, then one "x y" pair
// per input point, in map-frame metres.

#include <autoware/geography_utils/projection.hpp>
#include <autoware/map_projection_loader/map_projection_loader.hpp>

#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char ** argv)
{
  using autoware::geography_utils::project_forward;
  using GeoPoint = geographic_msgs::msg::GeoPoint;

  if (argc != 2) {
    std::cerr << "usage: project_points <map_projector_info.yaml>\n";
    return 2;
  }

  autoware_map_msgs::msg::MapProjectorInfo info;
  try {
    info = autoware::map_projection_loader::load_info_from_yaml(argv[1]);
  } catch (const std::exception & e) {
    std::cerr << "ERROR: failed to load " << argv[1] << ": " << e.what() << "\n";
    return 2;
  }

  std::cout << "PROJECTOR_TYPE " << info.projector_type << "\n";

  std::string line;
  size_t n = 0;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {
      continue;
    }
    std::istringstream iss(line);
    double lat, lon;
    if (!(iss >> lat >> lon)) {
      std::cerr << "ERROR: malformed point line " << n << ": " << line << "\n";
      return 2;
    }
    GeoPoint gp;
    gp.latitude = lat;
    gp.longitude = lon;
    gp.altitude = 0.0;
    try {
      auto local = project_forward(gp, info);
      std::cout.precision(9);
      std::cout << local.x << " " << local.y << "\n";
    } catch (const std::exception & e) {
      std::cerr << "ERROR: projection failed at point " << n << ": " << e.what() << "\n";
      return 3;
    }
    ++n;
  }
  return 0;
}
