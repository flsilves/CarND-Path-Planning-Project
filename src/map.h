#ifndef MAP_H
#define MAP_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class MapWaypoints {
 public:
  MapWaypoints(const std::string &map_filepath, double max_s);

 private:
  void read_map(const std::string &map_filepath);

 public:
  double max_s{0.};
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> dx;
  std::vector<double> dy;
};

#endif
