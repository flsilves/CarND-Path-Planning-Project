#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class Map {
 public:
  Map(const std::string &map_filepath, double max_s) : max_s(max_s) {
    read_map(map_filepath);
  }

 private:
  void read_map(const std::string &map_filepath);

 public:
  double max_s{0.};
  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;
};


void Map::read_map(const std::string &map_filepath) {

  std::ifstream input_filestream(map_filepath.c_str(), std::ifstream::in);

  if(not input_filestream.good()) {
    throw std::runtime_error(std::string("Unable to open file: \"") + map_filepath + '"');
  }

  std::string line;
  while (getline(input_filestream, line)) {
    std::istringstream iss(line);
    double x,y;
    float s, d_x, d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    waypoints_x.push_back(x);
    waypoints_y.push_back(y);
    waypoints_s.push_back(s);
    waypoints_dx.push_back(d_x);
    waypoints_dy.push_back(d_y);
  }
}