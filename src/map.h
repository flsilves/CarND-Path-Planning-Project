#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class MapWaypoints {
 public:
  MapWaypoints(const std::string &map_filepath, double max_s) : max_s(max_s) {
    read_map(map_filepath);
  }

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


void MapWaypoints::read_map(const std::string &map_filepath) {

  std::ifstream input_filestream(map_filepath.c_str(), std::ifstream::in);

  if(not input_filestream.good()) {
    throw std::runtime_error(std::string("Unable to open file: \"") + map_filepath + '"');
  }

  std::string line;
  while (getline(input_filestream, line)) {
    std::istringstream iss(line);
    double x_,y_;
    float s_, d_x_, d_y_;
    iss >> x_;
    iss >> y_;
    iss >> s_;
    iss >> d_x_;
    iss >> d_y_;
    x.push_back(x_);
    y.push_back(y_);
    s.push_back(s_);
    dx.push_back(d_x_);
    dy.push_back(d_y_);
  }
}