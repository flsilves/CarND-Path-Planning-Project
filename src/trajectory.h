#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <json.hpp>
#include "parameters.h"
#include <iomanip>



class Trajectory {
 public:
  Trajectory() = default;

  Trajectory(const Trajectory& other);

  void update(nlohmann::json telemetry_data);

  double calculate_end_angle();

  void calculate_end_frenet(const std::vector<double>& map_x,
                            const std::vector<double>& map_y);

  void trim(std::size_t new_size);

  bool empty() const;
  std::size_t size() const;
 
 public:
  std::vector<double> x, y;
  double end_angle{0.0};
  double end_s{0.0}, end_d{0.0};
};

std::ostream& operator<<(std::ostream& os, const Trajectory& path);


#endif