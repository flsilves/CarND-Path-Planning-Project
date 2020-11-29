#ifndef VEHICLE_H
#define VEHICLE_H

#include <cstdio>
#include <iomanip>
#include <json.hpp>
#include <vector>

#include "parameters.h"

class VehicleState {
 public:
  VehicleState() = default;

  VehicleState(const VehicleState& other) = default;

  VehicleState(std::size_t id, double x, double y, double vx, double vy,
               double s, double d);

  VehicleState get_prediction(double future_time,
                              const std::vector<double>& map_x,
                              const std::vector<double>& map_y);

  bool left_lane_exists() const;
  bool right_lane_exists() const;

  double calculate_distance_to(const VehicleState& other) const;
  bool is_valid() const;

  void update(double x, double y, double s, double d, double yaw, double speed);

  int get_lane(double lane_width = 4.0) const;
  bool in_right_side_of_road();

 public:
  std::size_t id{42};
  double x{0.0}, y{0.0};
  double vx{0.0}, vy{0.0};
  double d{0.0}, s{0.0};
  double yaw{0.0};
  double speed{0.0};
};

std::ostream& operator<<(std::ostream& os, const VehicleState& vehicle);

#endif