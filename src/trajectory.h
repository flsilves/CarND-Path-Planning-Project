#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <deque>
#include <iomanip>
#include <vector>

#include "parameters.h"

class Trajectory {
 public:
  Trajectory() = default;

  Trajectory(const Trajectory& other);

  void update(const std::vector<double>& x, const std::vector<double>& y,
              double end_s, double end_d);

  double calculate_end_angle();
  double calculate_begin_angle() const;

  void calculate_end_frenet(const std::vector<double>& map_x,
                            const std::vector<double>& map_y);

  double get_last_point_velocity() const;

  double get_begin_s(const std::vector<double>& map_x,
                     const std::vector<double>& map_y) const;

  void trim(std::size_t new_size);
  void consume_velocity_points(unsigned n_points);

  bool empty() const;
  std::size_t size() const;

  friend std::ostream& operator<<(std::ostream& os, const Trajectory& path);

 public:
  std::vector<double> x{}, y{};
  std::deque<double> v{};
  double end_angle{0.0};
  double end_s{0.0}, end_d{0.0};
  unsigned intended_lane{1}, end_lane{1};
};

#endif