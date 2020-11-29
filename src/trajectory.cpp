#include "trajectory.h"

#include "helpers.h"

Trajectory::Trajectory(const Trajectory& other)
    : x(other.x),
      y(other.y),
      v(other.v),
      end_angle(other.end_angle),
      end_s(other.end_s),
      end_d(other.end_d),
      intended_lane(other.intended_lane),
      end_lane(other.end_lane) {}

void Trajectory::update(const std::vector<double>& x_,
                        const std::vector<double>& y_, double end_s_,
                        double end_d_) {
  x = x_;
  y = y_;
  end_s = end_s_;
  end_d = end_d_;

  if (x.size() != y.size()) {
    throw std::runtime_error(
        "Trajectory::update(): x and y have different lengths");
  }
  if (not v.empty()) {
    consume_velocity_points(PATH_LENGTH - x.size());
  }
}

void Trajectory::consume_velocity_points(unsigned n_points) {
  for (auto i = 0u; i < n_points; ++i) {
    v.pop_front();
  }
}

double Trajectory::calculate_end_angle() {
  if (x.size() > 2 && y.size() > 2) {
    double x2 = x.end()[-1];
    double y2 = y.end()[-1];
    double x1 = x.end()[-2];
    double y1 = y.end()[-2];
    end_angle = atan2(y2 - y1, x2 - x1);
  }
  return end_angle;
}

double Trajectory::calculate_begin_angle() const {
  double begin_angle = 0.;
  if (x.size() > 2 && y.size() > 2) {
    double x2 = x.at(1);
    double y2 = y.at(1);
    double x1 = x.at(0);
    double y1 = y.at(0);
    begin_angle = atan2(y2 - y1, x2 - x1);
  }
  return begin_angle;
}

double Trajectory::get_begin_s(const std::vector<double>& map_x,
                               const std::vector<double>& map_y) const {
  double theta = calculate_begin_angle();

  double s = 0.;
  if (not x.empty() && not y.empty()) {
    auto v = getFrenet(x.front(), y.front(), theta, map_x, map_y);
    s = v[0];
  }
  return s;
}

void Trajectory::calculate_end_frenet(const std::vector<double>& map_x,
                                      const std::vector<double>& map_y) {
  double theta = calculate_end_angle();

  if (not x.empty() && not y.empty()) {
    auto v = getFrenet(x.back(), y.back(), theta, map_x, map_y);
    end_s = v[0];
    end_d = v[1];
  }
}

double Trajectory::get_last_point_velocity() const {
  if (v.empty()) {
    return 0.;
  } else {
    return v.back();
  }
}

void Trajectory::trim(std::size_t new_size) {
  if (new_size > x.size()) {
    x.resize(new_size);
    y.resize(new_size);
  }
}

bool Trajectory::empty() const { return x.empty(); }
std::size_t Trajectory::size() const { return x.size(); }

std::ostream& operator<<(std::ostream& os, const Trajectory& path) {
  os << std::fixed << std::setprecision(2);
  os << "x[" << path.x.front() << " -> " << path.x.back() << "] ";
  os << "y[" << path.y.front() << " -> " << path.y.back() << "] ";
  os << "end_s[" << path.end_s << "] ";
  os << "end_d[" << path.end_d << "] ";
  os << "end_velocity[" << path.get_last_point_velocity() << "] ";
  os << "intended_lane[" << path.intended_lane << "] ";
  os << "end lane[" << path.end_lane << "] ";
  os << "size[" << path.size() << ']';
  return os;
}