
#include "vehicle.h"

#include "helpers.h"

using std::vector;

VehicleState::VehicleState(std::size_t id, double x, double y, double vx,
                           double vy, double s, double d)
    : id(id), x(x), y(y), vx(vx), vy(vy), s(s), d(d) {
  speed = sqrt(vx * vx + vy * vy);
  yaw = rad2deg(atan2(vy, vx));
  if (yaw < 0) {
    yaw += 360;
  }
}

void VehicleState::update(double x_, double y_, double s_, double d_,
                          double yaw_, double speed_) {
  x = x_;
  y = y_;
  s = s_;
  d = d_;
  yaw = yaw_;      // degrees [positive in ccw direction]
  speed = speed_;  // units m/s
}

int VehicleState::get_lane(double lane_width) const {
  return fmax(fmin(2, floor(d / lane_width)), 0);
}

bool VehicleState::left_lane_exists() const { return d > 4.0; }

bool VehicleState::right_lane_exists() const { return d < 12.0; }

bool VehicleState::is_valid() const { return id != 42; }

double VehicleState::calculate_distance_to(const VehicleState& other) const {
  return distance(x, y, other.x, other.y);
}

VehicleState VehicleState::get_prediction(double future_time,
                                          const vector<double>& map_x,
                                          const vector<double>& map_y) {
  VehicleState prediction{*this};
  auto rad_yaw = deg2rad(yaw);
  prediction.x = x + speed * MPH_2_MPS * future_time * cos(rad_yaw);
  prediction.y = y + speed * MPH_2_MPS * future_time * sin(rad_yaw);

  auto v = getFrenet(prediction.x, prediction.y, rad_yaw, map_x, map_y);
  prediction.s = v[0];
  prediction.d = v[1];

  return prediction;
}

bool VehicleState::in_right_side_of_road() { return (d <= 12.0) && (d >= 0.0); }

std::ostream& operator<<(std::ostream& os, const VehicleState& vehicle) {
  os << std::fixed << std::setprecision(2);
  os << "id[" << std::setw(2) << vehicle.id << "] ";
  os << "lane[" << vehicle.get_lane() << "] ";
  os << "x[" << std::setw(7) << vehicle.x << "] ";
  os << "y[" << std::setw(7) << vehicle.y << "] ";
  os << "s[" << std::setw(7) << vehicle.s << "] ";
  os << "d[" << std::setw(5) << vehicle.d << "] ";
  os << "speed[" << std::setw(5) << vehicle.speed << "] ";
  os << "yaw[" << std::setw(6) << vehicle.yaw << ']';
  return os;
}