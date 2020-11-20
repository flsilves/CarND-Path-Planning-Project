#include "trajectory_generator.h"

#include "helpers.h"

namespace {
static tk::spline spline;
}  // namespace

TrajectoryGenerator::TrajectoryGenerator(const Trajectory& previous_trajectory,
                                         const VehicleState& ego,
                                         const MapWaypoints& map)
    : previous_trajectory_(previous_trajectory), ego_(ego), map(map) {}

Trajectory TrajectoryGenerator::generate_trajectory(double target_velocity,
                                                    int target_lane) {
  const double anchor_spacement = 50.0;
  const unsigned extra_anchors = 2;
  const unsigned path_length = 50;

  anchors_init();  // move this outside -> just call once per cycle
  anchors_trim();
  anchors_add(anchor_spacement, extra_anchors, target_lane);
  anchors_recenter();

  spline.set_points(anchors_x, anchors_y);

  auto generated_path = previous_trajectory_;

  // generated_path.trim(10);

  auto missing_points = path_length - generated_path.size();

  auto x_increment = get_x_increment(target_velocity);

  double x{0}, y{0};

  for (int i = 1; i <= missing_points; ++i) {
    x = x + x_increment;
    y = spline(x);

    generated_path.x.push_back(x * cos(ref_yaw) - y * sin(ref_yaw) + ref_x);
    generated_path.y.push_back(x * sin(ref_yaw) + y * cos(ref_yaw) + ref_y);
  }

  generated_path.calculate_end_frenet(map.x, map.y);

  return generated_path;
}

double TrajectoryGenerator::get_x_increment(double target_velocity) {
  constexpr double time_per_point = .02;

  double horizon_x = 30.0;
  double horizon_y = spline(horizon_x);
  double target_dist =
      sqrt((horizon_x) * (horizon_x) + (horizon_y) * (horizon_y));
  double N = target_dist / (time_per_point * target_velocity * MPH_2_MPS);
  return horizon_x / N;
}

void TrajectoryGenerator::anchors_init() {
  anchors_x.clear();
  anchors_y.clear();
  ref_yaw = deg2rad(ego_.yaw);

  std::size_t prev_size = previous_trajectory_.size();

  if (prev_size < 2) {
    anchors_x.push_back(ego_.x - cos(ref_yaw));
    anchors_x.push_back(ego_.x);

    anchors_y.push_back(ego_.y - sin(ref_yaw));
    anchors_y.push_back(ego_.y);
  } else {
    ref_x = previous_trajectory_.x.end()[-1];
    ref_y = previous_trajectory_.y.end()[-1];

    double ref_x_prev = previous_trajectory_.x.end()[-2];
    double ref_y_prev = previous_trajectory_.y.end()[-2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // std::cout << "ref_yaw:" << rad2deg(ref_yaw) << '\n';

    anchors_x.push_back(ref_x_prev);
    anchors_x.push_back(ref_x);

    anchors_y.push_back(ref_y_prev);
    anchors_y.push_back(ref_y);
  }
  ref_x = anchors_x.at(1);
  ref_y = anchors_y.at(1);
}

void TrajectoryGenerator::anchors_add(double anchor_spacement,
                                      unsigned extra_anchors, int target_lane) {
  for (auto i = 1u; i <= extra_anchors; ++i) {
    auto next_anchor = getXY(ego_.s + (i) * (anchor_spacement),
                             (2 + 4 * target_lane), map.s, map.x, map.y);

    anchors_x.emplace_back(next_anchor[0]);
    anchors_y.emplace_back(next_anchor[1]);
  }
}

void TrajectoryGenerator::anchors_recenter() {
  for (int i = 0; i < anchors_x.size(); ++i) {
    // shift car reference angle to 0 degrees
    double shift_x = anchors_x[i] - ref_x;
    double shift_y = anchors_y[i] - ref_y;

    anchors_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    anchors_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }
}

void TrajectoryGenerator::anchors_trim() {
  anchors_x.resize(2);
  anchors_y.resize(2);
}
