#include "trajectory_generator.h"

#include "helpers.h"
#include "parameters.h"

namespace {
static tk::spline spline;
}  // namespace

TrajectoryGenerator::TrajectoryGenerator(const Trajectory& previous_trajectory,
                                         const VehicleState& ego,
                                         const MapWaypoints& map,
                                         const Prediction& predictions)
    : previous_trajectory(previous_trajectory),
      ego(ego),
      map(map),
      predictions(predictions) {}

double TrajectoryGenerator::get_keep_lane_velocity(Trajectory& new_trajectory) {
  double planned_velocity = 49.5;
  unsigned ego_lane = ego.get_lane();

  double predicted_front_gap =
      predictions.predicted_gaps[ego_lane].distance_ahead;
  double current_lane_speed = predictions.lane_speeds[ego_lane];

  if (predicted_front_gap < 0) {
    // std::cout << "NEGATIVE GAP" << std::endl;
    planned_velocity = current_lane_speed - 10.0;  // EMERGENCY_BRAKE
    new_trajectory.trim(5);
  } else if ((predicted_front_gap / ego.speed) < KEEP_DISTANCE_TIME) {
    planned_velocity = predicted_front_gap / KEEP_DISTANCE_TIME;
  }
  return planned_velocity;
}

double TrajectoryGenerator::prepare_lane_change_velocity(
    Trajectory& new_trajectory) {
  double planned_velocity = 49.5;
  return planned_velocity;
}

double TrajectoryGenerator::lane_change_velocity(Trajectory& new_trajectory) {
  double planned_velocity = 49.5;
  return planned_velocity;
}

Trajectory TrajectoryGenerator::generate_trajectory(unsigned intended_lane,
                                                    unsigned end_lane) {
  auto new_trajectory = previous_trajectory;

  double planned_velocity;

  if (ego.get_lane() == end_lane) {
    planned_velocity = get_keep_lane_velocity(new_trajectory);

    if (end_lane != intended_lane) {
      planned_velocity =
          fmin(planned_velocity, prepare_lane_change_velocity(new_trajectory));
    }
  } else {
    planned_velocity = lane_change_velocity(new_trajectory);
  }

  fill_trajectory_points(new_trajectory, planned_velocity, end_lane);
  new_trajectory.calculate_end_frenet(map.x, map.y);
  return new_trajectory;
}

void TrajectoryGenerator::fill_trajectory_points(Trajectory& trajectory,
                                                 double target_velocity,
                                                 unsigned end_lane) {
  // Hyper-parameters
  const double anchor_spacement{50.0};
  const unsigned extra_anchors{2};

  anchors_trim();
  anchors_add(anchor_spacement, extra_anchors, end_lane);
  anchors_recenter();

  spline.set_points(anchors_x, anchors_y);

  auto missing_points = PATH_LENGTH - trajectory.size();
  std::cout << "Missing points" << missing_points << std::endl;

  double x{0}, y{0};
  double next_point_velocity = trajectory.get_last_point_velocity();
  double target_distance = get_target_distance();

  for (auto i = 0u; i < missing_points; ++i) {
    next_point_velocity =
        get_next_point_velocity(next_point_velocity, target_velocity);

    auto xy = calculate_next_point(x, next_point_velocity, target_distance);
    x = xy[0];
    y = xy[1];

    trajectory.x.push_back(x * cos(ref_yaw) - y * sin(ref_yaw) + ref_x);
    trajectory.y.push_back(x * sin(ref_yaw) + y * cos(ref_yaw) + ref_y);
    trajectory.v.push_back(next_point_velocity);
  }
}

double TrajectoryGenerator::get_next_point_velocity(
    double last_planned_velocity, double target_velocity) {
  std::cout << "Target" << target_velocity << std::endl;

  std::cout << "last_planned_velocity" << last_planned_velocity << std::endl;

  if (last_planned_velocity < target_velocity) {
    last_planned_velocity +=
        fmin(MAX_ACCELERATION, target_velocity - last_planned_velocity);
    std::cout << "future_ego_speed" << last_planned_velocity << std::endl;
  } else if (last_planned_velocity > target_velocity) {
    last_planned_velocity -=
        fmin(MAX_ACCELERATION, last_planned_velocity - target_velocity);
    std::cout << "future_ego_speed" << last_planned_velocity << std::endl;
  }
  return last_planned_velocity;
}

double TrajectoryGenerator::get_target_distance() {
  double horizon_x = HORIZON_DISTANCE;
  double horizon_y = spline(horizon_x);
  double target_distance =
      sqrt((horizon_x) * (horizon_x) + (horizon_y) * (horizon_y));

  return target_distance;
}

std::vector<double> TrajectoryGenerator::calculate_next_point(
    double starting_x, double target_velocity, double target_distance) {
  double N = target_distance / (TIME_PER_POINT * target_velocity * MPH_2_MPS);
  double x = starting_x + HORIZON_DISTANCE / N;

  double y = spline(x);
  return {x, y};
}

void TrajectoryGenerator::anchors_init() {
  anchors_x.clear();
  anchors_y.clear();
  ref_yaw = deg2rad(ego.yaw);

  std::size_t prev_size = previous_trajectory.size();

  if (prev_size < 2) {
    anchors_x.push_back(ego.x - cos(ref_yaw));
    anchors_x.push_back(ego.x);

    anchors_y.push_back(ego.y - sin(ref_yaw));
    anchors_y.push_back(ego.y);
  } else {
    ref_x = previous_trajectory.x.end()[-1];
    ref_y = previous_trajectory.y.end()[-1];

    double ref_x_prev = previous_trajectory.x.end()[-2];
    double ref_y_prev = previous_trajectory.y.end()[-2];

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
    auto next_anchor = getXY(ego.s + (i) * (anchor_spacement),
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
