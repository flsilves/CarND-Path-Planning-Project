#include "trajectory_generator.h"

#include "helpers.h"

namespace {
static tk::spline spline;
}  // namespace

static double planned_velocity{0.0};

TrajectoryGenerator::TrajectoryGenerator(const Trajectory& previous_trajectory,
                                         const VehicleState& ego,
                                         const MapWaypoints& map,
                                         const Prediction& predictions)
    : previous_trajectory(previous_trajectory),
      ego(ego),
      map(map),
      predictions(predictions) {}

double TrajectoryGenerator::calculate_velocity(
    double previous_velocity) {  // avs

  // std::cout << "PREVIOUS VELOCITY" << previous_velocity << '\n';
  if (velocity < 49.5) {
    velocity += MAX_ACCELERATION;
  }

  return velocity;
  // if (ego.speed <= TARGET_EGO_SPEED) {
  //  std::cout << "EGO_SPEED:" << ego.speed << previous_velocity << '\n';
  //
  //  std::cout << "first:" << previous_velocity + MAX_ACCELERATION << '\n';
  //
  //  std::cout << "second:"
  //            << previous_velocity + (TARGET_EGO_SPEED - previous_velocity)
  //            << '\n';
  //
  //  return fmin(previous_velocity + MAX_ACCELERATION,
  //              previous_velocity + (TARGET_EGO_SPEED - previous_velocity));
  //} else {
  //  return TARGET_EGO_SPEED;
  //}
}

Trajectory TrajectoryGenerator::generate_trajectory(unsigned end_lane,
                                                    unsigned intended_lane) {
  auto new_trajectory = previous_trajectory;
  // new_trajectory.trim(10);

  // if(end lane != current lane) -> trim

  // predict where front and rear car are going to be at 50

  // calculate velocity for safe distance

  // clang-format off
  // if(lane_change) // trim current path -> validate at the end for collisions -> return empty if not valid
  // if(prepare) // evaluate gap -> calculate velocity for maneuver
  // if(keep lane) // check for gap or cutting in vehicles
  // clang-format on
  auto target_velocity = calculate_velocity(ego.speed);

  std::cout << "Target velocity" << target_velocity << std::endl;

  fill_trajectory_points(new_trajectory, target_velocity, end_lane);
  new_trajectory.calculate_end_frenet(map.x, map.y);
  new_trajectory.end_velocity = target_velocity;

  std::cout << "FUCK" << new_trajectory.end_velocity << '\n';
  return new_trajectory;
}

void TrajectoryGenerator::fill_trajectory_points(Trajectory& trajectory,
                                                 double target_velocity,
                                                 unsigned end_lane) {
  const double anchor_spacement = 50.0;
  const unsigned extra_anchors = 2;
  const unsigned path_length = 50;

  anchors_init();
  anchors_trim();
  anchors_add(anchor_spacement, extra_anchors, end_lane);
  anchors_recenter();

  spline.set_points(anchors_x, anchors_y);

  auto missing_points = path_length - trajectory.size();

  auto x_increment = get_x_increment(target_velocity);

  double x{0}, y{0};

  for (int i = 1; i <= missing_points; ++i) {
    x = x + x_increment;
    y = spline(x);

    trajectory.x.push_back(x * cos(ref_yaw) - y * sin(ref_yaw) + ref_x);
    trajectory.y.push_back(x * sin(ref_yaw) + y * cos(ref_yaw) + ref_y);
  }
}

double TrajectoryGenerator::get_x_increment(double target_velocity) {
  constexpr double time_per_point = .02;

  double horizon_x = 30.0;
  double horizon_y = spline(horizon_x);
  double target_distance =
      sqrt((horizon_x) * (horizon_x) + (horizon_y) * (horizon_y));
  double N = target_distance / (time_per_point * target_velocity * MPH_2_MPS);
  return horizon_x / N;
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
