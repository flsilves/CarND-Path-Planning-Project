
#include "prediction.h"

#include "vehicle.h"

Prediction::Prediction(const MapWaypoints& map, const VehicleState& ego)
    : map(map), ego(ego){};

void Prediction::update(nlohmann::json sensor_fusion) {
  for (auto& object_entry : sensor_fusion) {
    std::size_t id = object_entry[0];
    double x = object_entry[1];
    double y = object_entry[2];
    double vx = object_entry[3];
    double vy = object_entry[4];
    double s = object_entry[5];
    double d = object_entry[6];

    VehicleState vehicle_detected{id, x, y, vx / 0.44704, vy / 0.44704, s, d};

    if (vehicle_detected.in_right_side_of_road()) {
      detected_vehicles[id] = vehicle_detected;
    } else {
      detected_vehicles[id] = {};
    }
  }
}

void Prediction::reset_lane_speeds() {
  for (auto& lane_speed : lane_speeds) {
    lane_speed = TARGET_EGO_SPEED;
  }
}

unsigned Prediction::get_fastest_lane() const {
  auto result = std::max_element(lane_speeds.begin(), lane_speeds.end());
  return std::distance(lane_speeds.begin(), result);
}

void Prediction::reset_gaps() {
  for (auto& gap : predicted_gaps) {
    gap = Gap{SENSOR_RANGE_METERS, SENSOR_RANGE_METERS, {}, {}};
  }
}

void Prediction::predict(const Trajectory& previous_trajectory) {
  reset_gaps();
  reset_lane_speeds();

  constexpr auto future_time = PATH_LENGTH * TIME_PER_POINT;
  constexpr auto TRAFFIC_SPEED_HORIZON_DISTANCE{100.0};

  for (auto& traffic_vehicle : detected_vehicles) {
    if (not traffic_vehicle.is_valid()) {
      continue;
    }

    auto future_traffic_vehicle =
        traffic_vehicle.get_prediction(future_time, map.x, map.y);

    auto vehicle_lane = traffic_vehicle.get_lane();

    // Not worth to predict the state of car behind ego lane
    if ((vehicle_lane == ego.get_lane()) && (ego.s > traffic_vehicle.s)) {
      continue;
    }

    auto delta_s = fabs(traffic_vehicle.s - ego.s);

    auto& lane_gap = predicted_gaps[vehicle_lane];
    auto& lane_speed = lane_speeds[vehicle_lane];

    if (traffic_vehicle.s > ego.s) {
      if (delta_s < lane_gap.distance_ahead) {
        lane_gap.distance_ahead = delta_s;
        lane_gap.vehicle_ahead = {traffic_vehicle, future_traffic_vehicle};
      }
    } else {
      if (delta_s < lane_gap.distance_behind) {
        lane_gap.distance_behind = delta_s;
        lane_gap.vehicle_behind = {traffic_vehicle, future_traffic_vehicle};
      }
    }

    // Calculate lane speeds based on vehicles ahead
    if ((traffic_vehicle.s > ego.s) &&
        (traffic_vehicle.s < (ego.s + TRAFFIC_SPEED_HORIZON_DISTANCE))) {
      lane_speed = fmin(lane_speed, traffic_vehicle.speed);
    }
  }
}

std::ostream& operator<<(std::ostream& os, const Prediction& rhs) {
  os << std::fixed << std::setprecision(2);
  // clang-format off
  for (auto& vehicle : rhs.detected_vehicles) {
    if (vehicle.is_valid()) {
      os << vehicle << '\n';
    } else {
      os << "EMPTY" << '\n';
    }
  }
  os << '\n';
  os << "|LANE_SPEEDS|\n";
  os << "Left[" << rhs.lane_speeds[0] << "] ";
  os << "Center[" << rhs.lane_speeds[1]  << "] ";
  os << "Right[" << rhs.lane_speeds[2] << "]\n\n";

  os << "|GAPS|\n";
  os << "Left[" << rhs.predicted_gaps[0].distance_behind << " <-> " << rhs.predicted_gaps[0].distance_ahead << "] ";
  os << "Center[" << rhs.predicted_gaps[1].distance_behind << " <-> " << rhs.predicted_gaps[1].distance_ahead << "] ";
  os << "Right[" << rhs.predicted_gaps[2].distance_behind << " <-> " << rhs.predicted_gaps[2].distance_ahead << "] ";
  // clang-format on
  return os;
}