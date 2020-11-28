
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

    auto& vehicle_history = history[id];

    if (vehicle_detected.in_right_side_of_road()) {
      if (not vehicle_history.empty()) {
        auto& previous_state = vehicle_history.back();

        bool continuous = previous_state.evaluate_continuity(vehicle_detected);

        if (not continuous) {
          // std::cout << "ID[" << id << "] not continous!" << '\n';
          vehicle_history.clear();
        }
      }

      vehicle_history.push_back(vehicle_detected);

    } else {
      vehicle_history.clear();
    }

    if (vehicle_history.size() > OBJECT_HISTORY_SIZE) {
      vehicle_history.pop_front();
    }
  }
}

// VehicleState Prediction::get_front_vehicle()
/*
VehicleState Prediction::vehicle_close_ahead(int steps_into_future,
                                             double ego_planned_s, int ego_lane,
                                             double ego_s) const {
  double predicted_gap{1000};
  VehicleState closest_front_vehicle{};

  for (auto& vehicle_history : history) {
    if (vehicle_history.empty()) {
      continue;
    }

    auto vehicle = vehicle_history.back();
    bool vehicle_in_same_lane = (vehicle.get_lane() == ego_lane);
    bool vehicle_currently_in_front = (vehicle.s > ego_s);

    if (vehicle_in_same_lane && vehicle_currently_in_front) {
      double future_vehicle_s =
          vehicle.s + vehicle.speed * steps_into_future * TIME_PER_POINT;

      double future_gap = future_vehicle_s - ego_planned_s;
      if (future_gap < predicted_gap) {
        predicted_gap = future_gap;
        closest_front_vehicle = vehicle;
      }
    }
  }
  return closest_front_vehicle;
} */

void Prediction::reset_lane_speeds() {
  for (auto& lane_speed : lane_speeds) {
    lane_speed = MAX_LANE_SPEED;
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

void Prediction::predict_gaps(VehicleState ego, double future_ego_s,
                              double future_time) {
  reset_gaps();
  reset_lane_speeds();

  constexpr auto TRAFFIC_SPEED_HORIZON_DISTANCE{100.0};

  for (auto& vehicle_history : history) {
    if (vehicle_history.empty()) {
      continue;
    }

    auto& traffic_vehicle = vehicle_history.back();
    auto future_traffic_vehicle =
        traffic_vehicle.get_prediction(future_time, map.x, map.y);

    // std::cout << "traffic_vehicle:" << traffic_vehicle << '\n';
    // std::cout << "future_traffic_vehicle:" << future_traffic_vehicle <<
    // '\n';

    auto vehicle_lane = traffic_vehicle.get_lane();

    // Irrelevant to predict state of the cars that are behind in the same
    // lane
    if (vehicle_lane == ego.get_lane() && ego.s > traffic_vehicle.s) {
      continue;
    }

    auto delta_s = fabs(future_traffic_vehicle.s - future_ego_s);

    auto& lane_gap = predicted_gaps[vehicle_lane];
    auto& lane_speed = lane_speeds[vehicle_lane];

    if (future_traffic_vehicle.s > future_ego_s) {
      if (delta_s < lane_gap.distance_ahead) {
        lane_gap.distance_ahead = delta_s;
        lane_gap.vehicle_ahead = future_traffic_vehicle;
        // lane_speed = traffic_vehicle.speed;
      }
    } else {
      if (delta_s < lane_gap.distance_behind) {
        // std::cout << "HERE!!!!!!!!!!!!!!!!!!!11" << std::endl;
        lane_gap.distance_behind = delta_s;
        // std::cout << "delta_s:" << delta_s << std::endl;

        lane_gap.vehicle_behind = future_traffic_vehicle;
        // std::cout << lane_gap.vehicle_behind << std::endl;
      }
    }

    // std::cout << "vehicle.s[" << traffic_vehicle.s << "] ego.s[" << ego.s
    //          << "]";
    if ((traffic_vehicle.s > ego.s) &&
        (traffic_vehicle.s < (ego.s + TRAFFIC_SPEED_HORIZON_DISTANCE))) {
      lane_speed = fmin(lane_speed, traffic_vehicle.speed);
    }
  }
}

std::ostream& operator<<(std::ostream& os, const Prediction& rhs) {
  os << std::fixed << std::setprecision(2);
  // clang-format off
  //for (auto& vehicle_history : rhs.history) {
  //  if (not vehicle_history.empty()) {
  //    os << vehicle_history.back() << '\n';
  //  } else {
  //    os << "EMPTY" << '\n';
  //  }
  //}
  //os << '\n';
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