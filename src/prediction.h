#ifndef PREDICTION_H
#define PREDICTION_H

#include <array>
#include <deque>
#include <iomanip>
#include <json.hpp>

#include "map.h"
#include "parameters.h"
#include "trajectory.h"
#include "vehicle.h"

struct Gap {
  double distance_behind;
  double distance_ahead;
  std::pair<VehicleState, VehicleState> vehicle_ahead; // <current_state, future_state>
  std::pair<VehicleState, VehicleState> vehicle_behind;// <current_state, future_state>
};

class Prediction {
 public:
  Prediction(const MapWaypoints& map, const VehicleState& ego);
  void update(nlohmann::json sensor_fusion);

  void reset_lane_speeds();
  unsigned get_fastest_lane() const;

  void reset_gaps();
  void predict(const Trajectory& previous_trajectory);
  bool safe_gap_for_trajectory(unsigned lane,
                               const Trajectory& trajectory) const;

  friend std::ostream& operator<<(std::ostream& os, const Prediction& rhs);

 public:
  std::array<VehicleState, N_DETECTED_VEHICLES> detected_vehicles;
  std::array<Gap, NUMBER_OF_LANES> predicted_gaps;
  std::array<double, NUMBER_OF_LANES> lane_speeds;
  const MapWaypoints map;
  const VehicleState& ego;
};

#endif