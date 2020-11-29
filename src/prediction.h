#ifndef PREDICTION_H
#define PREDICTION_H

#include <array>
#include <deque>
#include <iomanip>
#include <json.hpp>

#include "trajectory.h"
#include "map.h"
#include "parameters.h"
#include "vehicle.h"

struct Gap {
  double distance_behind;
  double distance_ahead;
  std::pair<VehicleState, VehicleState> vehicle_ahead;
  std::pair<VehicleState, VehicleState> vehicle_behind;
};

class Prediction {
 public:
  Prediction(const MapWaypoints& map, const VehicleState& ego);
  void update(nlohmann::json sensor_fusion);

  void reset_lane_speeds();
  unsigned get_fastest_lane() const;

  void reset_gaps();
  void predict(const Trajectory& previous_trajectory);

 public:
  std::array<VehicleState, N_DETECTED_VEHICLES> detected_vehicles;
  std::array<Gap, NUMBER_OF_LANES> predicted_gaps;
  std::array<double, NUMBER_OF_LANES> lane_speeds;
  const MapWaypoints map;
  const VehicleState& ego;
};

std::ostream& operator<<(std::ostream& os, const Prediction& rhs);

#endif