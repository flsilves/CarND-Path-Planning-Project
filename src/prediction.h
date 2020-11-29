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

  // double vehicle_close_ahead(int steps_into_future, double ego_future_s,
  //                           int ego_lane, double ego_s) const;

  void reset_lane_speeds();
  unsigned get_fastest_lane() const;

  void reset_gaps();
  void predict(const Trajectory& previous_trajectory);

 public:
  std::array<std::deque<VehicleState>, DETECTED_VEHICLES> history;
  std::array<Gap, NUMBER_OF_LANES> predicted_gaps;
  std::array<double, NUMBER_OF_LANES> lane_speeds;
  const MapWaypoints map;
  const VehicleState& ego;
};

std::ostream& operator<<(std::ostream& os, const Prediction& rhs);

#endif