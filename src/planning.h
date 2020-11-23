#ifndef PLANNING_H
#define PLANNING_H

#include "map.h"
#include "parameters.h"
#include "prediction.h"
#include "trajectory.h"
#include "trajectory_generator.h"
#include "vehicle.h"

enum class State {
  KeepLane,
  PrepareLeft,
  ChangeLeft,
  PrepareRight,
  ChangeRight
};

class Planner {
 public:
  Planner(const VehicleState& ego, TrajectoryGenerator& gen,
          const Prediction& predictions, const MapWaypoints& map);
  Planner step();

  Trajectory get_trajectory();

  Trajectory plan_trajectory(const std::string& candidate_state);

  std::vector<std::string> successor_states();

 public:
  std::string state{std::string("KL")};
  double target_velocity;
  unsigned target_lane{1};
  TrajectoryGenerator& trajectory_generator;
  const VehicleState& ego;
  const MapWaypoints& map;
  const Prediction& predictions;
};

std::ostream& operator<<(std::ostream& os, const Planner& planner);

#endif