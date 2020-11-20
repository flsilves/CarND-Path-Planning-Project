#ifndef PLANNING_H
#define PLANNING_H

#include "trajectory.h"
#include "vehicle.h"
#include "map.h"
#include "prediction.h"
#include "trajectory_generator.h"
#include "parameters.h"

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

  Trajectory get_trajectory();
 public:
  const VehicleState& ego;
  TrajectoryGenerator& trajectory_generator;
  const MapWaypoints& map;
  const Prediction& predictions;
  State state;
  double target_velocity;
  unsigned target_lane{1};
};

#endif