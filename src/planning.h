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