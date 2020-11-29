#ifndef PLANNING_H
#define PLANNING_H

#include <unordered_map>

#include "map.h"
#include "parameters.h"
#include "prediction.h"
#include "trajectory.h"
#include "trajectory_generator.h"
#include "vehicle.h"

class Planner {
 public:
  Planner(const VehicleState& ego, TrajectoryGenerator& gen,
          const Prediction& predictions, const MapWaypoints& map,
          const Trajectory& previous_trajectory);
  Planner step();

  Trajectory get_trajectory();
  double cost_inneficient_lane(const Trajectory& trajectory);
  double cost_distance_to_fastest_lane(const Trajectory& trajectory);
  double cost_front_gap(const Trajectory& trajectory);
  double cost_lane_change(const Trajectory& trajectory);

  double calculate_cost(const Trajectory& trajectory);
  Trajectory plan_trajectory(const std::string& candidate_state);

  std::vector<std::string> successor_states();

 public:
  std::string state{std::string("KL")};
  std::unordered_map<std::string, Trajectory> state_trajectories;
  std::unordered_map<std::string, double> state_costs;
  TrajectoryGenerator& trajectory_generator;
  const VehicleState& ego;
  const MapWaypoints& map;
  const Prediction& predictions;
  const Trajectory& previous_trajectory;
};

std::ostream& operator<<(std::ostream& os, const Planner& planner);

#endif