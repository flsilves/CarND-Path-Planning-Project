#include "planning.h"

#include <map>

using std::string;
using std::vector;
namespace {
static std::map<string, int> lane_direction = {
    {"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};
}  // namespace

Planner::Planner(const VehicleState& ego, TrajectoryGenerator& gen,
                 const Prediction& predictions, const MapWaypoints& map)
    : ego(ego),
      trajectory_generator(gen),
      map(map),
      predictions(predictions),
      state("KL") {}

vector<string> Planner::successor_states() {
  vector<string> states;
  states.push_back("KL");
  if (state.compare("KL") == 0) {
    if (ego.left_lane_exists()) {
      states.push_back("PLCL");
    }
    if ((ego.left_lane_exists())) {
      states.push_back("PLCR");
    }
  } else if (state.compare("PLCL") == 0) {
    states.push_back("PLCL");
    states.push_back("LCL");
  } else if (state.compare("PLCR") == 0) {
    states.push_back("PLCR");
    states.push_back("LCR");
  }
  return states;
}

Trajectory Planner::get_trajectory() {
  vector<string> states = successor_states();
  vector<float> costs;
  vector<Trajectory> final_trajectories;

  for (auto& candidate_state : states) {
    auto trajectory = plan_trajectory(candidate_state);
    if (trajectory.size() != 0) {
      // costs.push_back(calculate_cost(trajectory));
      costs.push_back(10.0);
      final_trajectories.push_back(trajectory);
    }
  }

  auto it_best_const = min_element(begin(costs), end(costs));
  unsigned best_idx = distance(begin(costs), it_best_const);

  return final_trajectories[best_idx];
}

Trajectory Planner::plan_trajectory(const std::string& candidate_state) {
  Trajectory trajectory;

  trajectory_generator.anchors_init();

  unsigned intended_lane = ego.get_lane() + lane_direction[candidate_state];

  if (state.compare("KL") == 0) {
    trajectory = trajectory_generator.generate_trajectory(intended_lane);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = trajectory_generator.generate_trajectory(intended_lane);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = trajectory_generator.generate_trajectory(intended_lane);
  }

  return trajectory;
}

/* Trajectory Planner::get_trajectory() {
  double front_speed = predictions.vehicle_close_ahead(
      trajectory_generator.previous_trajectory_.size(),
      trajectory_generator.previous_trajectory_.end_s, ego.get_lane(), ego.s);

  if (front_speed < target_velocity) {
    if (fabs(front_speed - target_velocity) > 1.0) {
      target_velocity -= MAX_ACCELERATION;
    } else {
      target_velocity = front_speed;
    }
    //
  } else {
    target_velocity += MAX_ACCELERATION;
  }

  if (state == State::ChangeLeft) {
    if (ego.get_lane() == target_lane) {
      state = State::KeepLane;
    }
  } else {
    if (target_velocity > 20.0) {
      unsigned ideal_lane = predictions.get_fastest_lane();
      std::cout << "\n****IDEAL:" << ideal_lane << std::endl;

      if (ideal_lane != ego.get_lane() &&
          fabs(ideal_lane - ego.get_lane()) < 1.1) {
        if (predictions.predicted_gaps[target_lane].distance_behind > 30.0 &&
            predictions.predicted_gaps[target_lane].distance_ahead > 30.0) {
          state = State::ChangeLeft;
          target_lane = ideal_lane;
        }
      }
    }
  }
  return trajectory_generator.generate_trajectory(target_velocity, target_lane);
} */
