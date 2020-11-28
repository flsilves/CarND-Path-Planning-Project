#include "planning.h"

#include <unordered_map>

using std::string;
using std::vector;
namespace {
static std::map<string, int> lane_direction = {
    {"KL", 0}, {"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};
}  // namespace

Planner::Planner(const VehicleState& ego, TrajectoryGenerator& gen,
                 const Prediction& predictions, const MapWaypoints& map)
    : ego(ego),
      trajectory_generator(gen),
      map(map),
      predictions(predictions),
      state("KL") {}

vector<string> Planner::successor_states() {
  vector<string> possible_states;
  possible_states.push_back("KL");
  if (state.compare("KL") == 0) {
    if (ego.left_lane_exists()) {
      possible_states.push_back("PLCL");
    }
    if ((ego.left_lane_exists())) {
      possible_states.push_back("PLCR");
    }
  } else if (state.compare("PLCL") == 0) {
    possible_states.push_back("PLCL");
    possible_states.push_back("LCL");
  } else if (state.compare("PLCR") == 0) {
    possible_states.push_back("PLCR");
    possible_states.push_back("LCR");
  }
  return possible_states;
}

double Planner::calculate_cost(const Trajectory& trajectory) {
  const auto INNEFICIENCY_WEIGHT = 10.0;
  double cost{0.0};

  cost += cost_inneficient_lane(trajectory);

  return cost;
}

double Planner::cost_inneficient_lane(const Trajectory& trajectory) {
  double intended_lane_speed =
      predictions.lane_speeds[trajectory.intended_lane];
  double end_lane_speed = predictions.lane_speeds[trajectory.end_lane];
  double current_speed = ego.speed;

  std::cout << "end_lane_speed" << end_lane_speed << std::endl;
  std::cout << "intended_lane_speed" << intended_lane_speed << std::endl;
  std::cout << "current_speed" << current_speed << std::endl;

  double cost = 2.0 - (intended_lane_speed + end_lane_speed) / current_speed;
  cost = fmax(0.0, cost);
  return cost;
}

Trajectory Planner::get_trajectory() {
  vector<string> states = successor_states();

  std::unordered_map<string, Trajectory> state_trajectories;
  std::unordered_map<string, double> state_costs;

  // return plan_trajectory("KL");

  for (auto& candidate_state : states) {
    std::cout << "\n\nplanning:" << candidate_state << std::endl;
    auto trajectory = plan_trajectory(candidate_state);
    if (trajectory.size() != 0) {
      double cost = calculate_cost(trajectory);
      std::cout << "cost:" << cost << std::endl;

      state_trajectories[candidate_state] = trajectory;
      state_costs[candidate_state] = cost;
    }
  }

  for (auto it : state_costs) {
    std::cout << "STATE:" << it.first << " COST:" << it.second << std::endl;
  }

  auto next_state = std::min_element(
      state_costs.begin(), state_costs.end(),
      [](const std::pair<string, double>& a,
         const std::pair<string, double>& b) { return a.second < b.second; });

  std::cout << "NEXT_STATE:" << next_state->first << std::endl;

  state = next_state->first;

  return state_trajectories[state];
}

Trajectory Planner::plan_trajectory(const std::string& candidate_state) {
  Trajectory trajectory;

  unsigned current_lane, intended_lane, end_lane;

  current_lane = ego.get_lane();
  intended_lane = current_lane + lane_direction[candidate_state];

  if (candidate_state.compare("KL") == 0 ||
      candidate_state.compare("PLCL") == 0 ||
      candidate_state.compare("PLCR") == 0) {
    end_lane = current_lane;
    // std::cout << "state1" << candidate_state << std::endl;
    // std::cout << "intendend" << intended_lane << std::endl;
    // std::cout << "end" << end_lane << std::endl;

  } else if (candidate_state.compare("LCL") == 0 ||
             candidate_state.compare("LCR") == 0) {
    end_lane = intended_lane;
    // std::cout << "state2" << candidate_state << std::endl;
    // std::cout << "intendend" << intended_lane << std::endl;

    // std::cout << "end" << end_lane << std::endl;
  }

  trajectory_generator.anchors_init();
  trajectory =
      trajectory_generator.generate_trajectory(intended_lane, end_lane);
  return trajectory;
}

std::ostream& operator<<(std::ostream& os, const Planner& planner) {
  os << std::fixed << std::setprecision(2);
  os << "state[" << planner.state << "] ";
  // os << "target_lane[" << planner.target_lane << "] ";
  return os;
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

/* Trajectory Planner::cost_inneficient_lane(const Trajectory& trajectory)

    auto current_speed = ego.speed;
auto fpredi

    double cost =
        2.0 - (intended_lane_speed + endpoint_lane_speed) / current_speed;

int current_speed = parameters.desired_speed;
int intended_lane = trajectory.characteristics.intended_lane_id;
int endpoint_lane = trajectory.characteristics.endpoint_lane_id;

double intended_lane_speed = predictions.lanes[intended_lane].speed;
double endpoint_lane_speed = predictions.lanes[endpoint_lane].speed;
double cost = 2.0 - (intended_lane_speed + endpoint_lane_speed) / current_speed;
cost = fmax(0.0, cost);

return cost * kFunctionWeight;
}

private:
double kFunctionWeight{15};
}
;

class PreferEmptyLaneCostFunction : public CostFunction {
 public:
  double getCost(const Trajectory& trajectory,
                 const PredictionData& predictions, const EgoStatus&,
                 const Parameters& parameters) override {
    int intended_lane_id = trajectory.characteristics.intended_lane_id;
    auto& lane = predictions.lanes[intended_lane_id];

    double d_max = parameters.cost_empty_lane_dmax;
    double c_max = parameters.cost_empty_lane_cmax;

    double cost = 0;
    if (lane.has_vehicle_ahead and lane.vehicle_ahead.distance < d_max) {
      double distance = lane.vehicle_ahead.distance;
      cost = c_max * (1.0 - distance / d_max);
    }
    // std::cout << "[Cost:EmptyLane]" << trajectory.characteristics.action
    //           << ", ilane: " << intended_lane_id
    //           << ", c: " << cost * kFunctionWeight << std::endl;

    return cost * kFunctionWeight;
  }

 private:
  double kFunctionWeight{10};
}; */