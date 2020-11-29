#include "planning.h"

#include <unordered_map>

using std::string;
using std::vector;

static unsigned change_lane_timer{0};
namespace {
static std::map<string, int> lane_direction = {
    {"KL", 0}, {"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};
}  // namespace

Planner::Planner(const VehicleState& ego, TrajectoryGenerator& gen,
                 const Prediction& predictions, const MapWaypoints& map,
                 const Trajectory& previous_trajectory)
    : ego(ego),
      trajectory_generator(gen),
      map(map),
      predictions(predictions),
      state("KL"),
      previous_trajectory(previous_trajectory) {}

vector<string> Planner::successor_states() {
  vector<string> possible_states;

  possible_states.push_back(state);
  change_lane_timer++;
  // std::cout << "change_time" << change_lane_timer << std::endl;
  if (state.compare("KL") == 0 && (change_lane_timer > 30)) {
    if (ego.left_lane_exists()) {
      possible_states.push_back("PLCL");
    }
    if ((ego.right_lane_exists())) {
      possible_states.push_back("PLCR");
    }
  } else if (state.compare("PLCL") == 0) {
    possible_states.push_back("KL");
    possible_states.push_back("LCL");
  } else if (state.compare("PLCR") == 0) {
    possible_states.push_back("KL");
    possible_states.push_back("LCR");
  } else if (state.compare("LCL") == 0) {
    change_lane_timer = 0;
    possible_states.push_back("KL");
  } else if (state.compare("LCR") == 0) {
    change_lane_timer = 0;
    possible_states.push_back("KL");
  }
  return possible_states;
}

double Planner::calculate_cost(const Trajectory& trajectory) {
  const auto INNEFICIENCY_WEIGHT = 10.0;
  // double cost{0.0};

  double c1 = 10 * cost_inneficient_lane(trajectory);
  double c2 = cost_distance_to_fastest_lane(trajectory);
  double c3 = 4 * cost_lane_change(trajectory);
  double c4 = 5 * cost_front_gap(trajectory);

  // std::cout << "c1:" << c1 << '\n';

  // std::cout << "c2:" << c2 << '\n';

  // std::cout << "c3:" << c3 << '\n';

  // std::cout << "c4:" << c4 << '\n';

  return c1 + c2 + c3 + c4;
}

double Planner::cost_inneficient_lane(const Trajectory& trajectory) {
  double intended_lane_speed =
      predictions.lane_speeds[trajectory.intended_lane];
  double end_lane_speed = predictions.lane_speeds[trajectory.end_lane];
  double current_speed = ego.speed;

  // std::cout << "end_lane_speed" << end_lane_speed << std::endl;
  // std::cout << "intended_lane_speed" << intended_lane_speed << std::endl;
  // std::cout << "current_speed" << current_speed << std::endl;

  // std::cout << intended_lane_speed << "+" << end_lane_speed << "/"
  //          << TARGET_EGO_SPEED << " = ";

  double xx = (intended_lane_speed + end_lane_speed) / TARGET_EGO_SPEED;

  // std::cout << xx << '\n';

  double cost = 2.0 - xx;
  cost = fmax(0.0, cost);
  return cost;
}

double Planner::cost_front_gap(const Trajectory& trajectory) {
  double gap_ahead =
      predictions.predicted_gaps[trajectory.intended_lane].distance_ahead;

  double cost = (150 - gap_ahead) / 150;

  cost = fmax(0.0, cost);
  return cost;
}

double Planner::cost_distance_to_fastest_lane(const Trajectory& trajectory) {
  double cost;

  unsigned fastest_lane = predictions.get_fastest_lane();
  double fastest_lane_speed = predictions.lane_speeds[fastest_lane];

  unsigned distance_to_fastest_lane =
      abs(fastest_lane - trajectory.intended_lane);
  // std::cout << "distance_to_fastest_lane:" << distance_to_fastest_lane <<
  // '\n';

  double speed_gain =
      fastest_lane_speed - predictions.lane_speeds[ego.get_lane()];

  if (distance_to_fastest_lane == 0) {
    cost = 0;
  } else {
    cost = distance_to_fastest_lane * speed_gain;
  }

  return cost;
}

double Planner::cost_lane_change(const Trajectory& trajectory) {
  // std::cout << "end_lane:" << trajectory.end_lane
  //          << " intended:" << trajectory.intended_lane << '\n';
  if (trajectory.end_lane == trajectory.intended_lane) {
    // std::cout << "DIFFERENT" << '\n';
    return 0.0;
  } else {
    // std::cout << "EQUAL" << '\n';

    return 1.0;
  }
}

Trajectory Planner::get_trajectory() {
  vector<string> states = successor_states();

  state_costs.clear();
  state_trajectories.clear();

  for (auto& candidate_state : states) {
    // std::cout << "\n\nplanning:" << candidate_state << std::endl;
    auto trajectory = plan_trajectory(candidate_state);
    std::cout << "candidate:" << candidate_state << '\n';

    if (trajectory.size() != 0) {
      double cost = calculate_cost(trajectory);
      // std::cout << "cost:" << cost << std::endl;

      state_trajectories[candidate_state] = trajectory;
      state_costs[candidate_state] = cost;
    }
  }

  // for (auto it : state_costs) {
  // std::cout << "STATE:" << it.first << " COST:" << it.second << std::endl;
  //}

  auto next_state = std::min_element(
      state_costs.begin(), state_costs.end(),
      [](const std::pair<string, double>& a,
         const std::pair<string, double>& b) { return a.second < b.second; });

  state = next_state->first;

  // std::cout << "NEXT_STATE:" << next_state->first << std::endl;

  last_ego = ego;
  return state_trajectories[state];
}

Trajectory Planner::plan_trajectory(const std::string& candidate_state) {
  Trajectory trajectory;

  unsigned current_lane, intended_lane, end_lane;

  current_lane = ego.get_lane();

  // std::cout << "previous_trajectory_end_lane:" <<
  // previous_trajectory.end_lane
  //          << std::endl;

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

  // std::cout << "candidate_state:" << candidate_state << std::endl;

  // std::cout << "intended_lane:" << intended_lane << std::endl;
  // std::cout << "end_lane:" << end_lane << std::endl;

  if (end_lane < 0 || end_lane > 2 || intended_lane < 0 || intended_lane > 2) {
    return {};
  }

  trajectory_generator.anchors_init();

  // std::cout << "candidate:" << candidate_state << " i[" << intended_lane
  //          << "] end[" << end_lane << "]\n";

  trajectory = trajectory_generator.generate_trajectory(intended_lane, end_lane,
                                                        candidate_state);
  return trajectory;
}

std::ostream& operator<<(std::ostream& os, const Planner& planner) {
  std::vector<std::string> all_states{"KL", "PLCL", "PLCR", "LCL", "LCR"};

  os << std::fixed << std::setprecision(2);
  os << "current_state[" << planner.state << "]\n";

  for (auto s : all_states) {
    double cost;
    if (planner.state_costs.find(s) != planner.state_costs.end()) {
      cost = planner.state_costs.at(s);
      os << s << " cost[" << cost << "]\n";
    } else {
      os << s << " cost["
         << "NA"
         << "]\n";
    }
  }

  return os;
}
