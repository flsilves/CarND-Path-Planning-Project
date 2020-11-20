#include "planning.h"

Planner::Planner(const VehicleState& ego, TrajectoryGenerator& gen,
                 const Prediction& predictions, const MapWaypoints& map)
    : ego(ego),
      trajectory_generator(gen),
      map(map),
      predictions(predictions),
      state(State::KeepLane) {}

Trajectory Planner::get_trajectory() {
  double front_speed = predictions.vehicle_close_ahead(
      trajectory_generator.previous_trajectory_.size(),
      trajectory_generator.previous_trajectory_.end_s, ego.get_lane(), ego.s);

  if (front_speed < target_velocity) {
    if (fabs(front_speed - target_velocity) > 1.0) {
      target_velocity -= .224;
    } else {
      target_velocity = front_speed;
    }
    //
  } else {
    target_velocity += .224;
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
}
