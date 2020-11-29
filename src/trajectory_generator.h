#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <spline.h>

#include "map.h"
#include "parameters.h"
#include "prediction.h"
#include "trajectory.h"
#include "vehicle.h"
class TrajectoryGenerator {
 public:
  TrajectoryGenerator(const Trajectory& previous_trajectory,
                      const VehicleState& ego, const MapWaypoints& map,
                      const Prediction& predictions);

  Trajectory generate_trajectory(unsigned intended_lane, unsigned end_lane, std::string state);

  void anchors_init();

 private:
  double calculate_velocity(double previous_velocity);
  std::vector<double> calculate_next_point(double starting_x,
                                           double target_velocity,
                                           double target_distance);

  double get_next_point_velocity(double last_planned_velocity,
                                 double target_velocity);

  double get_target_distance();
  double get_keep_lane_velocity(Trajectory& new_trajectory);
  double get_last_planned_velocity();

  double prepare_lane_change_velocity(Trajectory& new_trajectory,
                                      unsigned intented_lane);

  double lane_change_velocity(Trajectory& new_trajectory);

  void anchors_add(double anchor_spacement, unsigned extra_anchors,
                   int target_lane);

  void anchors_recenter();
  void anchors_trim();
  bool validate_trajectory(Trajectory& trajectory, double ego_s);

  void fill_trajectory_points(Trajectory& trajectory, double target_velocity,
                              unsigned end_lane);

 private:
  std::vector<double> anchors_x, anchors_y;
  double ref_yaw, ref_x, ref_y;
  const Trajectory& previous_trajectory;
  const VehicleState& ego;
  const MapWaypoints map;
  const Prediction& predictions;
};

#endif