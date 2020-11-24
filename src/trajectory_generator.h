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
  TrajectoryGenerator(Trajectory& previous_trajectory, const VehicleState& ego,
                      const MapWaypoints& map, const Prediction& predictions);

  Trajectory generate_trajectory(unsigned end_lane, unsigned intended_lane);

  void anchors_init();

 private:
  double calculate_velocity(double previous_velocity);
  std::vector<double> calculate_next_point(double starting_x,
                                           double target_velocity,
                                           double target_distance);

  double get_next_point_velocity(double last_planned_velocity,
                                 double target_velocity);

  double get_target_distance();

  double get_last_planned_velocity();
  void anchors_add(double anchor_spacement, unsigned extra_anchors,
                   int target_lane);

  void anchors_recenter();
  void anchors_trim();

  void fill_trajectory_points(Trajectory& trajectory, double target_velocity,
                              unsigned end_lane);

 public:
  std::vector<double> anchors_x, anchors_y;
  double ref_yaw, ref_x, ref_y;
  Trajectory& previous_trajectory;
  const VehicleState& ego;
  const MapWaypoints map;
  const Prediction& predictions;
};

#endif