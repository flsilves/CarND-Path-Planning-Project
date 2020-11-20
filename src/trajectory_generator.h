#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "map.h"
#include "vehicle.h"
#include "trajectory.h"
#include <spline.h>
#include "parameters.h"


class TrajectoryGenerator {
 public:
  TrajectoryGenerator(const Trajectory& previous_trajectory,
                      const VehicleState& ego, const MapWaypoints& map);

  Trajectory generate_trajectory(double target_velocity, int target_lane);

 private:
  double get_x_increment(double target_velocity);
  void anchors_init();
  void anchors_add(double anchor_spacement, unsigned extra_anchors,
                   int target_lane);

  void anchors_recenter();
  void anchors_trim();

 public:
  std::vector<double> anchors_x, anchors_y;
  double ref_yaw, ref_x, ref_y;
  const Trajectory& previous_trajectory_;
  const VehicleState& ego_;
  const MapWaypoints map;
};

#endif