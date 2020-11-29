# CarND-Path-Planning-Project

[image1]: ./data/fsm.png "state_machine"

## Overview

### Goal

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


## Execution

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
* [spline](http://kluge.in-chemnitz.de/opensource/spline/)  
  * Single header library 


### Simulation

Term3 Simulator which contains the Path Planning Project:
https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2


### Build

```bash
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
```

## Input Data
### Map

Is provided in `data/highway_map.txt` as a collection of waypoints. Each `[x,y,s,dx,dy]` values. `x` and `y` are the waypoint's map coordinate position, the `s` value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet `s` value, distance along the road, goes from `0` to `6945.554`.

### Telemetry

The simulator provides telemetry information in json format. There's always a list of 12 vehicles 

```
# Main car's localization Data (No Noise)
["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

# Previous path data given to the Planner
# Return the previous list but with processed points removed.
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator

# Previous path's end s and d values 
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

# Sensor Fusion Data: a list of all other car's attributes on the same side of the road. (No Noise)
["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 
```

## Model Documentation

### Prediction

The prediction module mainly identifies the immediate front and rear vehicles using the frenet coordinates, for all lanes. It also predicts where those vehicles will be at the end time of the planned trajectory, a linear motion model with the latest velocity of the vehicle is used for that purpose. These predictions are used to adjust the velocity and to validate generated trajectories for possible collisions.

Alongside the gap prediction, this module also evaluates the lane speeds based on the vehicles ahead of ego, over a certain horizon distance.

### Trajectory Generation

Most of the trajectories generated reuse the previous one, between each telemetry step there's usually 3 to 6 points consumed. In each step, the previous path is extended using splines. To ensure better results with the spline generation the coordinates `x/y` are shifted to the origin with the same orientation of the vehicle.

The spacement of each point of the spline is calculated based on the target velocity and the maximum acceleration allowed, instead of just using a fixed velocity each time a trajectory is extended or retriggered. 


### State Machine

The state machine is the same used from the lessons, without the `Ready` state:

![alt text][image1]

During each planning step, only trajectories for the next possible states are generated. In the end the one with the lowest cost is chosen and the FSM transitates to the correspondent state.

Depending on the state the trajectories are generated differently:

- `KL`: Keeps the current velocity of the lane, if there's a vehicle ahead it maintains a time gap. If there's a vehicle cutting-in it applies the maximum deceleration possible.
- `PLCL/PLCR`: Tries to adjust the speed to the intended lane (10% more), without compromising the gap safety of the current lane, similarly to `KL`.
- `LCL/LCR`: Generates the lane change maneuver, it discards the majority of the points from the previous trajectory to allow for a quick lane change. A validity check is performed to evaluate the gaps (present and future) from both ego lane and intended lane.


#### Cost functions:

Four cost functions were used:

- `inneficient_lane`- Penalizes trajectories that have slower end/intended lanes 
- `distance_to_fastest_lane`- Penalizes trajectories that are farther away from the fastest lane, in proportion to the speed difference.
- `front_gap`: Favors trajectories in lanes that have a larger gap in front.
-  `lane_change`: Penalizes trajectories that intend to perform lane changes.


### Source files

The source code is located in the `src/` directory:

- `main.cpp` - Provides/receives trajectories from simulator, executes the planner
- `planning.h`- Contains the state machine and the cost functions to trigger the state changes.
- `trajectory_generator.h`- Generates the candidate trajectories
- `prediction.h`- Predicts the immediate vehicles close to ego, evaluates the lane speeds.
- `vehicle.h`- Holds a vehicle representation
- `trajectory.h`- Holds a trajectory representation
- `vehicle.h`- Holds a map representation
- `parameters.h`- Several parameters used along all the other sources
- `helpers.h`- Utility functions, frenet conversion, distance calculation, etc.

## Shortcomings

- The trajectory generation method is the one used from the Q&A video from Udacity, which does not guarantee that the ego vehicle is always centered. 

- The state machine is not designed to do only one lane change at a time, for safety reasons would be better to do so.

- When changing lanes the trajectory can be invalidated when it's already in the middle of the maneuver, this causes sometimes swerving of the vehicle.

- Prediction doesn't use the past points from sensor fusion, to identify vehicles that are changing lanes. 

- Cost functions could be averaged over time to prevent rapid state changes. 
 
