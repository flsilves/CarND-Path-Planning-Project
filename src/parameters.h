#ifndef PARAMETERS_H
#define PARAMETERS_H

constexpr double MPH_2_MPS{0.44704};

// MAP
constexpr auto NUMBER_OF_LANES{3u};
constexpr auto LANE_WIDTH{4u};
constexpr auto MAP_MAX_S{6945.554};

const std::string MAP_FILEPATH("../data/highway_map.csv");

// Trajectory generation
constexpr auto TARGET_EGO_SPEED{49.0};
constexpr auto MAX_ACCELERATION{.2}; // m/s per point
constexpr double TIME_PER_POINT = .02;
constexpr double ANCHOR_HORIZON_DISTANCE{20.0};
constexpr double KEEP_DISTANCE_TIME{0.7};
constexpr auto PATH_LENGTH{50u};

// Prediction / Sensor fusion
constexpr auto N_DETECTED_VEHICLES{12u};
constexpr auto SENSOR_RANGE_METERS{200.0};

#endif