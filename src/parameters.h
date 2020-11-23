#ifndef PARAMETERS_H
#define PARAMETERS_H

constexpr double MPH_2_MPS{0.44704};
constexpr auto MAX_LANE_SPEED{50.0};
constexpr auto TARGET_EGO_SPEED{49.5};

constexpr auto MAX_ACCELERATION{0.8}; // m/s per point


constexpr auto NUMBER_OF_LANES{3u};
constexpr auto MAP_MAX_S{6945.554};
const std::string MAP_FILEPATH("data/highway_map.csv");

constexpr auto DETECTED_VEHICLES{12u};
constexpr auto OBJECT_HISTORY_SIZE{10u};
constexpr auto SENSOR_RANGE_METERS{200.0};

namespace parameters {


}  // namespace parameters

#endif