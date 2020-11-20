#ifndef PARAMETERS_H
#define PARAMETERS_H


constexpr double MPH_2_MPS = 0.44704;
constexpr auto DETECTED_VEHICLES{12u};
constexpr auto OBJECT_HISTORY_SIZE{10u};
constexpr auto NUMBER_OF_LANES{3u};
constexpr auto MAX_LANE_SPEED{50.0};
constexpr auto SENSOR_RANGE_METERS{200.0};
namespace parameters {

const std::string map_file("data/highway_map.csv");
constexpr auto max_s{6945.554};

}

#endif