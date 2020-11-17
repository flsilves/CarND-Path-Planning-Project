#include <uWS/uWS.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include <spline.h>

#include <json.hpp>

#include "helpers.h"
#include "map.h"
#include "parameters.h"

// NOTES
// There's cars changing lanes in the simulation

using nlohmann::json;
using std::string;
using std::vector;

static std::ofstream telemetry_log("./telemetry.log", std::ios::app);

class VehicleState {
 public:
  VehicleState() = default;

  void update(json telemetry_data) {
    x = telemetry_data["x"];
    y = telemetry_data["y"];
    s = telemetry_data["s"];
    d = telemetry_data["d"];
    yaw = telemetry_data["yaw"];      // degrees [positive in ccw direction]
    speed = telemetry_data["speed"];  // units ??
  }

  VehicleState(int id, double x, double y, double vx, double vy, double s,
               double d)
      : id(id), x(x), y(y), vx(vx), vy(vy), s(s), d(d) {
    speed = sqrt(vx * vx + vy * vy);
    yaw = rad2deg(atan2(vy, vx));
    if (yaw < 0) {
      yaw += 360;
    }
  }

  int get_lane(double lane_width = 4.0) const {
    return fmax(fmin(2, floor(d / lane_width)), 0);
  }

 public:
  int id{-1};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double d{0.0};
  double s{0.0};
  double yaw{0.0};
  double speed{0.0};
};

class Path {
 public:
  // Path() = default;

  Path(string path_name = "Path") { name = path_name; }

  Path(const Path& other, const std::string& path_name = "Path")
      : name(path_name),
        x(other.x),
        y(other.y),
        end_s(other.end_s),
        end_d(other.end_d) {}

  void update(json telemetry_data) {
    x = telemetry_data["previous_path_x"].get<vector<double>>();
    y = telemetry_data["previous_path_y"].get<vector<double>>();
    end_s = telemetry_data["end_path_s"];
    end_d = telemetry_data["end_path_d"];

    if (x.size() != y.size()) {
      throw std::runtime_error("<Path::ctor>: x and y have different lengths");
    }
  }

  bool empty() const { return x.empty(); }
  size_t size() const { return x.size(); }

 public:
  std::string name{""};
  vector<double> x, y;
  double end_s{0.0}, end_d{0.0};
};

std::ostream& operator<<(std::ostream& os, const Path& path) {
  os << std::fixed << std::setprecision(2);
  os << "[" << path.name << "] ";
  os << "x[" << path.x.front() << " -> " << path.x.back() << "] ";
  os << "y[" << path.y.front() << " -> " << path.y.back() << "] ";
  os << "end_s[" << path.end_s << "] ";
  os << "end_d[" << path.end_d << "] ";
  os << "size[" << path.size() << ']';
  return os;
}

std::ostream& operator<<(std::ostream& os, const VehicleState& vehicle) {
  os << std::fixed << std::setprecision(2);
  os << "id[" << vehicle.id << "] ";
  os << "x[" << vehicle.x << "] ";
  os << "y[" << vehicle.y << "] ";
  os << "s[" << vehicle.s << "] ";
  os << "d[" << vehicle.d << "] ";
  os << "lane[" << vehicle.get_lane() << "] ";
  os << "yaw[" << vehicle.yaw << "] ";
  os << "speed[" << vehicle.speed << ']';
  return os;
}

class SensorData {
 public:
  SensorData(json sensor_fusion) {
    for (auto& sensor_data : sensor_fusion) {
      auto id = sensor_data[0];
      auto x = sensor_data[1];
      auto y = sensor_data[2];
      auto vx = sensor_data[3];
      auto vy = sensor_data[4];
      auto s = sensor_data[5];
      auto d = sensor_data[6];
      surrounding_vehicles.emplace_back(id, x, y, vx, vy, s, d);
    }
  }

  bool vehicle_close_ahead(int steps_into_future, double ego_future_s,
                           int ego_lane) {
    for (auto& vehicle : surrounding_vehicles) {
      // std::cout << "v_lane[" << vehicle.get_lane() << "] ego_lane[" <<
      // ego_lane
      //          << "]" << std::endl;
      if (vehicle.get_lane() == ego_lane) {
        double check_car_s =
            vehicle.s + vehicle.speed * steps_into_future *
                            0.02;  // TODO use parameter <time per point>

        if ((check_car_s > ego_future_s) &&
            (check_car_s - ego_future_s < 30)) {  // TODO: use parameter gap
          return true;
        }
      }
    }
    return false;
  }

 public:
  std::vector<VehicleState> surrounding_vehicles;
};

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(const Path& previous_path, const VehicleState& ego_state,
                      const MapWaypoints& map)
      : previous_path_(std::ref(previous_path)),
        ego_state_(std::ref(ego_state)),
        map(map) {}

  void update_input(const Path& previous_path, const VehicleState& ego_state) {
    previous_path_ = std::ref(previous_path);
    ego_state_ = std::ref(ego_state);
  }

  Path generate_trajectory(double target_velocity, int target_lane) {
    vector<double> anchor_x;
    vector<double> anchor_y;
    int lane = 0;
    double ref_x = ego_state_.get().x;
    double ref_y = ego_state_.get().y;
    double ref_yaw = deg2rad(ego_state_.get().yaw);

    size_t prev_size = previous_path_.get().size();

    // Initialization when there's no previous points
    if (prev_size < 2) {
      // Extrapolate the previous position based on current yaw angle
      double prev_car_x = ego_state_.get().x - cos(ref_yaw);
      double prev_car_y = ego_state_.get().y - sin(ref_yaw);

      // std::cout << "Ego.x:" << prev_car_x << " -> " << ego_state_.x
      //          << std::endl;
      // std::cout << "Ego.y:" << prev_car_y << " -> " << ego_state_.y
      //          << std::endl;

      anchor_x.push_back(prev_car_x);
      anchor_x.push_back(ego_state_.get().x);

      anchor_y.push_back(prev_car_y);
      anchor_y.push_back(ego_state_.get().y);
    } else {
      ref_x = previous_path_.get().x.end()[-1];
      ref_y = previous_path_.get().y.end()[-1];

      double ref_x_prev = previous_path_.get().x.end()[-2];
      double ref_y_prev = previous_path_.get().y.end()[-2];

      ref_yaw = atan2(ref_y - ref_y_prev,
                      ref_x - ref_x_prev);  // where is this used?? Why do we
                                            // need it? can't we use just yaw?

      // std::cout << "ref_yaw:" << rad2deg(ref_yaw) << '\n';

      anchor_x.push_back(ref_x_prev);
      anchor_x.push_back(ref_x);

      anchor_y.push_back(ref_y_prev);
      anchor_y.push_back(ref_y);
    }

    // In Freenet add evenly 30 spaced points ahead of the starting
    // reference
    vector<double> next_wp0 = getXY(ego_state_.get().s + 50,
                                    (2 + 4 * target_lane), map.s, map.x, map.y);

    vector<double> next_wp1 = getXY(ego_state_.get().s + 100,
                                    (2 + 4 * target_lane), map.s, map.x, map.y);

    vector<double> next_wp2 = getXY(ego_state_.get().s + 150,
                                    (2 + 4 * target_lane), map.s, map.x, map.y);

    anchor_x.push_back(next_wp0[0]);
    anchor_x.push_back(next_wp1[0]);
    anchor_x.push_back(next_wp2[0]);

    anchor_y.push_back(next_wp0[1]);
    anchor_y.push_back(next_wp1[1]);
    anchor_y.push_back(next_wp2[1]);

    for (int i = 0; i < anchor_x.size(); ++i) {
      // shift car reference angle to 0 degrees
      double shift_x = anchor_x[i] - ref_x;
      double shift_y = anchor_y[i] - ref_y;

      anchor_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
      anchor_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    tk::spline spline;
    spline.set_points(anchor_x, anchor_y);

    // start with all of the previous path points from last frame
    generated_path = previous_path_.get();

    // Calculate how to break up spline points so that we travel at our
    // desired reference velocity
    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dist =
        sqrt((target_x) * (target_x) + (target_y) * (target_y));

    double x_add_on = 0;
    double N = target_dist / (.02 * target_velocity / 2.2369);

    // std::cout << "N:" << N << std::endl;

    // include acceleration for trajectory generation
    for (int i = 1; i <= 50 - prev_size; ++i) {
      // distance =  N (point) * 0.02
      // (second/point) * v (miles/second)
      double x_point = x_add_on + (target_x) / N;

      double y_point = spline(x_point);

      // std::cout << "Points: x[" << x_point << "] y[" << y_point << "]
      // "
      //          << std::endl;

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw) + ref_x;
      y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw) + ref_y;

      // std::cout << "Appending: x[" << x_point << "] y[" << y_point <<
      // "] "
      //          << std::endl;

      generated_path.x.push_back(x_point);
      generated_path.y.push_back(y_point);
    }
    return generated_path;
  }

  Path get_path() { return generated_path; }

 public:
  Path generated_path{"generated"};
  std::reference_wrapper<const Path> previous_path_;
  std::reference_wrapper<const VehicleState> ego_state_;
  MapWaypoints map;
};

std::ostream& operator<<(std::ostream& os, const SensorData& data) {
  for (auto& vehicle : data.surrounding_vehicles) {
    os << vehicle << '\n';
  }
  return os;
}

int main() {
  uWS::Hub h;

  MapWaypoints map(parameters::map_file, parameters::max_s);
  double target_velocity = 0;  // mph
  int lane = 1;

  VehicleState ego_state;
  Path previous_path("previous_path");
  TrajectoryGenerator trajectory_generator(previous_path, ego_state, map);

  h.onMessage([&map, &target_velocity, &lane, &previous_path, &ego_state,
               &trajectory_generator](uWS::WebSocket<uWS::SERVER> ws,
                                      char* data, size_t length,
                                      uWS::OpCode opCode) {
    if (valid_socket_message(length, data)) {
      auto s = hasData(data);

      if (not s.empty()) {
        auto j = json::parse(s);
        auto event = j[0].get<string>();

        if (event == "telemetry") {
          auto telemetry_data = j[1];

          ego_state.update(telemetry_data);
          previous_path.update(telemetry_data);

          SensorData sensor_fusion{telemetry_data["sensor_fusion"]};

          int prev_size = previous_path.size();

          bool too_close = sensor_fusion.vehicle_close_ahead(
              previous_path.size(), previous_path.end_s, ego_state.get_lane());

          std::cout << sensor_fusion << '\n';

          if (too_close && lane > 0) {
            lane = 0;
          }

          if (target_velocity < 49.5) {
            target_velocity += .224;
          }

          std::cout << ego_state << '\n';

          if (not previous_path.empty()) {
            std::cout << previous_path << '\n';
          }

          auto next_path =
              trajectory_generator.generate_trajectory(target_velocity, lane);
          std::cout << next_path << '\n' << std::endl;

          json msgJson;
          msgJson["next_x"] = next_path.x;
          msgJson["next_y"] = next_path.y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char* message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}