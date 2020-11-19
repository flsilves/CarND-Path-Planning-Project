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
constexpr double MPH_2_MPS = 0.44704;
constexpr auto DETECTED_VEHICLES{12u};
constexpr auto OBJECT_HISTORY_SIZE{10u};

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

  VehicleState(std::size_t id, double x, double y, double vx, double vy,
               double s, double d)
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

  bool evaluate_continuity(VehicleState next) {
    if (abs(next.speed - speed) > 3.0 || (next.x - x) < 0.0 ||
        abs(next.y - y) > 3.0) {
      std::cout << "next.speed[" << next.speed << "] speed[" << speed << ']';
      std::cout << "next.x[" << next.x << "] x[" << x << ']';
      std::cout << "next.y[" << next.y << "] y[" << y << ']';

      return false;
    }

    return true;
  }

  bool in_right_side_of_road() { return (d <= 12.0) && (d >= 0.0); }

 public:
  std::size_t id{1000};
  double x{0.0}, y{0.0};
  double vx{0.0}, vy{0.0};
  double d{0.0}, s{0.0};
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

  void trim(std::size_t new_size) {
    if (new_size > x.size()) {
      x.resize(new_size);
      y.resize(new_size);
    }
  }

  bool empty() const { return x.empty(); }
  std::size_t size() const { return x.size(); }

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

class ObjectHistory {
 public:
  ObjectHistory() = default;

  void update(json sensor_fusion) {
    for (auto& sensor_data : sensor_fusion) {
      std::size_t id = sensor_data[0];
      double x = sensor_data[1];
      double y = sensor_data[2];
      double vx = sensor_data[3];
      double vy = sensor_data[4];
      double s = sensor_data[5];
      double d = sensor_data[6];

      VehicleState detected_car{id, x, y, vx / 0.44704, vy / 0.44704, s, d};

      auto& vehicle_history = history[id];

      if (detected_car.in_right_side_of_road()) {
        if (not vehicle_history.empty()) {
          auto& previous_state = vehicle_history.back();

          bool continuous = previous_state.evaluate_continuity(detected_car);

          if (not continuous) {
            std::cout << "ID[" << id << "] not continous!" << '\n';
            vehicle_history.clear();
          }
        }

        vehicle_history.push_back(detected_car);

      } else {
        vehicle_history.clear();
      }

      if (vehicle_history.size() > OBJECT_HISTORY_SIZE) {
        vehicle_history.pop_front();
      }
    }
  }

  double vehicle_close_ahead(int steps_into_future, double ego_future_s,
                             int ego_lane) {
    for (auto& vehicle_history : history) {
      if (vehicle_history.empty()) {
        continue;
      }

      auto vehicle = vehicle_history.back();
      if (vehicle.get_lane() == ego_lane) {
        double check_car_s =
            vehicle.s + vehicle.speed * steps_into_future *
                            0.02;  // TODO use parameter <time per point>

        if (check_car_s > ego_future_s) {  // TODO: use parameter gap
          std::cout << "Vehicle in front at:" << (check_car_s - ego_future_s)
                    << "v:" << vehicle.speed << std::endl;
          if (check_car_s - ego_future_s < 60) {
            return vehicle.speed;
          }
        }
      }
    }
    return 0.0;
  }

 public:
  std::array<std::deque<VehicleState>, DETECTED_VEHICLES> history;
};

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(const Path& previous_path, const VehicleState& ego,
                      const MapWaypoints& map)
      : previous_path_(previous_path), ego_(ego), map(map) {}

  Path generate_trajectory(double target_velocity, int target_lane) {
    const double anchor_spacement = 50.0;
    const unsigned extra_anchors = 2;
    const unsigned path_length = 50;

    anchors_init();
    anchors_trim();
    anchors_add(anchor_spacement, extra_anchors, target_lane);
    anchors_recenter();

    spline.set_points(anchors_x, anchors_y);

    auto generated_path = previous_path_;

    generated_path.trim(10);

    auto missing_points = path_length - generated_path.size();

    auto x_increment = get_x_increment(target_velocity);

    double x{0}, y{0};

    for (int i = 1; i <= missing_points; ++i) {
      x = x + x_increment;
      y = spline(x);

      generated_path.x.push_back(x * cos(ref_yaw) - y * sin(ref_yaw) + ref_x);
      generated_path.y.push_back(x * sin(ref_yaw) + y * cos(ref_yaw) + ref_y);
    }
    return generated_path;
  }

 private:
  double get_x_increment(double target_velocity) {
    constexpr double time_per_point = .02;

    double horizon_x = 30.0;
    double horizon_y = spline(horizon_x);
    double target_dist =
        sqrt((horizon_x) * (horizon_x) + (horizon_y) * (horizon_y));
    double N = target_dist / (time_per_point * target_velocity * MPH_2_MPS);
    return horizon_x / N;
  }

  void anchors_init() {
    anchors_x.clear();
    anchors_y.clear();
    ref_yaw = deg2rad(ego_.yaw);

    std::size_t prev_size = previous_path_.size();

    if (prev_size < 2) {
      anchors_x.push_back(ego_.x - cos(ref_yaw));
      anchors_x.push_back(ego_.x);

      anchors_y.push_back(ego_.y - sin(ref_yaw));
      anchors_y.push_back(ego_.y);
    } else {
      ref_x = previous_path_.x.end()[-1];
      ref_y = previous_path_.y.end()[-1];

      double ref_x_prev = previous_path_.x.end()[-2];
      double ref_y_prev = previous_path_.y.end()[-2];

      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

      // std::cout << "ref_yaw:" << rad2deg(ref_yaw) << '\n';

      anchors_x.push_back(ref_x_prev);
      anchors_x.push_back(ref_x);

      anchors_y.push_back(ref_y_prev);
      anchors_y.push_back(ref_y);
    }
    ref_x = anchors_x.at(1);
    ref_y = anchors_y.at(1);
  }

  void anchors_add(double anchor_spacement, unsigned extra_anchors,
                   int target_lane) {
    for (auto i = 1u; i <= extra_anchors; ++i) {
      auto next_anchor = getXY(ego_.s + (i) * (anchor_spacement),
                               (2 + 4 * target_lane), map.s, map.x, map.y);

      anchors_x.emplace_back(next_anchor[0]);
      anchors_y.emplace_back(next_anchor[1]);
    }
  }

  void anchors_recenter() {
    for (int i = 0; i < anchors_x.size(); ++i) {
      // shift car reference angle to 0 degrees
      double shift_x = anchors_x[i] - ref_x;
      double shift_y = anchors_y[i] - ref_y;

      anchors_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
      anchors_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }
  }

  void anchors_trim() {
    anchors_x.resize(2);
    anchors_y.resize(2);
  }

 public:
  tk::spline spline;
  std::vector<double> anchors_x, anchors_y;
  double ref_yaw, ref_x, ref_y;
  const Path& previous_path_;
  const VehicleState& ego_;
  const MapWaypoints map;
};

std::ostream& operator<<(std::ostream& os, const ObjectHistory& rhs) {
  for (auto& vehicle_history : rhs.history) {
    if (not vehicle_history.empty()) {
      os << vehicle_history.back() << '\n';
    }
  }
  return os;
}

int main() {
  uWS::Hub h;

  MapWaypoints map(parameters::map_file, parameters::max_s);
  double target_velocity = 0;  // mph
  int lane = 1;

  VehicleState ego;
  ObjectHistory object_history;
  Path previous_path("previous_path");
  TrajectoryGenerator trajectory_generator(previous_path, ego, map);

  h.onMessage([&map, &target_velocity, &lane, &previous_path, &ego,
               &trajectory_generator,
               &object_history](uWS::WebSocket<uWS::SERVER> ws, char* data,
                                std::size_t length, uWS::OpCode opCode) {
    if (valid_socket_message(length, data)) {
      auto s = hasData(data);

      if (not s.empty()) {
        auto j = json::parse(s);
        auto event = j[0].get<string>();

        if (event == "telemetry") {
          auto telemetry_data = j[1];

          VehicleState prev_ego{ego};

          ego.update(telemetry_data);
          previous_path.update(telemetry_data);

          double delta_x = fabs(ego.x - prev_ego.x);
          double delta_y = fabs(ego.y - prev_ego.y);

          double distance_traveled =
              sqrt(delta_x * delta_x + delta_y * delta_y);

          double average_speed = (ego.speed / 2 + prev_ego.speed / 2);

          double time = distance_traveled / average_speed;

          std::cout << "time_step[" << time << "] average_speed["
                    << average_speed << "] dist_step[" << distance_traveled
                    << "]" << std::endl;

          object_history.update(telemetry_data["sensor_fusion"]);

          double front_speed = object_history.vehicle_close_ahead(
              previous_path.size(), previous_path.end_s, ego.get_lane());

          std::cout << object_history << '\n';

          if (front_speed > 1.0 && front_speed < target_velocity) {
            if (front_speed < target_velocity) {
              target_velocity -= .224;
            } else {
              target_velocity += .224;
            }

          } else if (target_velocity < 49.5) {
            target_velocity += .224 * 2;
          }

          std::cout << ego << '\n';

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
                         char* message, std::size_t length) {
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