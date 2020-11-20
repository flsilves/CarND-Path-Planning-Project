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
constexpr auto NUMBER_OF_LANES{3u};
constexpr auto MAX_LANE_SPEED{50.0};
constexpr auto SENSOR_RANGE_METERS{200.0};

static std::ofstream telemetry_log("./telemetry.log", std::ios::app);

class VehicleState {
 public:
  VehicleState() = default;

  VehicleState(const VehicleState& other) = default;

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
    return (abs(next.speed - speed) < 3.0) && (abs(next.x - x) < 5.0) &&
           (abs(next.y - y) < 5.0);
  }

  VehicleState get_prediction(double future_time, const vector<double>& map_x,
                              const vector<double>& map_y) {
    VehicleState prediction{*this};
    auto rad_yaw = deg2rad(yaw);
    prediction.x = x + speed * MPH_2_MPS * future_time * cos(rad_yaw);
    prediction.y = y + speed * MPH_2_MPS * future_time * sin(rad_yaw);

    auto v = getFrenet(prediction.x, prediction.y, rad_yaw, map_x, map_y);
    prediction.s = v[0];
    prediction.d = v[1];

    // std::cout << "ref" << *this << std::endl;
    // std::cout << "prediction:" << prediction << std::endl;
    return prediction;
  }

  bool in_right_side_of_road() { return (d <= 12.0) && (d >= 0.0); }

 public:
  std::size_t id{42};
  double x{0.0}, y{0.0};
  double vx{0.0}, vy{0.0};
  double d{0.0}, s{0.0};
  double yaw{0.0};
  double speed{0.0};
};

/* class Planner {

  public:

  //state
  //


};
 */
class Trajectory {
 public:
  // Trajectory() = default;

  Trajectory(string path_name = "Trajectory") { name = path_name; }

  Trajectory(const Trajectory& other,
             const std::string& path_name = "Trajectory")
      : name(path_name),
        x(other.x),
        y(other.y),
        end_s(other.end_s),
        end_d(other.end_d) {
    calculate_end_angle();
  }

  void update(json telemetry_data) {
    x = telemetry_data["previous_trajectory_x"].get<vector<double>>();
    y = telemetry_data["previous_trajectory_y"].get<vector<double>>();
    end_s = telemetry_data["end_path_s"];
    end_d = telemetry_data["end_path_d"];

    if (x.size() != y.size()) {
      throw std::runtime_error(
          "Trajectory::update(): x and y have different lengths");
    }
  }

  double calculate_end_angle() {
    if (x.size() > 2 && y.size() > 2) {
      double x2 = x.end()[-1];
      double y2 = y.end()[-1];
      double x1 = x.end()[-2];
      double y1 = y.end()[-2];
      end_angle = atan2(y2 - y1, x2 - x1);
    }
    return end_angle;
  }

  void calculate_end_frenet(const vector<double>& map_x,
                            const vector<double>& map_y) {
    double theta = calculate_end_angle();

    if (not x.empty() && not y.empty()) {
      auto v = getFrenet(x.back(), y.back(), theta, map_x, map_y);
      end_s = v[0];
      end_d = v[1];
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
  double end_angle{0.0};
  double end_s{0.0}, end_d{0.0};
};

std::ostream& operator<<(std::ostream& os, const Trajectory& path) {
  os << std::fixed << std::setprecision(2);
  // os << "[" << path.name << "] ";
  os << "x[" << path.x.front() << " -> " << path.x.back() << "] ";
  os << "y[" << path.y.front() << " -> " << path.y.back() << "] ";
  os << "end_s[" << path.end_s << "] ";
  os << "end_d[" << path.end_d << "] ";
  os << "size[" << path.size() << ']';
  return os;
}

std::ostream& operator<<(std::ostream& os, const VehicleState& vehicle) {
  os << std::fixed << std::setprecision(2);
  os << "id[" << std::setw(2) << vehicle.id << "] ";
  os << "lane[" << vehicle.get_lane() << "] ";
  os << "x[" << std::setw(7) << vehicle.x << "] ";
  os << "y[" << std::setw(7) << vehicle.y << "] ";
  os << "s[" << std::setw(7) << vehicle.s << "] ";
  os << "d[" << std::setw(5) << vehicle.d << "] ";
  os << "speed[" << std::setw(5) << vehicle.speed << "] ";
  os << "yaw[" << std::setw(6) << vehicle.yaw << ']';
  return os;
}

struct Gap {
  double distance_behind;
  double distance_ahead;
};
class Prediction {
 public:
  Prediction(const MapWaypoints& map) : map(map){};

  void update_object_history(json sensor_fusion) {
    for (auto& object_entry : sensor_fusion) {
      std::size_t id = object_entry[0];
      double x = object_entry[1];
      double y = object_entry[2];
      double vx = object_entry[3];
      double vy = object_entry[4];
      double s = object_entry[5];
      double d = object_entry[6];

      VehicleState vehicle_detected{id, x, y, vx / 0.44704, vy / 0.44704, s, d};

      auto& vehicle_history = history[id];

      if (vehicle_detected.in_right_side_of_road()) {
        if (not vehicle_history.empty()) {
          auto& previous_state = vehicle_history.back();

          bool continuous =
              previous_state.evaluate_continuity(vehicle_detected);

          if (not continuous) {
            // std::cout << "ID[" << id << "] not continous!" << '\n';
            vehicle_history.clear();
          }
        }

        vehicle_history.push_back(vehicle_detected);

      } else {
        vehicle_history.clear();
      }

      if (vehicle_history.size() > OBJECT_HISTORY_SIZE) {
        vehicle_history.pop_front();
      }
    }
  }

  double vehicle_close_ahead(int steps_into_future, double ego_future_s,
                             int ego_lane, double ego_s) {
    for (auto& vehicle_history : history) {
      if (vehicle_history.empty()) {
        continue;
      }

      auto vehicle = vehicle_history.back();
      if (vehicle.get_lane() == ego_lane && vehicle.s > ego_s) {
        double check_car_s =
            vehicle.s + vehicle.speed * steps_into_future *
                            0.02;  // TODO use parameter <time per point>

        if (check_car_s > ego_future_s) {  // TODO: use parameter gap
          // std::cout << "Vehicle in front s[" << vehicle.s << "] v["
          //          << vehicle.speed << "] s_gap[" << check_car_s << " - "
          //          << ego_future_s << " = " << (check_car_s - ego_future_s)
          //          << "] v[" << vehicle.speed << ']' << std::endl;
          if (check_car_s - ego_future_s < 60) {
            return vehicle.speed;
          }
        }
      }
    }
    return 49.5;
  }

  void reset_lane_speeds() {
    for (auto& lane_speed : lane_speeds) {
      lane_speed = MAX_LANE_SPEED;
    }
  }

  void reset_gaps() {
    for (auto& gap : predicted_gaps) {
      gap = Gap{SENSOR_RANGE_METERS, SENSOR_RANGE_METERS};
    }
  }

  void predict_gaps(VehicleState ego, double future_ego_s, double future_time) {
    reset_gaps();
    reset_lane_speeds();

    constexpr auto TRAFFIC_SPEED_HORIZON_DISTANCE{100.0};

    for (auto& vehicle_history : history) {
      if (vehicle_history.empty()) {
        continue;
      }

      auto& traffic_vehicle = vehicle_history.back();
      auto future_traffic_vehicle =
          traffic_vehicle.get_prediction(future_time, map.x, map.y);

      // std::cout << "traffic_vehicle:" << traffic_vehicle << '\n';
      // std::cout << "future_traffic_vehicle:" << future_traffic_vehicle <<
      // '\n';

      auto vehicle_lane = traffic_vehicle.get_lane();

      // Irrelevant to predict state of the cars that are behind in the same
      // lane
      if (vehicle_lane == ego.get_lane() && ego.s > traffic_vehicle.s) {
        continue;
      }

      auto delta_s = fabs(future_traffic_vehicle.s - future_ego_s);

      auto& lane_gap = predicted_gaps[vehicle_lane];
      auto& lane_speed = lane_speeds[vehicle_lane];

      if (future_traffic_vehicle.s > future_ego_s) {
        if (delta_s < lane_gap.distance_ahead) {
          lane_gap.distance_ahead = delta_s;
          // lane_speed = traffic_vehicle.speed;
        }
      } else {
        if (delta_s < lane_gap.distance_behind) {
          lane_gap.distance_behind = delta_s;
        }
      }

      std::cout << "vehicle.s[" << traffic_vehicle.s << "] ego.s[" << ego.s
                << "]";
      if ((traffic_vehicle.s > ego.s) &&
          (traffic_vehicle.s < (ego.s + TRAFFIC_SPEED_HORIZON_DISTANCE))) {
        lane_speed = fmin(lane_speed, traffic_vehicle.speed);
      }
    }
  }

 public:
  std::array<std::deque<VehicleState>, DETECTED_VEHICLES> history;
  std::array<Gap, NUMBER_OF_LANES> predicted_gaps;
  std::array<double, NUMBER_OF_LANES> lane_speeds;
  const MapWaypoints map;
};

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(const Trajectory& previous_trajectory,
                      const VehicleState& ego, const MapWaypoints& map)
      : previous_trajectory_(previous_trajectory), ego_(ego), map(map) {}

  Trajectory generate_trajectory(double target_velocity, int target_lane) {
    const double anchor_spacement = 50.0;
    const unsigned extra_anchors = 2;
    const unsigned path_length = 50;

    anchors_init();  // move this outside -> just call once per cycle
    anchors_trim();
    anchors_add(anchor_spacement, extra_anchors, target_lane);
    anchors_recenter();

    spline.set_points(anchors_x, anchors_y);

    auto generated_path = previous_trajectory_;

    // generated_path.trim(10);

    auto missing_points = path_length - generated_path.size();

    auto x_increment = get_x_increment(target_velocity);

    double x{0}, y{0};

    for (int i = 1; i <= missing_points; ++i) {
      x = x + x_increment;
      y = spline(x);

      generated_path.x.push_back(x * cos(ref_yaw) - y * sin(ref_yaw) + ref_x);
      generated_path.y.push_back(x * sin(ref_yaw) + y * cos(ref_yaw) + ref_y);
    }

    generated_path.calculate_end_frenet(map.x, map.y);

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

    std::size_t prev_size = previous_trajectory_.size();

    if (prev_size < 2) {
      anchors_x.push_back(ego_.x - cos(ref_yaw));
      anchors_x.push_back(ego_.x);

      anchors_y.push_back(ego_.y - sin(ref_yaw));
      anchors_y.push_back(ego_.y);
    } else {
      ref_x = previous_trajectory_.x.end()[-1];
      ref_y = previous_trajectory_.y.end()[-1];

      double ref_x_prev = previous_trajectory_.x.end()[-2];
      double ref_y_prev = previous_trajectory_.y.end()[-2];

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
  const Trajectory& previous_trajectory_;
  const VehicleState& ego_;
  const MapWaypoints map;
};

std::ostream& operator<<(std::ostream& os, const Prediction& rhs) {
  os << std::fixed << std::setprecision(2);
  // clang-format off
  for (auto& vehicle_history : rhs.history) {
    if (not vehicle_history.empty()) {
      os << vehicle_history.back() << '\n';
    } else {
      os << "EMPTY" << '\n';
    }
  }
  os << '\n';
  os << "|LANE_SPEEDS|\n";
  os << "Left[" << rhs.lane_speeds[0] << "] ";
  os << "Center[" << rhs.lane_speeds[1]  << "] ";
  os << "Right[" << rhs.lane_speeds[2] << "]\n\n";


  os << "|GAPS|\n";
  os << "Left[" << rhs.predicted_gaps[0].distance_behind << " <-> " << rhs.predicted_gaps[0].distance_ahead << "] ";
  os << "Center[" << rhs.predicted_gaps[1].distance_behind << " <-> " << rhs.predicted_gaps[1].distance_ahead << "] ";
  os << "Right[" << rhs.predicted_gaps[2].distance_behind << " <-> " << rhs.predicted_gaps[2].distance_ahead << "] ";

  // clang-format on

  return os;
}

int main() {
  uWS::Hub h;

  MapWaypoints map(parameters::map_file, parameters::max_s);
  double target_velocity = 0;  // mph
  int lane = 1;

  VehicleState ego;
  Prediction prediction{map};
  Trajectory previous_trajectory("previous_trajectory");
  TrajectoryGenerator trajectory_generator(previous_trajectory, ego, map);

  h.onMessage([&map, &target_velocity, &lane, &previous_trajectory, &ego,
               &trajectory_generator,
               &prediction](uWS::WebSocket<uWS::SERVER> ws, char* data,
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
          previous_trajectory.update(telemetry_data);

          double delta_x = fabs(ego.x - prev_ego.x);
          double delta_y = fabs(ego.y - prev_ego.y);

          double distance_traveled =
              sqrt(delta_x * delta_x + delta_y * delta_y);

          double average_speed = (ego.speed / 2 + prev_ego.speed / 2);

          double time = distance_traveled / average_speed;

          prediction.update_object_history(telemetry_data["sensor_fusion"]);
          prediction.predict_gaps(ego, previous_trajectory.end_s,
                                  previous_trajectory.size() * 0.02);

          double front_speed = prediction.vehicle_close_ahead(
              previous_trajectory.size(), previous_trajectory.end_s,
              ego.get_lane(), ego.s);

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

          auto next_path =
              trajectory_generator.generate_trajectory(target_velocity, lane);

          // LOGGING ------------------------
          std::cout << "|EGO|\n" << ego << "\n\n";

          std::cout << "|STEP|\n"
                    << "t_delta[" << time << "] dist_delta["
                    << distance_traveled << "] v_avg[" << average_speed << ']'
                    << "\n\n";

          if (not previous_trajectory.empty()) {
            std::cout << "|PREV_PATH|\n" << previous_trajectory << "\n\n";
          }

          std::cout << "|NEXT_PATH|\n" << next_path << "\n\n";

          std::cout << "|OBJECT_HISTORY|\n" << prediction << "\n";
          std::cout << "--------------------------------" << std::endl;
          // ---------------------------------

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
