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

class EgoDynamics {
 public:
  EgoDynamics(json telemetry_data) {
    x = telemetry_data["x"];
    y = telemetry_data["y"];
    s = telemetry_data["s"];
    d = telemetry_data["d"];
    yaw = telemetry_data["yaw"];      // degrees [positive in ccw direction]
    speed = telemetry_data["speed"];  // units ??
  }

  friend std::ostream& operator<<(std::ostream& os, const EgoDynamics& ego) {
    os << std::fixed << std::setprecision(2);
    os << "[Ego] ";
    os << "x[" << ego.x << "] ";
    os << "y[" << ego.y << "] ";
    os << "s[" << ego.s << "] ";
    os << "d[" << ego.d << "] ";
    os << "yaw[" << ego.yaw << "] ";
    os << "speed[" << ego.speed << ']';
    return os;
  }

 public:
  double x, y, s, d, yaw, speed;
};

class Path {
 public:
  Path(json telemetry_data) {
    x = telemetry_data["previous_path_x"].get<vector<double>>();
    y = telemetry_data["previous_path_y"].get<vector<double>>();
    end_s = telemetry_data["end_path_s"];
    end_d = telemetry_data["end_path_d"];

    if (x.size() != y.size()) {
      throw std::runtime_error("<Path::ctor>: x and y have different lengths");
    }
  }

  Path(const Path& other)
      : x(other.x), y(other.y), end_s(other.end_s), end_d(other.end_d) {}

  bool empty() const { return x.empty(); }
  size_t size() const { return x.size(); }

  friend std::ostream& operator<<(std::ostream& os, const Path& path) {
    os << std::fixed << std::setprecision(2);
    os << "[Path] ";
    os << "x[" << path.x.front() << " -> " << path.x.back() << "] ";
    os << "y[" << path.y.front() << " -> " << path.y.back() << "] ";
    os << "end_s[" << path.end_s << "] ";
    os << "end_d[" << path.end_d << "] ";
    os << "size[" << path.size() << ']';
    return os;
  }

 public:
  vector<double> x, y;
  double end_s, end_d;
};

int main() {
  uWS::Hub h;

  MapWaypoints map(parameters::map_file, parameters::max_s);
  double target_velocity = 0;  // mph
  int lane = 1;

  h.onMessage([&map, &target_velocity, &lane](uWS::WebSocket<uWS::SERVER> ws,
                                              char* data, size_t length,
                                              uWS::OpCode opCode) {
    if (valid_socket_message(length, data)) {
      auto s = hasData(data);

      if (not s.empty()) {
        auto j = json::parse(s);
        auto event = j[0].get<string>();

        if (event == "telemetry") {
          auto telemetry_data = j[1];

          EgoDynamics ego(telemetry_data);
          Path previous_path(telemetry_data);

          auto sensor_fusion = telemetry_data["sensor_fusion"];

          int prev_size = previous_path.size();

          double future_s;
          if (prev_size > 0) {
            future_s = previous_path.end_s;
          }

          bool too_close = false;

          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 + lane - 2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed =
                  sqrt(vx * vx + vy * vy);               // current car speed
              double check_car_s = sensor_fusion[i][5];  // current car s

              check_car_s += static_cast<double>(
                  prev_size * 0.02 *
                  check_speed);  // where is the vehicle in last point of the
                                 // previous preject

              // check if car is in front of us and gap is shorter than 30
              if ((check_car_s > future_s) && (check_car_s - future_s) < 30) {
                // target_velocity = 29.5;
                too_close = true;
              }
            }
          }

          if (too_close) {
            if (lane > 0) {
              lane = 0;
            }
            // target_velocity -= .224;

          } else if (target_velocity < 49.5) {
            target_velocity += .224;
          }

          vector<double> anchor_x;
          vector<double> anchor_y;

          double ref_x = ego.x;
          double ref_y = ego.y;
          double ref_yaw = deg2rad(ego.yaw);

          // Initialization when there's no previous points
          if (prev_size < 2) {
            // Extrapolate the previous position based on current yaw angle
            double prev_car_x = ego.x - cos(ref_yaw);
            double prev_car_y = ego.y - sin(ref_yaw);

            std::cout << "Ego.x:" << prev_car_x << " -> " << ego.x << std::endl;
            std::cout << "Ego.y:" << prev_car_y << " -> " << ego.y << std::endl;

            anchor_x.push_back(prev_car_x);
            anchor_x.push_back(ego.x);

            anchor_y.push_back(prev_car_y);
            anchor_y.push_back(ego.y);
          } else {
            ref_x = previous_path.x[prev_size - 1];
            ref_y = previous_path.y[prev_size - 1];

            double ref_x_prev = previous_path.x[prev_size - 2];
            double ref_y_prev = previous_path.y[prev_size - 2];

            ref_yaw =
                atan2(ref_y - ref_y_prev,
                      ref_x - ref_x_prev);  // where is this used?? Why do we
                                            // need it? can't we use just yaw?

            anchor_x.push_back(ref_x_prev);
            anchor_x.push_back(ref_x);

            anchor_y.push_back(ref_y_prev);
            anchor_y.push_back(ref_y);
          }

          // In Freenet add evenly 30 spaced points ahead of the starting
          // reference
          vector<double> next_wp0 =
              getXY(ego.s + 30, (2 + 4 * lane), map.s, map.x, map.y);

          vector<double> next_wp1 =
              getXY(ego.s + 60, (2 + 4 * lane), map.s, map.x, map.y);

          vector<double> next_wp2 =
              getXY(ego.s + 90, (2 + 4 * lane), map.s, map.x, map.y);

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

            anchor_x[i] =
                (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            anchor_y[i] =
                (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          //  set (x,y) points to the spline
          tk::spline spline;
          spline.set_points(anchor_x, anchor_y);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          std::cout << "prev_size:" << prev_size << std::endl;
          // start with all of the previous path points from last frame

          Path next_path{previous_path};

          // Calculate how to break up spline points so that we travel at our
          // desired reference velocity
          double target_x = 30.0;
          double target_y = spline(target_x);
          double target_dist =
              sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

          // include acceleration for trajectory generation
          for (int i = 1; i <= 50 - prev_size; ++i) {
            double N =
                target_dist / (.02 * target_velocity /
                               2.2369);  // distance =  N (point) * 0.02
                                         // (second/point) * v (miles/second)
            double x_point = x_add_on + (target_x) / N;
            double y_point = spline(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_path.x.push_back(x_point);
            next_path.y.push_back(y_point);
          }

          std::cout << ego << std::endl;

          if (not previous_path.empty()) {
            std::cout << previous_path << std::endl;
          }

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