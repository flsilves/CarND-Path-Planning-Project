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

using nlohmann::json;
using std::string;
using std::vector;

static std::ofstream telemetry_log("./telemetry.log", std::ios::app);

class EgoDynamics {
 public:
  EgoDynamics(json data) {
    x = data["x"];
    y = data["y"];
    s = data["s"];
    d = data["d"];
    yaw = data["yaw"];      // degrees [positive in ccw direction]
    speed = data["speed"];  // units ??
  }

  double x, y, s, d, yaw, speed;
};

int main() {
  uWS::Hub h;

  MapWaypoints map(parameters::map_file, parameters::max_s);
  double target_velocity = 0;  // mph
  int lane = 1;

  h.onMessage([&map, &target_velocity](uWS::WebSocket<uWS::SERVER> ws,
                                       char *data, size_t length,
                                       uWS::OpCode opCode) {
    if (valid_socket_message(length, data)) {
      auto s = hasData(data);

      if (not s.empty()) {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          auto data = j[1];

          EgoDynamics ego(data);

          auto previous_path_x = data["previous_path_x"];
          auto previous_path_y = data["previous_path_y"];
          double end_path_s = data["end_path_s"];
          double end_path_d = data["end_path_d"];

          auto sensor_fusion = data["sensor_fusion"];

          int lane = 1;
          int prev_size = previous_path_x.size();

          double future_s;
          if (prev_size > 0) {  // ?
            future_s = end_path_s;
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
            target_velocity -= .224;
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
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];

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
          for (int i = 0; i < prev_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our
          // desired reference velocity
          double target_x = 30.0;
          double target_y = spline(target_x);
          double target_dist =
              sqrt((target_x) * (target_x) + (target_y) * (target_y));

          double x_add_on = 0;

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

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          std::cout << "car_x[" << std::fixed << std::setprecision(1)
                    << std::setw(7) << ego.x << ']' << " car_y["
                    << std::setprecision(1) << std::setw(7) << ego.y << ']'
                    << '\n';

          if (not previous_path_x.empty()) {
            std::cout << std::fixed << std::setprecision(1) << "prev_x["
                      << std::fixed << std::setprecision(1) << std::setw(7)
                      << std::setprecision(1) << previous_path_x.front() << "->"
                      << std::setprecision(1) << std::setw(7)
                      << previous_path_x.back() << "] prev_y["
                      << std::setprecision(1) << std::setw(7)
                      << previous_path_y.front() << "->" << std::setprecision(1)
                      << std::setw(7) << previous_path_y.back() << ']' << '\n';
          }

          std::cout << "next_x[" << std::fixed << std::setprecision(3)
                    << std::setw(7) << std::setprecision(3)
                    << next_x_vals.front() << "->" << std::setprecision(3)
                    << std::setw(7) << next_x_vals.back() << "] next_y["
                    << std::setprecision(3) << std::setw(7)
                    << next_y_vals.front() << "->" << std::setprecision(3)
                    << std::setw(7) << next_y_vals.back() << '\n';

          std::cout << "end_path_s[" << end_path_s << "] end_path_d["
                    << end_path_d << "]" << '\n'
                    << std::endl;

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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
                         char *message, size_t length) {
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