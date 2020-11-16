#include <uWS/uWS.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "map.h"
#include "parameters.h"

using nlohmann::json;
using std::string;
using std::vector;

static std::ofstream telemetry_log("./telemetry.log", std::ios::app);

int main() {
  uWS::Hub h;

  Map map(parameters::map_file, parameters::max_s);

  h.onMessage([&map](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    if (valid_socket_message(length, data)) {
      auto s = hasData(data);

      if (not s.empty()) {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          std::cout << "car_x[" << std::fixed << std::setprecision(1)
                    << std::setw(7) << car_x << ']' << " car_y["
                    << std::setprecision(1) << std::setw(7) << car_y << ']'
                    << std::endl;

          if (not previous_path_x.empty()) {
            std::cout << std::fixed << std::setprecision(1) << "prev_x["
                      << std::fixed << std::setprecision(1) << std::setw(7)
                      << std::setprecision(1) << previous_path_x.front() << "->"
                      << std::setprecision(1) << std::setw(7)
                      << previous_path_x.back() << "] prev_y["
                      << std::setprecision(1) << std::setw(7)
                      << previous_path_y.front() << "->" << std::setprecision(1)
                      << std::setw(7) << previous_path_y.back() << ']'
                      << std::endl;
          }
          // std::cout << map.waypoints_x.size() << std::endl;

          // std::cout << '\r' << "s:" << car_s << "d:" << car_d
          //          << "yaw:" << car_yaw << "   ";

          // auto vec = getFrenet(car_x, car_y, car_yaw, map.waypoints_x,
          //                     map.waypoints_y);

          // std::cout << "Frenet{" << vec[0] << ',' << vec[1] << '}';

          // std::cout << "Closest Waypoint"
          //          << ClosestWaypoint(car_x, car_y, map.waypoints_x,
          //                             map.waypoints_y)
          //          << std::endl;

          double dist_inc = 0.3;
          for (int i = 0; i < 50; ++i) {
            double next_s = car_s + (i + 1) * (dist_inc);
            double next_d = 6;

            auto xy = getXY(next_s, next_d, map.waypoints_s, map.waypoints_x,
                            map.waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }

          std::cout << "next_x[" << std::fixed << std::setprecision(3)
                    << std::setw(7) << std::setprecision(3)
                    << next_x_vals.front() << "->" << std::setprecision(3)
                    << std::setw(7) << next_x_vals.back() << "] next_y["
                    << std::setprecision(3) << std::setw(7)
                    << next_y_vals.front() << "->" << std::setprecision(3)
                    << std::setw(7) << next_y_vals.back() << "]\n"
                    << std::endl;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
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