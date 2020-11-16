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

class EgoDynamics {
 public:
  EgoDynamics(json data) {
    x = data["x"];
    y = data["y"];
    s = data["s"];
    d = data["d"];
    yaw = data["yaw"];
    speed = data["speed"];
  }

  double x, y, s, d, yaw, speed;
};

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
          auto data = j[1];

          EgoDynamics ego(data);

          auto previous_path_x = data["previous_path_x"];
          auto previous_path_y = data["previous_path_y"];
          double end_path_s = data["end_path_s"];
          double end_path_d = data["end_path_d"];

          auto sensor_fusion = data["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          std::cout << "car_x[" << std::fixed << std::setprecision(1)
                    << std::setw(7) << ego.x << ']' << " car_y["
                    << std::setprecision(1) << std::setw(7) << ego.y << ']'
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

          double dist_inc = 0.3;
          for (int i = 0; i < 50; ++i) {
            double next_s = ego.s + (i + 1) * (dist_inc);
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