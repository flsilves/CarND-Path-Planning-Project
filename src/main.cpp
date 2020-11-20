#include <uWS/uWS.h>

#include <iostream>
#include <json.hpp>
#include <string>
#include <vector>

#include "helpers.h"
#include "map.h"
#include "parameters.h"
#include "planning.h"
#include "prediction.h"
#include "trajectory.h"
#include "vehicle.h"

using nlohmann::json;
using std::string;
using std::vector;

// static std::ofstream telemetry_log("./telemetry.log", std::ios::app);

int main() {
  uWS::Hub h;

  VehicleState ego;
  Trajectory previous_trajectory;
  MapWaypoints map(parameters::map_file, parameters::max_s);
  TrajectoryGenerator trajectory_generator{previous_trajectory, ego, map};
  Prediction prediction{map};
  Planner motion_planning{ego, trajectory_generator, prediction, map};

  h.onMessage([&ego, &previous_trajectory, &map, &trajectory_generator,
               &prediction, &motion_planning](uWS::WebSocket<uWS::SERVER> ws,
                                              char* data, std::size_t length,
                                              uWS::OpCode opCode) {
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
          prediction.predict_gaps(
              ego, previous_trajectory.end_s,
              50 * 0.02);  // TODO PREDICT INTO COMPLETE SIZE OF PATH

          auto next_path = motion_planning.get_trajectory();

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
