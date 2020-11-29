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

void print_info(const VehicleState& ego, const VehicleState& previous_ego,
                const Trajectory& previous_trajectory,
                const Trajectory& planned_trajectory,
                const Prediction& prediction, const Planner& planning);

int main() {
  uWS::Hub h;

  VehicleState ego{};
  Trajectory previous_trajectory{};
  Trajectory planned_trajectory{};
  MapWaypoints map(MAP_FILEPATH, MAP_MAX_S);
  Prediction prediction(map, ego);
  TrajectoryGenerator trajectory_generator{previous_trajectory, ego, map,
                                           prediction};
  Planner motion_planning{ego, trajectory_generator, prediction, map,
                          previous_trajectory};

  h.onMessage([&ego, &previous_trajectory, &planned_trajectory, &map,
               &prediction, &motion_planning](uWS::WebSocket<uWS::SERVER> ws,
                                              char* data, std::size_t length,
                                              uWS::OpCode opCode) {
    if (valid_socket_message(length, data)) {
      auto s = hasData(data);

      if (not s.empty()) {
        auto j = json::parse(s);
        auto event = j[0].get<string>();

        if (event == "telemetry") {
          auto data = j[1];

          VehicleState prev_ego{ego};

          ego.update(data["x"], data["y"], data["s"], data["d"], data["yaw"],
                     data["speed"]);

          previous_trajectory.update(
              data["previous_path_x"].get<std::vector<double>>(),
              data["previous_path_y"].get<std::vector<double>>(),
              data["end_path_s"], data["end_path_d"]);

          prediction.update(data["sensor_fusion"]);
          prediction.predict_gaps(
              ego, previous_trajectory.end_s,
              50 * 0.02);  // TODO PREDICT INTO COMPLETE SIZE OF PATH

          planned_trajectory = motion_planning.get_trajectory();

          print_info(ego, prev_ego, previous_trajectory, planned_trajectory,
                     prediction, motion_planning);

          previous_trajectory = planned_trajectory;

          json msgJson;
          msgJson["next_x"] = planned_trajectory.x;
          msgJson["next_y"] = planned_trajectory.y;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
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

void print_info(const VehicleState& ego, const VehicleState& prev_ego,
                const Trajectory& previous_trajectory,
                const Trajectory& planned_trajectory,
                const Prediction& prediction, const Planner& planner) {
  // double delta_x = fabs(ego.x - prev_ego.x);
  // double delta_y = fabs(ego.y - prev_ego.y);
  //
  // double distance_traveled = sqrt(delta_x * delta_x + delta_y * delta_y);
  //
  // double average_speed = (ego.speed / 2 + prev_ego.speed / 2);
  //
  // double time = distance_traveled / average_speed;

  // clang-format off
  std::cout << "|EGO|\n" 
            << ego << "\n\n";

  //std::cout << "|STEP|\n"
  //          << "t_delta[" << time << "] "
  //          << "dist_delta[" << distance_traveled << "] "
  //          << "v_avg[" << average_speed << "]\n\n";

  std::cout << prediction << "\n\n";

  std::cout << "|PLANNER|\n"
            << planner << "\n\n";            

  if (not previous_trajectory.empty()) {
    std::cout << "|PREV_PATH|\n" << previous_trajectory << "\n\n";
  }

  std::cout << "|NEXT_PATH|\n" << planned_trajectory << "\n\n";



  std::cout << "--------------------------------" << std::endl;
  // clang-format onOBJECT_HISTORY
}