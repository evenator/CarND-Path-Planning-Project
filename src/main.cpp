#include "Eigen-3.3/Eigen/Dense"
#include "geometry.h"
#include "json.hpp"
#include "map.h"
#include "obstacle.h"
#include "path.h"
#include "planner.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;

inline double mph_to_mps(double speed_mph) { return speed_mph * 0.44704; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;
  auto map = Map::from_file("../data/highway_map.csv");
  FSMPlanner fsm(&map);
  auto last_time = std::chrono::system_clock::now();

  h.onMessage([&fsm, &last_time](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                 size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto this_time = std::chrono::system_clock::now();
        std::chrono::duration<double> time_since = this_time - last_time;
        std::cout << "LOOP TIME: " << time_since.count()
                  << " s since last callback" << std::endl;
        last_time = this_time;
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double speed = mph_to_mps(j[1]["speed"]);
          double theta = deg2rad(j[1]["yaw"]);
          Eigen::Vector4d car_state_xy(j[1]["x"], j[1]["y"], theta, speed);
          Eigen::Vector3d car_state_sd(j[1]["s"], j[1]["d"], speed);

          Eigen::IOFormat vector_format(Eigen::StreamPrecision,
                                        Eigen::DontAlignCols, ", ", ", ", "",
                                        "", "(", ")");
          std::cout << "Car State " << car_state_xy.format(vector_format)
                    << std::endl;

          // Previous path data given to the Planner
          auto previous_path =
              Path(j[1]["previous_path_x"], j[1]["previous_path_y"]);
          std::cout << "Previous path has " << previous_path.size()
                    << " points." << std::endl;

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          std::vector<Obstacle> obstacles;
          for (auto const &obstacle_params : j[1]["sensor_fusion"]) {
            obstacles.emplace_back(obstacle_params);
          }

          json msgJson;

          Path path = fsm.plan(car_state_xy, obstacles, previous_path);

          std::cout << "Sending " << path.size() << " points." << std::endl;
          msgJson["next_x"] = path.get_x();
          msgJson["next_y"] = path.get_y();

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          std::cout << "--------------------------------" << std::endl;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
