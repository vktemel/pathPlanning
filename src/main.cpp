#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <algorithm>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

class Vehicle {
  public:
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double prev_speed;
  int lane;
};

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  Vehicle ego;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          ego.x = j[1]["x"];
          ego.y = j[1]["y"];
          ego.s = j[1]["s"];
          ego.d = j[1]["d"];
          ego.yaw = j[1]["yaw"];
          ego.prev_speed = j[1]["speed"];
          
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

          double lane = 0;

          // Scan for targets
          // Initialize flag for finding a target to false, and speed of target to 100 m/s.
          bool found_target = false;
          double v_target_obj = 100; // m/s
          double dist_target_obj = 1000; // m

          // Scan all of sensor_fusion list for relevant targets
          for(auto& obj : sensor_fusion)
          {
            //Extract object information
            double id_obj = obj[0];
            double x_obj = obj[1];
            double y_obj = obj[2];
            double vx_obj = obj[3];
            double vy_obj = obj[4];
            double s_obj = obj[5];
            double d_obj = obj[6];

            // If an object is ahead of vehicle, but not too far ahead, 
            // And if it's in the same lane as ego vehicle
            if((s_obj > ego.s) & (s_obj < ego.s + 20) & (d_obj < lane*4+4) & (d_obj > lane*4)){
              // Set the distance to the target and estimate speed of the object based 
              // on velocity vectors in x and y. 
              dist_target_obj = abs(ego.s-s_obj); // m
              v_target_obj = sqrt(vx_obj*vx_obj+vy_obj*vy_obj); // m/s
              
              // Stop scanning
              break;
            }
          }

          // Initialize x and y points to create a rough trajectory. These points will be then
          // used in a spline to have a smoothened trajectory. 
          vector<double> x_pts; 
          vector<double> y_pts; 

          double ref_x = ego.x;
          double ref_y = ego.y;
          double ref_yaw = deg2rad(ego.yaw);

          // The first point to consider is the car's current position
          x_pts.push_back(ego.x);
          y_pts.push_back(ego.y);

          // In order to have a smoothened behavior, points from previous path can be added
          // to the generation of spline, so there are no immediate jerky movements. 

          // Get the size of the previous path.
          int prev_path_size = previous_path_y.size();

          // Push previous path points for smoothening
          int prev_path_limit = 3;
          if(prev_path_size > prev_path_limit)
          {
            for(int p=0; p<prev_path_limit; p++) {
              double x_pt = previous_path_x[p];
              double y_pt = previous_path_y[p];

              vector<double> frenetCoord = getFrenet(x_pt, y_pt, deg2rad(ego.yaw), map_waypoints_x, map_waypoints_y);
  
              // if the found point is behind the vehicle, then ignore
              if(ego.s > frenetCoord[0])
              {
                continue;
              }

              x_pts.push_back(x_pt);
              y_pts.push_back(y_pt);
            }
          }
          
          for(int i=0; i<3; i++) {
            vector<double> xy = getXY(ego.s+(i+1)*20, 2+lane*2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            double x_pt = xy[0];
            double y_pt = xy[1];

            x_pts.push_back(x_pt);
            y_pts.push_back(y_pt);
          }

          for(int i=0; i<x_pts.size(); i++)
          {
            double transformed_x = (x_pts[i]-ref_x)*cos(-ref_yaw)-(y_pts[i]-ref_y)*sin(-ref_yaw);
            double transformed_y = (x_pts[i]-ref_x)*sin(-ref_yaw)+(y_pts[i]-ref_y)*cos(-ref_yaw);

            x_pts[i] = transformed_x;
            y_pts[i] = transformed_y; 
          }

          tk::spline s;
          s.set_points(x_pts, y_pts);

          double t = 0.02; // s
          double max_jerk = 10.0; // m/s^3

          double prev_x = 0;
          double prev_y = 0;
          double prev_theta = deg2rad(ego.yaw);
          double prev_speed = ego.prev_speed*1.6/3.6; // m/s

          double target_speed = prev_speed;

          double lim_speed = 49.5*1.6/3.6; 

          std::cout << "prev speed: " << prev_speed << "\tspeed lim: " << lim_speed << "\tcar in front: " << v_target_obj << std::endl;

          if(prev_speed > v_target_obj*0.98) {
            target_speed = prev_speed-max_jerk*t*0.2;
          }
          else if(prev_speed < lim_speed){
            target_speed = prev_speed+max_jerk*t;
          }

          //double target_speed = std::min(prev_speed+max_jerk*t, std::min(49.5, v_car_in_front)); // mph

          double dist_inc = target_speed*t;

          for(int i=0; i<50; i++)
          {
            double next_x = prev_x + dist_inc;
            double next_y = s(next_x); 
            //double next_x = prev_x + dist_inc*cos(prev_theta);
            //double next_y = s(next_x);

            double dist = distance(prev_x, prev_y, next_x, next_y); 

            prev_theta = atan2(next_y-prev_y,next_x-prev_x);
            prev_x = next_x;
            prev_y = next_y; 

            next_x = ref_x + (prev_x*cos(ref_yaw) - prev_y*sin(ref_yaw));
            next_y = ref_y + (prev_x*sin(ref_yaw) + prev_y*cos(ref_yaw));

            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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