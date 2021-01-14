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

/* what needs to be in vehicle state? 
- State name, i.e. KeepLane, LaneChangeLeft
- Vehicle speed
- Lane

- Future state needs to have:
-- Target state
-- Target speed
-- Target lane

Example: Initial state is KeepLane. Possible states from that are KeepLane, PrepLaneChangeLeft or PrepLaneChangeRight
KeepLaneCosts are proportional to
- Speed of Lane

If PLCL is selected, then the vehicle needs to adjust speed in the given trajectory; but shouldn't necessarily change
selected lane. So the waypoint calculation will still be the same.

If LCL is selected, then the waypoints should refer to the left lane in some point in future. 

*/ 

enum VehicleStates {
  KeepLane,
  PrepareLaneChangeLeft, 
  PrepareLaneChangeRight,
  LaneChangeLeft,
  LaneChangeRight
};


class State{
  public:
  VehicleStates name;
  int lane;
  double speed;
  vector<double> x_pts;
  vector<double> y_pts;
};
class Vehicle {
  public:
  void set_values(double x, double y, double s, double d, double yaw, double speed);
  
  // Initialization of Vehicle Class
  Vehicle() :
  x(0.0F), y(0.0F), s(0.0F), d(0.0F), yaw(0.0F), speed(0.0F),
  lane(1.0F)
  {}

  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  State curr_state;
  State next_state;
  int lane;
};

void Vehicle::set_values(double x_in, double y_in, double s_in, double d_in, double yaw_in, double speed_in)
{
  x = x_in; 
  y = y_in;
  s = s_in;
  d = d_in;
  yaw = yaw_in;
  speed = speed_in;
}

VehicleStates select_state(Vehicle veh){
  VehicleStates state;

  if((veh.curr_state.name == VehicleStates::KeepLane) & (veh.lane == 1) & (veh.speed > 20))
  {
    state = VehicleStates::LaneChangeLeft;
  }

  return state;
};


void generate_state_trajectory(State &state, const Vehicle veh, 
                               const vector<double> prev_path_x, const vector<double> prev_path_y, 
                               const vector<double> map_wp_x, const vector<double> map_wp_y, const vector<double> map_wp_s){
  
  // The first point to consider is the car's current position
  state.x_pts.push_back(veh.x);
  state.x_pts.push_back(veh.y);

  // In order to have a smoothened behavior, points from previous path can be added
  // to the generation of spline, so there are no immediate jerky movements. 

  // Get the size of the previous path.
  int prev_path_size = prev_path_y.size();

  // Push previous path points for smoothening
  int prev_path_limit = 3;
  if(prev_path_size > prev_path_limit){
    for(int p=0; p<prev_path_limit; p++) {
      double x_pt = prev_path_x[p];
      double y_pt = prev_path_y[p];

      vector<double> frenetCoord = getFrenet(x_pt, y_pt, deg2rad(veh.yaw), map_wp_x, map_wp_y);
  
      // if the found point is behind the vehicle, then ignore
      if(veh.s > frenetCoord[0])
      {
        continue;
      }

      state.x_pts.push_back(x_pt);
      state.y_pts.push_back(y_pt);
    }
  }
          
  for(int i=0; i<3; i++) {
    vector<double> xy = getXY(veh.s+(i+1)*20, 2+state.lane*4, map_wp_s, map_wp_x, map_wp_y);

    double x_pt = xy[0];
    double y_pt = xy[1];

    state.x_pts.push_back(x_pt);
    state.y_pts.push_back(y_pt);
  }
};

void transform_traj_to_ego_coord(vector<double> &x, vector<double> &y, double ref_x, double ref_y, double ref_theta)
{
  for(int i=0; i<x.size(); i++)
  {
    double transformed_x = (x[i]-ref_x)*cos(-ref_theta)-(y[i]-ref_y)*sin(-ref_theta);
    double transformed_y = (x[i]-ref_x)*sin(-ref_theta)+(y[i]-ref_y)*cos(-ref_theta);

    x[i] = transformed_x;
    y[i] = transformed_y; 
  }
};

void evaluate_successor_states(){};

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
  ego.lane = 1;

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
          ego.set_values(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"], j[1]["yaw"], j[1]["speed"]);

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

          VehicleStates next_state = select_state(ego);
          if(next_state == VehicleStates::LaneChangeLeft)
          {
            ego.lane -= 1;
          }
          else if(next_state == VehicleStates::LaneChangeRight)
          {
            ego.lane += 1;
          }

          int lane = ego.lane;

          //double lane = 1;

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
            vector<double> xy = getXY(ego.s+(i+1)*20, 2+lane*4, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            double x_pt = xy[0];
            double y_pt = xy[1];

            x_pts.push_back(x_pt);
            y_pts.push_back(y_pt);
          }

          transform_traj_to_ego_coord(x_pts, y_pts, ref_x, ref_y, ref_yaw);

          tk::spline s;
          s.set_points(x_pts, y_pts);

          double t = 0.02; // s
          double max_jerk = 10.0; // m/s^3

          double prev_x = 0;
          double prev_y = 0;
          double prev_theta = deg2rad(ego.yaw);
          double prev_speed = ego.speed*1.6/3.6; // m/s

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