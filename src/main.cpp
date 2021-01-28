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

enum class VehicleStates 
{
  KeepLane,
  LaneChangeLeft,
  LaneChangeRight
};
class Vehicle 
{
  public:
  void set_values(double x, double y, double s, double d, double yaw, double speed);
  
  // Initialization of Vehicle Class
  Vehicle() :
  x(0.0F), y(0.0F), s(0.0F), d(0.0F), yaw(0.0F), speed(0.0F), lane(1.0F) {}

  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  VehicleStates state;
  int lane;
};

void Vehicle::set_values(const double x_in, const double y_in, const double s_in, const double d_in, const double yaw_in, const double speed_in) 
{
  // This function sets properties of the Vehicle class based on the given input values
  x = x_in; 
  y = y_in;
  s = s_in;
  d = d_in;
  yaw = yaw_in;
  speed = speed_in;

  if(d > 0 && d <= 4)
  {
    lane = 0;
  }
  else if(d > 4 && d <= 8)
  {
    lane = 1;
  }
  else if(d > 8 && d <= 12)
  {
    lane = 2;
  }
}

void transform_pts_to_ego_coord(vector<double> &x, vector<double> &y, const Vehicle ego) 
{
  // This function transforms the points from map coordinate system to ego coordinate system
  for(int i=0; i<x.size(); i++)
  {
    double transformed_x = (x[i]-ego.x)*cos(-deg2rad(ego.yaw))-(y[i]-ego.y)*sin(-deg2rad(ego.yaw));
    double transformed_y = (x[i]-ego.x)*sin(-deg2rad(ego.yaw))+(y[i]-ego.y)*cos(-deg2rad(ego.yaw));

    x[i] = transformed_x;
    y[i] = transformed_y; 
  }
};

tk::spline generate_state_spline(int target_lane, const Vehicle veh, 
                               const vector<double> prev_path_x, const vector<double> prev_path_y, 
                               const vector<double> map_wp_x, const vector<double> map_wp_y, const vector<double> map_wp_s) 
{
  
  // The first point to consider is the car's current position
  vector<double> x_pts;
  vector<double> y_pts;
  x_pts.push_back(veh.x);
  y_pts.push_back(veh.y);
  
  // In order to have a smoothened behavior, points from previous path can be added
  // to the generation of spline, so there are no immediate jerky movements. 

  // Get the size of the previous path.
  int prev_path_size = prev_path_y.size();

  // Push previous path points for smoothening
  int prev_path_limit = 3;
  if(prev_path_size > prev_path_limit)
  {
    for(int p=0; p<prev_path_limit; p++) 
    {
      double x_pt = prev_path_x[p];
      double y_pt = prev_path_y[p];

      vector<double> frenetCoord = getFrenet(x_pt, y_pt, deg2rad(veh.yaw), map_wp_x, map_wp_y);
  
      // if the found point is behind the vehicle, then ignore
      if(veh.s > frenetCoord[0])
      {
        continue;
      }

      // push the x point and y point to the main vector
      x_pts.push_back(x_pt);
      y_pts.push_back(y_pt);
    }
  }
          
  // In order to have spline extend to future, get 3 more points which are pointing towards the target lane
  // This way, a spline will be generated that ends at target lane, and the vehicle can have
  // a smooth transition to reach the target lane. 

  for(int i=0; i<3; i++) 
  {
    vector<double> xy = getXY(veh.s+(i+1)*30, 2+target_lane*4, map_wp_s, map_wp_x, map_wp_y);

    double x_pt = xy[0];
    double y_pt = xy[1];

    x_pts.push_back(x_pt);
    y_pts.push_back(y_pt);
  }
  
  // transform the points to ego coordinate system
  transform_pts_to_ego_coord(x_pts, y_pts, veh);

  // create and set points to the spline
  tk::spline s;
  s.set_points(x_pts, y_pts);

  // return the spline
  return s;
};

vector<VehicleStates> find_successor_states(const Vehicle veh) 
{
  // This function finds the possible next states based on the vehicles current state
  // It assumes KeepLane is always a possible state (in case it's less costlier to stay
  // in current lane), and it adds options to change lanes to left and right
  // based on ego vehicle's current lane. 
  vector<VehicleStates> successor_states;

  VehicleStates current_state = veh.state;

  successor_states.push_back(VehicleStates::KeepLane);
  
  if(current_state == VehicleStates::KeepLane) 
  {
    // ego vehicle is not on left-most lane
    if(veh.lane != 0) 
    {
      successor_states.push_back(VehicleStates::LaneChangeLeft);
    }
    // ego vehicle is not on right-most lane
    if(veh.lane != 2) 
    {
      successor_states.push_back(VehicleStates::LaneChangeRight);
    }
  }
  else if(current_state == VehicleStates::LaneChangeRight) 
  {
    // If LaneChangeRight is current state, then it's assumed by default to be a possible successor state
    successor_states.push_back(VehicleStates::LaneChangeRight);
  }
  else if(current_state == VehicleStates::LaneChangeLeft) 
  {
    // If LaneChangeLeft is current state, then it's assumed by default to be a possible successor state
    successor_states.push_back(VehicleStates::LaneChangeLeft);
  }

  return successor_states;
};

double calc_cost_lane_emptiness(const Vehicle ego, const int lane, const vector<vector<double>> sensor_fusion) 
{
  // This function calculates the cost for a given lane on the road based on the emptiness of the lane
  // If there are no vehicles in sight, then the cost is 0. If there's only 1 vehicle, then cost is 0.5,
  // if there are more than 1 vehicle in a given lane, then the cost will be set to 1. 

  // Initialize variables
  double cost = 1.0;
  int cnt = 0;

  // Count objects in the given lane
  for(auto& obj : sensor_fusion) 
  {
    double s_obj = obj[5];
    double d_obj = obj[6];

    if((s_obj < ego.s+50) && (s_obj > ego.s) && (d_obj > lane*4) && (d_obj < lane*4+4))
    {
      cnt += 1;
    }
  }

  // Set the costs
  if(cnt == 0) 
  {
    cost = 0.0;
  }
  else if(cnt == 1) 
  {
    cost = 0.5;
  }
  else 
  {
    cost = 1.0;
  }

  return cost;
}

double calc_cost_collision(const Vehicle ego, const int target_lane, const vector<vector<double>> sensor_fusion) {

  // This function looks if another object exists in the target lane that ego vehicle may collide with. 
  // The distance to be scanned behind and ahead of the ego vehicle.  is parameterized for better optimization.
  double cost = 0.0; 
  double scan_dist = 10.0; // meters

  // Scan objects to find if any object is within immediate proximity of ego vehicle on the target lane. 
  for(auto& obj : sensor_fusion) 
  {
    double s_obj = obj[5];
    double d_obj = obj[6];

    // Set cost to 1 if a collision is expected. 
    if((s_obj < ego.s+scan_dist) && (s_obj > ego.s-scan_dist) && (d_obj >= target_lane*4) && (d_obj <= target_lane*4+4)) 
    {
      cost = 1.0;
      break;
    }
  }
  return cost; 
}

double calc_cost_speed(const Vehicle ego, const int lane, const vector<vector<double>> sensor_fusion) 
{
  
  double cost = 0.0;
  double lane_speed = 0.0;
  double max_speed = 22.0;

  vector<vector<double>> objects_in_lane;
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

    if((s_obj > (ego.s-5)) && (s_obj < ego.s + 30) && (d_obj < lane*4+4) & (d_obj >= lane*4)) 
    {
      vector<double> selected;
      selected.push_back(id_obj);
      selected.push_back(x_obj);
      selected.push_back(y_obj);
      selected.push_back(vx_obj);
      selected.push_back(vy_obj);
      selected.push_back(s_obj);
      selected.push_back(d_obj);
      objects_in_lane.push_back(selected);
    }
  }

  int cnt_object = objects_in_lane.size();
  if(cnt_object == 0)
  {
    return cost;
  }
  else
  {
    int cnt = 0; 
    
    for(auto& obj : objects_in_lane)
    {
      double veh_speed = sqrt(obj[3]*obj[3]+obj[4]*obj[4]);
      lane_speed += veh_speed;
    }

    lane_speed = lane_speed/cnt_object;
  }

  cost = std::max(0.0, max_speed-lane_speed)/max_speed;
  return cost;
}

VehicleStates evaluate_successor_states(const vector<VehicleStates> successor_states, const Vehicle ego, const vector<vector<double>> sensor_fusion){
  // default, stay in lane
  VehicleStates min_cost_state = VehicleStates::KeepLane;
  double min_cost = 1;

  vector<double> state_costs;

  // This loop always start with keeplane, as keep lane is always an available successor state
  // In case that multiple lanes have equal costs, this will default to keep lane as it's the
  // initial min cost
  for(int i=0; i<successor_states.size(); i++)
  {
    double cost = 1;

    std::cout << i << ": ";

    if(successor_states[i] == VehicleStates::KeepLane){
      // Speed Cost
      double speed_cost = calc_cost_speed(ego, ego.lane, sensor_fusion); // std::max(0.0, max_speed-lane_speeds[ego.lane])/max_speed;
      double empty_cost = calc_cost_lane_emptiness(ego, ego.lane, sensor_fusion);

      cost = speed_cost + empty_cost;
      
      if(ego.lane != 2) {
        cost += 0.1;
      }

      std::cout << "KeepLane cost:\t" << cost << "\t speed: " << speed_cost << "\t empty: " << empty_cost << std::endl;
    }
    else if(successor_states[i] == VehicleStates::LaneChangeLeft) {
      
      double speed_cost = calc_cost_speed(ego, ego.lane, sensor_fusion); //std::max(0.0, max_speed-lane_speeds[ego.lane-1])/max_speed;
      double empty_cost = calc_cost_lane_emptiness(ego, ego.lane-1, sensor_fusion);
      double collision_cost = calc_cost_collision(ego, ego.lane-1, sensor_fusion);
      cost = speed_cost + empty_cost + collision_cost;
      cost += 0.1;
      if(ego.lane != 2) {
        cost += 0.1;
      }
      std::cout << "Left cost:\t" << cost << "\t speed: " << speed_cost << "\t empty: " << empty_cost << std::endl;

    }
    else if(successor_states[i] == VehicleStates::LaneChangeRight) {
      
      double speed_cost = calc_cost_speed(ego, ego.lane, sensor_fusion); //std::max(0.0, max_speed-lane_speeds[ego.lane+1])/max_speed;  
      double empty_cost = calc_cost_lane_emptiness(ego, ego.lane+1, sensor_fusion);
      double collision_cost = calc_cost_collision(ego, ego.lane+1, sensor_fusion);
      cost = speed_cost + empty_cost + collision_cost;
      cost += 0.1;  
      std::cout << "Right cost:\t" << cost << "\t speed: " << speed_cost << "\t empty:" << empty_cost << std::endl;
    }
//    std::cout << "successor state: " << VehStateNames[successor_states[i]] << ", cost: " << cost << std::endl;

    cost = std::min(1.0, cost);
    state_costs.push_back(cost); 

    if(cost < min_cost)
    {
      min_cost_state = successor_states[i];
      min_cost = cost;
    }
  }

  //std::cout << "min cost state: " << min_cost_state << std::endl;
  return min_cost_state;

};

vector<double> get_vehicles_in_lane(const int lane, const vector<vector<double>> sensor_fusion, const Vehicle ego){
  vector<double> obj_in_lane;
  
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
      obj_in_lane.push_back(id_obj);
      obj_in_lane.push_back(x_obj);
      obj_in_lane.push_back(y_obj);
      obj_in_lane.push_back(vx_obj);
      obj_in_lane.push_back(vy_obj);
      obj_in_lane.push_back(s_obj);
      obj_in_lane.push_back(d_obj);
      // Stop scanning
      break;
    }
  }
  return obj_in_lane;
}

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
  ego.state = VehicleStates::KeepLane;

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

          vector<VehicleStates> successor_states = find_successor_states(ego);

          VehicleStates min_cost_state = evaluate_successor_states(successor_states, ego, sensor_fusion);

          int target_lane = ego.lane;

          double t = 0.02; // s
          double max_jerk = 10.0; // m/s^3

          double target_speed = ego.speed*1.6/3.6; // m/s
          double lim_speed = 49.5*1.6/3.6; 

          double target_acc = 0.0;

          double v_target_obj = 100; // m/s
          double dist_target_obj = 1000; // m

          if(min_cost_state == VehicleStates::KeepLane) {
            // Scan for target in lane
            // Initialize speed of target to 100 m/s.
            ego.state = VehicleStates::KeepLane;

            vector<double> obj_ahead = get_vehicles_in_lane(ego.lane, sensor_fusion, ego);

            if(obj_ahead.size() > 0){
              dist_target_obj = abs(ego.s-obj_ahead[5]); // m
              v_target_obj = sqrt(obj_ahead[3]*obj_ahead[3]+obj_ahead[4]*obj_ahead[4]); // m/s
            }
          }
          else if(min_cost_state == VehicleStates::LaneChangeRight)
          {
            ego.state = VehicleStates:: LaneChangeRight;
            
            vector<double> obj_ahead = get_vehicles_in_lane(ego.lane+1, sensor_fusion, ego);

            if(obj_ahead.size() > 0){
              dist_target_obj = abs(ego.s-obj_ahead[5]); // m
              v_target_obj = sqrt(obj_ahead[3]*obj_ahead[3]+obj_ahead[4]*obj_ahead[4]); // m/s
            }

            target_lane = ego.lane + 1;
          }

          else if(min_cost_state == VehicleStates::LaneChangeLeft)
          {
            ego.state = VehicleStates:: LaneChangeLeft;
            
            vector<double> obj_ahead = get_vehicles_in_lane(ego.lane-1, sensor_fusion, ego);

            if(obj_ahead.size() > 0){
              dist_target_obj = abs(ego.s-obj_ahead[5]); // m
              v_target_obj = sqrt(obj_ahead[3]*obj_ahead[3]+obj_ahead[4]*obj_ahead[4]); // m/s
            }

            target_lane = ego.lane-1;
          }

          if(target_speed > v_target_obj*0.98) {
            //target_speed -= (max_jerk * t * 0.2);
            target_acc = -1.0*max_jerk*t*0.2;
          }
          else if(target_speed < lim_speed){
            //target_speed += (max_jerk * t);
            target_acc = max_jerk*t;
          }

          
          // Initialize x and y points to create a rough trajectory. These points will be then
          // used in a spline to have a smoothened trajectory. 
          vector<double> x_pts; 
          vector<double> y_pts; 

          double ref_x = ego.x;
          double ref_y = ego.y;
          double ref_yaw = deg2rad(ego.yaw);


          tk::spline s;
          s = generate_state_spline(target_lane, ego, previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s);

          double prev_x = 0;
          double prev_y = 0;
          double prev_theta = deg2rad(ego.yaw);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //generate_ego_traj(next_x_vals, next_y_vals, ego, target_state);
          
          target_speed += target_acc;

          for(int i=0; i<50; i++)
          {
            
            double dist_inc = target_speed*t*(i-1);

            double next_x = 0 + dist_inc;
            double next_y = s(next_x); 

            double dist = distance(prev_x, prev_y, next_x, next_y); 

            prev_theta = atan2(next_y-prev_y,next_x-prev_x);
            prev_x = next_x;
            prev_y = next_y; 

            next_x = ego.x + (prev_x*cos(ref_yaw) - prev_y*sin(ref_yaw));
            next_y = ego.y + (prev_x*sin(ref_yaw) + prev_y*cos(ref_yaw));

            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

          for(int i=0; i<20; i++)
          {
            std::cout << "Next Path (Map)\tx: " << next_x_vals[i] << "\ty: " << next_y_vals[i] << std::endl;
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