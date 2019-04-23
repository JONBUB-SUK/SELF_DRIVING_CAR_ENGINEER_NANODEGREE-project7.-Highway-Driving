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


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  
  /*
  Before communication, we need to initiate variables
  */
  // number of lane my car exists
  int lane_number = 1;
  // velocity of my car driving
  double car_velocity = 0.0;
  
  // FSM state
  // This will be declared at helpers.h
  // FSM_state current_FSM_state = FSM_state::keep_lane;
  

  h.onMessage([&lane_number, &car_velocity, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          
          // the number of remaining path
          int prev_size = previous_path_x.size();
          
          if (prev_size > 0)
          {
            car_s = end_path_s;            
          }
          
          // initiate variables
          // these variables need when to check there is object(car) nearby in all lanes
          bool too_close_my_lane = false;
          bool too_close_left_lane = false;
          bool too_close_right_lane = false;
          
          // initiate variables
          // these variables need when everytime we calculate distance between cars
          // so it needs to be reset everytime sensor fusion data is updated
          closest_distance_front_car_my_lane = max_cost_front;
          closest_distance_back_car_my_lane = max_cost_back;
          
          closest_distance_front_car_left_lane = max_cost_front;
          closest_distance_back_car_left_lane = max_cost_back;
          
          closest_distance_front_car_right_lane = max_cost_front;
          closest_distance_back_car_right_lane = max_cost_back;
          
          /*
          1. Check there is car too close in boundry at my lane, left/right lane
           - use sensor fusion data
           - have to check every car being sensored
          */
          for (auto &i : sensor_fusion)
          {
             // I will check is there car nearby line by line
            // this is sensor fusion data of d
            float d = i[6];
            
            // 1. check same line
            if ((d < lane_number*4 + 4) && (d > lane_number*4))
            {
              too_close_my_lane = too_close_my_lane || check_too_close(i, car_s, prev_size, direction::my_lane);
            }
            
            // 2. check left line
            if ((lane_number != 0) && (d < lane_number*4) && (d > lane_number*4 - 4))
            {
              too_close_left_lane = too_close_left_lane || check_too_close(i, car_s, prev_size, direction::left_lane);
            }
            
            // 3. check right line
            if ((lane_number != 2) && (d < lane_number*4 + 8) && (d > lane_number*4 + 4))
            {
              too_close_right_lane = too_close_right_lane || check_too_close(i, car_s, prev_size, direction::right_lane);
            }
            
          }
          
          /*
          1. If there is not car nearby in front,
           1) If my velocity is slower than limit, speed up
           2) If my velocity is equal to limit, keep lane
          2. If there is car nearby in front,
           1) Slow down
           2) Prepare to chane lane
           3) Try lane change
          */
          
         if (!too_close_my_lane)
         {
           if ( car_velocity < 49.0)
           {
             car_velocity += 0.5; 
           }
           else
           {
             change_FSM_state(FSM_state::keep_lane);
           }
         }
         else
         {
           car_velocity -= 0.5;
           change_FSM_state(FSM_state::prepare_lane_change);
           try_lane_change(lane_number, car_d, too_close_left_lane, too_close_right_lane);
         }
         
          // Under this line, it is process of gerating path
          // Widely spaced points in 30m apart then fit spline and fill the points to get ref v
          vector<double> pts_x;
          vector<double> pts_y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_angle = deg2rad(car_yaw);

          if(prev_size < 2)
          {
            double car_prev_x = car_x - cos(car_yaw);
            double car_prev_y = car_y - sin(car_yaw);

            pts_x.push_back(car_prev_x);
            pts_x.push_back(car_x);

            pts_y.push_back(car_prev_y);
            pts_y.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_prev_x = previous_path_x[prev_size-2];
            double ref_prev_y = previous_path_y[prev_size-2];

            ref_angle = atan2(ref_y-ref_prev_y,ref_x-ref_prev_x);

            pts_x.push_back(ref_prev_x);
            pts_x.push_back(ref_x);

            pts_y.push_back(ref_prev_y);
            pts_y.push_back(ref_y);
          }
                
          vector<double> nextWP0 = getXY(car_s + 30, (2 + 4 * lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextWP1 = getXY(car_s + 60, (2 + 4 * lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextWP2 = getXY(car_s + 90, (2 + 4 * lane_number), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          pts_x.push_back(nextWP0[0]);
          pts_x.push_back(nextWP1[0]);
          pts_x.push_back(nextWP2[0]);

          pts_y.push_back(nextWP0[1]);
          pts_y.push_back(nextWP1[1]);
          pts_y.push_back(nextWP2[1]);

          for(int i = 0; i < pts_x.size() ; i++)
          {
            // Shift to Car ref angle of 0 degree
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;

            pts_x[i] = (shift_x * cos(0-ref_angle) - shift_y * sin(0-ref_angle));
            pts_y[i] = (shift_x * sin(0-ref_angle) + shift_y * cos(0-ref_angle));
          }

          tk::spline s;

          s.set_points(pts_x,pts_y);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;


          for(int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //Calculate the spacing of points
          double target_x = 30.0;
          double target_y = s(target_x);

          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
          double x_add_on = 0;

          // Fill the rest of the points after filling prev points
          double dist_inc = 0.44;
          for(int i = 1; i < 50-prev_size; i++)
          {
            // N steps req for desired speed = distance/distance/sec ; 2.24 - makes miles per hour to meters per sec
            double N = (target_dist/(0.02 * car_velocity/2.24));

            double x_point = x_add_on + (target_x/N);
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_point_backup = x_point;
            double y_point_backup = y_point;

            // Rotate back to global coordinates
            x_point = (x_point_backup * cos(ref_angle) - y_point_backup * sin(ref_angle));
            y_point = (x_point_backup * sin(ref_angle) + y_point_backup * cos(ref_angle));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
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