#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::endl;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

/*******************************************************************/
/* front/back car cost value that will be used at calculating cost */
/*******************************************************************/
const int max_cost_front = 50;
const int max_cost_back = 30;

/*******************************************************************/
/* Distance between cars */
/*******************************************************************/
double closest_distance_front_car_my_lane = max_cost_front;
double closest_distance_back_car_my_lane = max_cost_back;

double closest_distance_front_car_left_lane = max_cost_front;
double closest_distance_back_car_left_lane = max_cost_back;

double closest_distance_front_car_right_lane = max_cost_front;
double closest_distance_back_car_right_lane = max_cost_back;

/*******************************************************************/
/* Define direction */
/*******************************************************************/
enum direction
{
  my_lane,
  left_lane,
  right_lane
};

/*******************************************************************/
/* FSM states */
/*******************************************************************/
enum FSM_state
{
  keep_lane,
  prepare_lane_change,
  change_to_left_lane,
  change_to_right_lane,
};

// We need to initiate current FSM state
FSM_state current_FSM_state = FSM_state::keep_lane;


void print_FSM_state(FSM_state fsm)
{
    if(fsm == FSM_state::keep_lane)
    {
        cout << "Current FsmState :: Keep Lane :: "<< endl;
    }
    else if(fsm == FSM_state::prepare_lane_change)
    {
        cout << "FsmState :: Prepare Lane Change : Front Car too close : Decrease Speed : Try Lane change "<< endl;
    }
    else if(fsm == FSM_state::change_to_left_lane)
    {
        cout << "FsmState :: Change Lane Left : Left initiated " << endl;
    }
    else if(fsm == FSM_state::change_to_right_lane)
    {
        cout << "FsmState :: Change Lane Right : Right initiated " << endl;
    }
}

/*******************************************************************/
/* change FSM states */
/*******************************************************************/
void change_FSM_state(FSM_state fsm)
{
  current_FSM_state = fsm;
  print_FSM_state(current_FSM_state);  
}

/*******************************************************************/
/* Update closest distance values */
// We will calculate distance for every sensor fusion data
// So if this value is closer than before, we need to updata closest value
/*******************************************************************/
void update_closest_distance (direction dir, double front_car_distance, double back_car_distance)
{
  if (dir == direction::my_lane)
  {
    if (closest_distance_front_car_my_lane > front_car_distance)
    {
       closest_distance_front_car_my_lane = front_car_distance;
    }
    
    if (closest_distance_back_car_my_lane > back_car_distance)
    {
       closest_distance_back_car_my_lane = back_car_distance;
    }
  }
  
  else if (dir == direction::left_lane)
  {
    if (closest_distance_front_car_left_lane > front_car_distance)
    {
       closest_distance_front_car_left_lane = front_car_distance;
    }
    
    if (closest_distance_back_car_left_lane > back_car_distance)
    {
       closest_distance_back_car_left_lane = back_car_distance;
    }
  }
  
  else if (dir == direction::right_lane)
  {
    if (closest_distance_front_car_right_lane > front_car_distance)
    {
       closest_distance_front_car_right_lane = front_car_distance;
    }
    
    if (closest_distance_back_car_right_lane > back_car_distance)
    {
       closest_distance_back_car_right_lane = back_car_distance;
    }
  }
}

/*******************************************************************/
/* Retrieve closest distance values */
// When perform check_too_close function, we need to remember closest distance value
/*******************************************************************/
void retrieve_distances(direction dir, double &frontCarDist, double &backCarDist)
{
    if(dir == direction::my_lane)
    {
        frontCarDist = closest_distance_front_car_my_lane;
        backCarDist = closest_distance_back_car_my_lane;
    }
    else if(dir == direction::left_lane)
    {
        frontCarDist = closest_distance_front_car_left_lane;
        backCarDist = closest_distance_back_car_left_lane;
    }
    else if(dir == direction::right_lane)
    {
        frontCarDist = closest_distance_front_car_right_lane;
        backCarDist = closest_distance_back_car_right_lane;
    }
}


/*******************************************************************/
/* Check there is car nearby too close */
// 1. If that car is at front
//  1) If it is closer in 30m
//   : true
//    - because when lane changing, the front car position in anyline is important
// 2. If that car is at back
//  1) If it is in same line
//   : ignore, I don't care
//  2) If it is in different lane and closer in 10m
//   : true
/*******************************************************************/
bool check_too_close(vector<double> sensor_fusion, double car_s, int prev_size, direction dir)
{
  double vx = sensor_fusion[3];
  double vy = sensor_fusion[4];
  double predicted_next_s = sensor_fusion[5];
  double sensored_velocity = sqrt(vx*vx + vy*vy);
  
  // it is predicted position of this sensored car
  predicted_next_s += (double) prev_size * sensored_velocity * 0.02;
  
  // initiate distance variables
  // we will updata this value to closest_distance values to calculate cost later
  double front_car_distance = max_cost_front;
  double back_car_distance = max_cost_back;
  
  retrieve_distances(dir, front_car_distance, back_car_distance);
  
  bool result = false;
  
  if (predicted_next_s > car_s)
  {
    front_car_distance = predicted_next_s - car_s;
    if (front_car_distance < 30)
    {
      result = true;
    }
  }
  else
  {
    back_car_distance = car_s - predicted_next_s;
    if (dir == direction::my_lane)
    {
      result = false;
    }
    else
    {
      if (back_car_distance <= 10)
      {
        result = true;
      }
    }
  }
  update_closest_distance(dir, front_car_distance, back_car_distance);
  return result;
  
}

/*******************************************************************/
/* Calculate cost of lane change */
/*******************************************************************/
double lane_chage_cost (int lane, direction wanna_direction)
{
  double cost = 100;
  
  // 1. If my car is in the first lane
  if (lane == 0)
  {
    // we cannot go more left
    if (wanna_direction == direction::left_lane)
    {
      cost = 100;
    }
    // calculate cost by diffrence between max distance, min distance
    else if (wanna_direction == direction::right_lane)
    {
      cost = (max_cost_front - closest_distance_front_car_right_lane);
    }
  }
  
  // 2. If my car is in the second lane
  else if (lane == 1)
  {
    // calculate cost by diffrence between max distance, min distance
    if (wanna_direction == direction::left_lane)
    {
      cost = (max_cost_front - closest_distance_front_car_left_lane);
    }
    // calculate cost by diffrence between max distance, min distance
    else if (wanna_direction == direction::right_lane)
    {
      cost = (max_cost_front - closest_distance_front_car_right_lane);
    }
  }
  
  // 3. If my car is in the second lane
  else if (lane == 2)
  {
    // calculate cost by diffrence between max distance, min distance
    if (wanna_direction == direction::left_lane)
    {
      cost = (max_cost_front - closest_distance_front_car_left_lane);
    }
    // we cannot go more right
    else if (wanna_direction == direction::right_lane)
    {
      cost = 100;
    }
  }
  
  return cost;
  
}



/*******************************************************************/
/* Try lane change when front car is block me */
/*******************************************************************/
void try_lane_change (int &lane_number, double car_d, bool too_close_left_lane, bool too_close_right_lane)
{
  double left_change_cost = lane_chage_cost(lane_number, direction::left_lane);
  double right_change_cost = lane_chage_cost(lane_number, direction::right_lane);
  
  //1. 왼쪽 비용이 더 작고, 비어있으면 왼쪽으로 가라
  // 1. If left cost is lower than right. and it has no car nearby, go to left
  if (( left_change_cost < right_change_cost) && (!too_close_left_lane))
  {
    lane_number--;
    change_FSM_state(FSM_state::change_to_left_lane);
  }
  // 2. If right cost is lower than left, and it has no car nearby, go to right
  else if (( left_change_cost > right_change_cost) && (!too_close_right_lane))
  {
    lane_number++;
    change_FSM_state(FSM_state::change_to_right_lane);
  }
  // 왼쪽 오른쪽 모두 비어있을때는 왼쪽으로 가라
  else if ((left_change_cost == 0) && (right_change_cost == 0) && (!too_close_left_lane))
  {
    lane_number--;
    change_FSM_state(FSM_state::change_to_left_lane);
  } 
  else
  {
    std::cout<<"!!!!!!!!!!!!!!! Lane Change is Not Safe !!!!!!!!!!!!!!!!!!"<<std::endl;
  }
  
  std::cout<< "Left Change Cost : "<<left_change_cost<< endl;
  std::cout<< "Right Change Cost : "<<right_change_cost<< endl;
  std::cout<< "Too Close Left Lane : "<<too_close_left_lane<< endl;
  std::cout<< "Too Close Right Lane : "<<too_close_right_lane<< endl;
}




#endif  // HELPERS_H