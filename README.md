# SELF_DRIVING_CAR_ENGINEER_NANODEGREE-project7.-Highway-Driving


# Introduction

## 1. Purpose

Purpose of this project is to drive in highway without any collision or jerk

To do this project, I learned A* algorithm, Naive Bayes for prediction, FSM for behavior planning, trajectory generation

But all of this learning is used in this project

Because it is relatively easy that drive in highway than in complex city

## 2. Rubric points

1. Should drive at least 4.32 miles

2. Should obey speed limit

3. Max acceleration and jerk should not exceed

4. Car should not have collisions

5. The car should stay in its lane, except for the time between changing lanes.

6. The car is able to change lanes

## 3. Input data given by simulator / Output data for simulator

1. Input

- Map data for .csv ( x, y, s, dx, dy )
- Localization data ( x, y, s, d, yaw, speed )
- Previous path data ( remaining path map data )
- Sensor fusion data ( id, x, y, vx, vy, s, d ) 

2. Ouput
- Next position calulated by my algorithm ( x, y ) 

## 4. My intention of algorithm

My intent for algorithm is very simple

Just make beginner driver!

This is some principles my beginner driver should obey

1. Obey speed limit

2. Keep lane until a car showed up in front

3. Change lane when front is blocked

4. If want change lane, there should be no car near 10m in intended lane behind

5. If want change lane, select lane that have bigger space between front car

And this is Finite State Machine for this principles

<img src="./images/FSM.png" width="400">


# Background Learning

For this project, I had to learn principle of Particle-Filter

### 1. Motion models

- Assumption for bicycle model

<img src="./images/motion_models.jpg" width="500">




# Content Of This Repo
- ```src``` a directory with the project code
	- ```main.cpp``` : communicate with simulator, reads in data, calls a function in helpers.h to drive
	- ```helpers.h``` : have functions have to be used in main.cpp



# Flow

## 1. Flow Chart

<img src="./images/flow_chart.png" width="500">

## 2. Code Explanation

1. Read map data

This map data is used to calculate path to follow lane

```c++
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

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
```


# Results

<img src="./images/result.png" width="700">


# Conclusion & Discussion

### 1. About total flow of self-driving car

What do we need to make self driving car?








