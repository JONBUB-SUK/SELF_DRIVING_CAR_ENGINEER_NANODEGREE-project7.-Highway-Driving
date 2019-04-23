# SELF_DRIVING_CAR_ENGINEER_NANODEGREE-project7.-Highway-Driving


# Introduction

## 1. Purpose

Purpose of this project is to drive in highway without any collision or jerk

To do this project, I learned A* algorithm, Naive Bayes for prediction, FSM for behavior planning, trajectory generation


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

My intention for algorithm is very simple

Just make beginner driver!

This is some principles my beginner driver should obey

1. Obey speed limit

2. Keep lane until a car showed up in front

3. Change lane when front is blocked

4. If want change lane, there should be no car near 10m in intended lane behind

5. If want change lane, select lane that have bigger space between front car

And this is Finite State Machine explaining all this principles

<img src="./images/FSM.png" width="600">


# Background Learning


### 1. Total Subject Flow

This project is collection of Self Driving Car projects

Every concept I learned until now is represented in this project

Sensor fusion is used to detect any objects using radar/ridar

Localization is used to find my exact position

Search is used to find optimal path to destination

Prediction is used to predict next behavior of sensored objects

Behavioral planning is used to planning my action against predicted object's behavior

Trajectory generation is finally used to make trajectory for my final action

<img src="./images/total_subject_flow.jpg" width="400">


### 2. Search

For searching optimal path to destination, I learned A* & A* hibrid algorithm

A* hibrid algorithm is equal to A* but is different at continuous moving

<img src="./images/search.jpg" width="400">

### 3. Prediction

To drive manually, after got map data and knowing where the other cars are, I will curious about what will they do

There are two approaches for prediction

First method is called Model-based approach, using mathmatics and physics

Second method is called Data-driven approach

It needs so many labeled data

Then by using Naive Bayes Classifier, we can predict next action of detected objects

<img src="./images/prediction.jpg" width="400">
<img src="./images/prediction_2.jpg" width="400">
<img src="./images/prediction_3.jpg" width="400">
<img src="./images/prediction_4.jpg" width="400">

### 4. Behavior Planning

After Knowing what the other car do, I have to decide then, what will I do next?

To do I have to define Finite State Machine

That is method to define all the cases can happen in driving situation

In other words, if there happen the case I didn't define, there will be accident

So it will useful just like this highway simulation case

<img src="./images/behavior_planning.jpg" width="400">

### 5. Trajectory Generation

Making a trajectory after deciding action is just math

But have to careful about not to make jerk or rapid acceleration

Make polynimial path through point I decided

<img src="./images/trajectory_generation.jpg" width="400">
<img src="./images/trajectory_generation_2.jpg" width="400">
<img src="./images/trajectory_generation_3.jpg" width="400">
<img src="./images/trajectory_generation_4.jpg" width="400">
<img src="./images/trajectory_generation_5.jpg" width="400">
<img src="./images/trajectory_generation_6.jpg" width="400">

# Content Of This Repo
- ```src``` a directory with the project code
	- ```main.cpp``` : communicate with simulator, reads in data, calls a function in helpers.h to drive
	- ```helpers.h``` : have functions have to be used in main.cpp



# Flow

## 1. Flow Chart

Before makeing code, I made flow chart to check total flow and check what function will be need

##### main.cpp
<img src="./images/flow_chart_main.png" width="800">

##### check_too_close function
<img src="./images/flow_chart_check_too_close.png" width="800">

##### try_lane_change
<img src="./images/flow_chart_try_lane_change.png" width="800">

##### lane_change_cost
<img src="./images/flow_chart_lane_change_cost.png" width="800">


# Results

It drove well without ant jerks or collides

<img src="./images/result_1.png" width="700">
<img src="./images/result_2.png" width="700">


# Conclusion & Discussion

### 1. About limitation

In this project, I did not use search and prediction

Maybe it can be apart from the intention of project

My algorithm is just check there is car nearby and make trajectory to change my lane

It can be useful because this project is just driving in simple highway

I didn't have to calculate optimal path to destination,

I didn't have to predict opponent car's behavior at intersection

So I am very looking forward to final project, driving autonomously at real city








