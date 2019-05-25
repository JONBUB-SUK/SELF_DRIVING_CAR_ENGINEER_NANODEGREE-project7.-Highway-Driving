# SELF DRIVING CAR NANODEGREE
# project7. Highway Driving

## 1. Abstraction

The purpose of this project is to drive in highway without any collision or jerk

To do that, I studied A* algorithm, Naive Bayes for prediction, FSM for behavior planning, trajectory generation

The rubric points of this project is like below

- Should drive at least 4.32 miles
- Should obey speed limit
- Max acceleration and jerk should not exceed
- Car should not have collisions
- The car should stay in its lane, except for the time between changing lanes.
- The car is able to change lanes

And I receive 4 kinds of input data from simulator

- Map data for .csv ( x, y, s, dx, dy )
- Localization data ( x, y, s, d, yaw, speed )
- Previous path data ( remaining path map data )
- Sensor fusion data ( id, x, y, vx, vy, s, d ) 

Then I have to make next position (x,y) as output

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


## 2. Related Study

#### 1) Total Subject Flow

This project is collection of Self Driving Car projects

Every concept I learned until now is represented in this project

Sensor fusion is used to detect any objects using radar/ridar

Localization is used to find my exact position

Search is used to find optimal path to destination

Prediction is used to predict next behavior of sensored objects

Behavioral planning is used to planning my action against predicted object's behavior

Trajectory generation is finally used to make trajectory for my final action


#### 2) Search

For searching optimal path to destination, I learned A* & A* hibrid algorithm

A* hibrid algorithm is equal to A* but is different at continuous moving


#### 3) Prediction

To drive manually, after getting map data and knowing where the other cars are, I will curious about what will they do

There are two approaches for prediction

First method is called Model-based approach, using mathmatics and physics

Second method is called Data-driven approach

It needs so many labeled data

Then by using Naive Bayes Classifier, we can predict next action of detected objects

#### 4) Behavior Planning

After Knowing what the other car do, I have to decide then, what will I do next?

To do that, I have to define Finite State Machine

That is method to define all the cases can happen in driving situation

In other words, if there happen the case I didn't define, there will be accident

So it will useful just like this highway simulation case


#### 5) Trajectory Generation

Making a trajectory after deciding action is just math

But have to careful about not to make jerk or rapid acceleration

Make polynimial path through point I decided


## 3. Details

#### 1) Content Of This Repo

- ```src``` a directory of the project code
	- ```main.cpp``` : communicate with simulator, reads in data, calls a function in helpers.h to drive
	- ```helpers.h``` : have functions have to be used in main.cpp

#### 2) Code

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

