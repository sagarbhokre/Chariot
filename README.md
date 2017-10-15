
# Chariot
A path planning algorithm implementation 

For setup instructions please refer to the following repository
https://github.com/udacity/CarND-Path-Planning-Project

Code outline:
The entire code can be divided into two major blocks, 
1. Trajectory generation 
2. Behavioral Planning

#### Trajectory generation
Input received from the simulator is processed using API process_data()
  This API computes the next values in the vector for maneuvering the car in the simulator environment
  Values are computed based on following logic:
  1) Get current position and orientation of car. This could be extracted from car_x, car_y, car_yaw at start or from the left over path vector from the previous iteration
  2) Copy the leftovers from previous path
  3) Create a list of points through which a curve needs to be fit. The points considered here are car position(at start) or end point coordinates from the previous iteration, the point previous to this point, 3 more points looking ahead in the Lane spaced 50m ahead of the last point.
  4) Convert this list of points into local coordinates
  5) Fit a spline through these points
  6) Depending on the velocity of car, append remaining points to the partially filled list from step b)
  7) Convert these points back to global coordinate space and append them to next_x_vals and next_y_vals vector list

Figure below illustrates how trajectory is generated using spline tool.
![Alt text](imgs/Trajecotry_generation.png?raw=true "Trajectory generation")

#### Behavioral planning
After the points are created for the car to follow, next step is to check if the car is safe in the existing lane or would it need to switch lanes to move faster without influence of traffic. check_lane_change() API does the task of handling lane change requirements

  1) Here all the cars detected by sensor_fusion module are segregated into different lanes
  2) Considering that a car is at least 2m wide and each lane is 4 m wide, while changing lanes a car with centre at distance 3 from centre of a 6 lane road could be considered in both lane 0 and lane 1. Following are the boundaries for deciding which car influences which lanes
    range<0 >= x < 5>  --- lane 0
    range<3 >= x < 9>  --- lane 1
    range<7 >= x < 12> --- lane 2
  3) In each lane the closest car to the s dance of our car is computed to check which lane is safe to stay in. To be safe in a particular lane, the car ahead of our car should be at least 30 m ahead and car behind our car should be at least 7 m behind. The same constraint applies to all the 3 lanes
  4) Best of the three lanes is computed depending on the free space avaliable in front of the car. Larger the free space in front, more the likelihood that car would maneuver to that lane
  5) All the lanes are considered for deciding if a lane shift is required. Preference is given to stay in the same lane if difference is not significant.
  6) While switching lanes, the car is slowed down and then maneuvered to destination lane. slowdown mode ensures there are no abrupt velocity/acceleration changes
  7) If the lane change is not required, car tries to accelerate and drive at max speed (48 mph here)
  8) After a lane change, the car stays in cooldown mode for aroud ~1.5 sec; where it stays in the same.

Illustration below shows how the car sees free space in the middle lane and switches to this lane to move faster towards goal.
![Alt text](imgs/behavioral_planning.png?raw=true "Behavioral planning")
