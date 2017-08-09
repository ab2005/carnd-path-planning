
## TODO: Download Simulator 
Download the Term3 Simulator which contains the Path Planning Project from 
https://github.com/udacity/self-driving-car-sim/releases.

## The Project Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is 
driving **+-10 MPH** of the 50 MPH speed limit. 

### Data
The car's localization and sensor fusion data as well as a sparse map list of waypoints around the highway 
is provided as [text file](data/highway_map.csv).

| x | y |s | dx | dy |
| --- | --- | --- | --- | --- |
| 784.6001 | 1135.571 | 0 | -0.02359831 | -0.9997216
| 815.2679 | 1134.93 | 30.6744785308838 | -0.01099479 | -0.9999396 | 
| 844.6398 | 1134.911 | 60.0463714599609 | -0.002048373 | -0.9999979 | 
| 875.0436 | 1134.808 | 90.4504146575928 | -0.001847863 | -0.9999983 | 

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, 
the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit 
normal vector pointing outward of the highway loop.\

### Goals
 1. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic 
when possible, note that other cars will try to change lanes too. 


2. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, 
 unless going from one lane to another. 
3. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, 
 it should take a little over 5 minutes to complete 1 loop. 
4. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.
