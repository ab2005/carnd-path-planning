
 # Path Planning Project
 
In this project we have to develop a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane virtual highway with other traffic that is  driving **-+10 MPH** of the 50 MPH speed limit.

## The Project Goals
1. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic 
when possible, note that other cars will try to change lanes too. 
2. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, 
 unless going from one lane to another. 
3. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, 
 it should take a little over 5 minutes to complete 1 loop. 
4. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Project implementation 

### Understanding track data
The car's localization and sensor fusion data as well as a sparse map list of waypoints around the highway 
is provided as [text file](data/highway_map.csv).

| x | y |s | dx | dy |
| --- | --- | --- | --- | --- |
| 784.6001 | 1135.571 | 0 | -0.02359831 | -0.9997216 |
| 815.2679 | 1134.93 | 30.6744785308838 | -0.01099479 | -0.9999396 | 
| 844.6398 | 1134.911 | 60.0463714599609 | -0.002048373 | -0.9999979 | 
| 875.0436 | 1134.808 | 90.4504146575928 | -0.001847863 | -0.9999983 | 

Each waypoint in the list contains  [x, y, s, dx, dy] values. 
Where: 
 - x and y are the waypoint's map coordinate position, 
 - the s value is the distance along the road to get to that waypoint in meters. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.
 - the dx and dy values define the unit normal vector pointing outward of the highway loop. 

### Understanding simulator data
While running our program we'll have updates from the simulator in the form of a JSON object:
```json
["telemetry",
  { 
    "x":909.48,
    "y":1128.67,
    "s":124.8336,
    "d":6.164833,
    "speed":0,
    "yaw":0,    
    
    "end_path_d":0,
    "end_path_s":0,
    "previous_path_x":[],
    "previous_path_y":[],
    
    "sensor_fusion":[
      [0,844.6275,1128.911,53.04249,-0.108651,60.05444,6.000007],
      [1,1028.854,1148.57,41.2536,17.08123,243.9722,9.999819],
      [2,775.8,1429,0,0,6716.599,-282.9019],
      [3,775.8,1432.9,0,0,6713.911,-285.7268],
      [4,775.8,1436.3,0,0,6711.566,-288.1896],
      [5,775.8,1441.7,0,0,6661.772,-291.7797],
      [6,762.1,1421.6,0,0,6711.778,-268.0964],
      [7,762.1,1425.2,0,0,6709.296,-270.7039],
      [8,762.1,1429,0,0,6663.543,-273.1828],
      [9,762.1,1432.9,0,0,6660.444,-275.5511],
      [10,762.1,1436.3,0,0,6657.743,-277.6157],
      [11,762.1,1441.7,0,0,6653.453,-280.8947]
    ],
  }
]

```
 1. Main car's localization Data (No Noise)
   - "x" The car's x position in map coordinates
   - "y" The car's y position in map coordinates
   - "s" The car's s position in frenet coordinates
   - "d" The car's d position in frenet coordinates
   - "yaw" The car's yaw angle in the map
   - "speed" The car's speed in MPH
 2. Previous path data given to the Planner
   - "previous_path_x" The previous list of x points previously given to the simulator
   - "previous_path_y" The previous list of y points previously given to the simulator
   - *Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time.*
 3. Previous path's end s and d values
   - "end_path_s" The previous list's last point's frenet s value
   - "end_path_d" The previous list's last point's frenet d value  
 4. Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
   - "sensor_fusion" A 2d vector of cars:  
     - car's unique ID,
     - car's x position in map coordinates, 
     - car's y position in map coordinates, 
     - car's x velocity in m/s, 
     - car's y velocity in m/s, 
     - car's s position in frenet coordinates, 
     - car's d position in frenet coordinates.

### Loading track waypoints
```c++
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
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;
  while (getline(in_map_, line)) {
  	 istringstream iss(line);
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

## The Trajectory

We'll use a simple spline-based method to generate jerk minimized trajectory. Following are steps on creating the car trajectory on every simulator iteration:

#### 1. Creating a list of widedy spaced points, evenly spaced at 30 meters for spline fit
Spline fits 5 points in Cartesian spacee: two behing and three ahead of the car we control. Two points behind are taken from the end of the previous path provided by the simulator. On the car starting point or when previous path is not available we compute the previous points using a trigonometric projection:
```c++
// Create a list of widedy spaced points, evenly spaced at 30 meters.
// Later we'll interpolate these points with spline and use to generate next points.
vector<double> pts_x;
vector<double> pts_y;

// Reference states: it is eather where the car is or it is the previious path end point.
double ref_x = car_x;
double ref_y = car_y;
double ref_x_prev = car_x - cos(car_yaw);
double ref_y_prev = car_y - sin(car_yaw);
double ref_yaw = deg2rad(car_yaw);

if (prev_size > 1) {
  ref_x = previous_path_x[prev_size - 1];
  ref_y = previous_path_y[prev_size - 1];
  ref_x_prev = previous_path_x[prev_size - 2];
  ref_y_prev = previous_path_y[prev_size - 2];
  ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
}

pts_x.push_back(ref_x_prev);
pts_x.push_back(ref_x);
pts_y.push_back(ref_y_prev);
pts_y.push_back(ref_y);
```

For the future points we add evenly 30 meters spaced points ahead of the starting reference in Frenet space:
```c++
vector<double> next_wp0 = getXY(car_s + 30, lane_center, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, lane_center, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, lane_center, map_waypoints_s, map_waypoints_x, map_waypoints_y);
pts_x.push_back(next_wp0[0]);
pts_x.push_back(next_wp1[0]);
pts_x.push_back(next_wp2[0]);
pts_y.push_back(next_wp0[1]);
pts_y.push_back(next_wp1[1]);
pts_y.push_back(next_wp2[1]);
```          

#### 2. Interpolating sparce points with spline and generating future path points

In order to generate evenly distributed path points we will shift coordinates and rotate car reference angle to 0 degrees:
```c++
for (int i = 0; i < pts_x.size(); i++) {
  double shift_x = pts_x[i] - ref_x;
  double shift_y = pts_y[i] - ref_y;
  pts_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
  pts_y[i] = shift_y * cos(-ref_yaw) + shift_x * sin(-ref_yaw);
}
```
Then we will use a spline to fill the actual x, y points to use for the planner:
```c++
tk::spline s;
s.set_points(pts_x, pts_y);
```
We calulate how to break up spline point so that we travel at our desired reference velocity by using simple equasion:
```c++
double target_x = 30.;
double target_y = s(target_x);
double target_dist = sqrt(target_x * target_x + target_y * target_y);
double x_add_on = 0;
double step = target_dist / (.02 * ref_velocity / 2.24);

vector<double> next_x_vals;
vector<double> next_y_vals;
```
We start filling the path planner with available previous points (the points car actually moved):
```c++
for (int i = 0; i < previous_path_x.size(); i++) { 
  next_x_vals.push_back(previous_path_x[i]);
  next_y_vals.push_back(previous_path_y[i]);
}
```
Then we fill up the rest of our path planner by rotating back to normal corrdinates since we've rotated it earlier. 
Note: here we'll always output 50 points.
```c++
for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
  double x_point = x_add_on + target_x / step;
  double y_point = s(x_point);
  x_add_on = x_point;

  double x_ref = x_point;
  double y_ref = y_point;
  // rotate back to normal after rotating it earlier
  x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
  y_point = y_ref * cos(ref_yaw) + x_ref * sin(ref_yaw);
  x_point += ref_x;
  y_point += ref_y;

  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}
```

## The Planner

Our implementation uses a simple planning scheme with a state machine to control the car. The car will prefer to stay in the current lane and drive in constant speed close to speed limit. If there is a car ahead in a close proximity with a slower speed then we check if it is safe to change to the left ot right lane. If both lanes are available for safe passing of the car ahead we chose the lane with the highest score. The score is computed based on the closest cars speed and distance. If passing on both left and right lanes is not safe then we slow down and remain in the same lane following the car ahead accelerating only when we'll have enough space ahead. 


## Result and Reflection

There are two parts of this project: one is how to generate smooth trajectories that meet the minimum requirement, another one is how to plan a feasible trajectory for the car to drive as fast as possible below the speed limit avoiding incidents. Big parts of effort are generating smooth trajectories and tuning up state transitioining. We were able to run 48 miles without accidents on the project track [`data/highway_map.csv`](data/highway_map.csv) using [Term 3 Simulator v1.2](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). 
![path_planning_project_track](path_planning_project_track.jpg)

We also were able to run 4 out 5 races on Bosch'c Path Planning Challenge track [`data/highway_map_bosch1.scv`](data/highway_map_bosch1.scv) using [Bosch Challenge Track 1 simulator](https://github.com/udacity/Bosch-Challenge/releases/tag/v1.0)   
![path_planning_bosch_track](path_planning_bosch_track.jpg)

Future work should include:

Combine with MPC to find optimized trajectory. With MPC, using cost function we will be able to limit the speed and acceleration, and their variations.
Implement a complete state machine to handle different sceneraios that could cause accidents. More traffic information should be incorporated in making decision on state change.
Looking into ML technology on making decision.
