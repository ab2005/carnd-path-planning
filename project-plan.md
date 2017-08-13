
 ### TODO
 1. [ ] Download the Term3 Simulator which contains the Path Planning Project from [releases](https://github.com/udacity/self-driving-car-sim/releases/download/T3_v1.2/term3_sim_mac.zip)
 2. [ ] Install openssl, libuv, cmake, zlib. Build uWebSockets
 3. [ ] build, run
  ```
  mkdir build && cd build
  cmake .. && make
  ./path_planning
  ```
## The Project Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is 
driving **-10 MPH** of the 50 MPH speed limit.
1. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic 
when possible, note that other cars will try to change lanes too. 
2. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, 
 unless going from one lane to another. 
3. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, 
 it should take a little over 5 minutes to complete 1 loop. 
4. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

### Track data
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

### Simulator data
We have updates from the simulator as a JSON object:
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

#### Load up track values
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

### Trajectory generation

We use a simple spline-based method to generate jerk minimized trajectory. Following are steps on creating the car trajectory on every simulator iteration:

Spline fit 5 points: two behing and three ahead in Cartesian space. Two points behind are taken from the end of the previous path provided by the simulator. In the beginning or when previous path is empty we compute the current car state using a trigonometric projection:
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

As a future points we add in Frenet evenly 30 meters spaced points ahead of the starting reference:

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

Create a path planner that performs optimized lane changing, this means that the car only changes into a lane that improves its forward progress.

