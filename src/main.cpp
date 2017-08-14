#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

const double MAX_SPEED = 50.;

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  }
  if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/**
 * Get nearest waypoint (behind or in front)
 */
int getClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y){
  double closestLen = 100000; //large number
  int closestWaypoint = 0;
  for (int i = 0; i < maps_x.size(); i++) {
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
/**
 *
 * @Params: theta - angle
 *          maps_x, maps_y - sparse waypoints
 */
int getNextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y){
  int closestWaypoint = getClosestWaypoint(x, y, maps_x, maps_y);
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  double heading = atan2(map_y - y, map_x - x);
  double angle = abs(theta - heading);
  if (angle > pi() / 4) {
    closestWaypoint++;
  }
  return closestWaypoint;
}

/**
 * Transform from Cartesian x, y coordinates to Frenet s, d coordinates.
 *
 */
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
  int next_wp = getNextWaypoint(x,y, theta, maps_x,maps_y);
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
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);
  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }
  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }
  frenet_s += distance(0, 0, proj_x, proj_y);
  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
  int prev_wp = -1;
  if (maps_x.size() == 0) {
    cout << "maps_x is empty" << endl;
    return {0., 0.};
  }
  if (maps_y.size() == 0) {
    cout << "maps_y is empty" << endl;
    return {0., 0.};
  }
  if (maps_s.size() == 0) {
    cout << "maps_s is empty" << endl;
    return {0., 0.};
  }
  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
    prev_wp++;
  }
  int wp2 = (prev_wp+1)%maps_x.size();
  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);
  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
  double perp_heading = heading-pi()/2;
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  return {x,y};
}

void update_gap(double our_car_s, double another_car_s, long gap[], long max_s) {
  long distance = another_car_s - our_car_s;
  if (distance > max_s / 2) {
    distance -= max_s;
  } else if (distance < -max_s / 2) {
    distance += max_s;
  }
  if (distance < 0 && distance > gap[0]) {
    // is the car is behind us
    // TODO: check for max track distance!
    gap[0] = distance;
  } else if (distance >= 0 && distance < gap[1]) {
    // if the cat is ahead of us
    gap[1] = distance;
  }
}

// returns 0 or negative if no space
double get_lane_score(double gap[], double speed[], double car_speed) {
  // check for space
  if (gap[0] > -10. || gap[1] < 30.) return 0;
  return gap[1];
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
  //string map_file_ = "../data/highway_map_bosch1.scv";
  
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
  
  // start in lane 1
  int lane = 1;
  // target velocity = 49.5 mph
  double ref_velocity = 0.;
  
  h.onMessage([&ref_velocity,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          int prev_size = previous_path_x.size();
          // each lane is 4 meters wide
          double lane_center = 4. * (double)lane + 2.;
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          bool is_target_lane = true;//car_d < lane_center + 2 && car_d > lane_center - 2;
          
          bool too_close = false;
          bool change_lane = false;
          double left_gap[] =  {-10000., 10000.};
          double right_gap[] = {-10000., 10000.};
          if (lane == 0) {
            left_gap[0] = left_gap[1] = 0.;
          }
          if (lane == 2) {
            right_gap[0] = right_gap[1] = 0.;
          }
          double left_speed[] = {0., 100.};
          double right_speed[] = {0., 100.};
          double free_space_ahead = 0;
          double car_s_now = j[1]["s"];
          
          // Compute front car distance and left/right gaps
          for (int i = 0; i < sensor_fusion.size(); i++) {
            double check_car_d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_fusion[i][5];
            // if using previous points can project a value outward in time
            // we are looking where the car is in the future: 1 sec and 2 sec
            double check_car_s1 = check_car_s  + (double)prev_size * .02 * check_speed;
            // negative if behind
            double check_car_distance_now = check_car_s - car_s_now;
            double check_car_distance_future = check_car_s1 - car_s;

            if (check_car_d < lane_center + 2 && check_car_d > lane_center - 2) {
              // if car in my lane
              if (check_car_distance_now > 0    // if car is ahead of us
                    && check_speed < car_speed  // if car moves slower
                    && check_car_s1 > car_s     // if car is still ahead in the future
                    && check_car_distance_future < 30) {
                change_lane = true;
                too_close = true;
                free_space_ahead = check_car_s1 - car_s;
              }
            } else if (check_car_d < lane_center - 2 && check_car_d > lane_center - 6) {
              // if car in left lane
              if (check_car_distance_now < 0    // the car is behind us
                    && check_car_distance_future > left_gap[0]) {
                left_gap[0] = check_car_distance_future;
                left_speed[0] = check_speed;
              } else if (check_car_distance_now >= 0 // if the car is ahead of us
                   && check_car_distance_future < left_gap[1]) {
                left_gap[1] = check_car_distance_future;
                left_speed[1] = check_speed;
              }
            } else if (check_car_d < lane_center + 6 && check_car_d > lane_center + 2) {
              // if car in right lane
              if (check_car_distance_now < 0   // the car is behind us
                    && check_car_distance_future > right_gap[0]) {
                right_gap[0] = check_car_distance_future;
                right_speed[0] = check_speed;
              } else if (check_car_distance_now  >= 0  // the car is ahead of us
                    && check_car_distance_future < right_gap[1]) {
                right_gap[1] = check_car_distance_future;
                right_speed[1] = check_speed;
              }
            }
          }
          if (is_target_lane) {
            if (change_lane) {
              free_space_ahead += 0;
              double left_score = get_lane_score(left_gap, left_speed, car_speed); // returns negative if no space
              double right_score = get_lane_score(right_gap, right_speed, car_speed); // returns negative if no space
              cout << "[" << left_score << "-" << right_score << "] " << left_gap[0] << ".." << left_gap[1] << " : "<< right_gap[0] << ".." << right_gap[1] << endl;
              double delta = right_score - left_score;
              if (delta > 0 && lane < 2) {
                // move to the right
                cout << "move to the right " << delta << endl;
                lane++;
              } else if (delta < 0 && lane > 0) {
                // move to the left
                cout << "move to the left " << delta << endl;
                lane--;
              } else if (delta == 0 && right_score > 0 ) {
                if (lane > 0) {
                    lane--;
                } else if (lane < 2) {
                    lane++;
                }
              } else {
                cout << lane_center - car_d;
                if (too_close) {
                  cout << " slow down: " << ref_velocity;
                  if (ref_velocity > .112) {
                    ref_velocity -= .112;
                  } else if (ref_velocity < MAX_SPEED - .224) {
                    // accelerate
                    ref_velocity += .224;
                  }
                }
                cout << endl;
              }
            } else if (ref_velocity < MAX_SPEED - .336) {
              // accelerate
              ref_velocity += .336;
            } else {
              // enjoy the ride
            }
          } else {
            cout << lane_center - car_d;
            if (too_close) {
              cout << " slow down " << ref_velocity;
              if (ref_velocity > .112) {
                ref_velocity -= .112;
              } else if (ref_velocity < MAX_SPEED - .224) {
                // accelerate
                ref_velocity += .224;
              }
            }
            cout << endl;
          }
          
          // Create a list of widedy spaced points, evenly spaced at 30 meters
          // Later we'll interpolate these points with spline and use to generate next points
          vector<double> pts_x;
          vector<double> pts_y;
          
          // reference states: it is eather where the car is or it is the previious path end point
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
          
          // In Frenet add evenly 30 meters spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, lane_center, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, lane_center, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, lane_center, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          pts_x.push_back(next_wp0[0]);
          pts_x.push_back(next_wp1[0]);
          pts_x.push_back(next_wp2[0]);
          pts_y.push_back(next_wp0[1]);
          pts_y.push_back(next_wp1[1]);
          pts_y.push_back(next_wp2[1]);
          
          // Shift and rotate car reference angle to 0 degrees
          for (int i = 0; i < pts_x.size(); i++) {
            double shift_x = pts_x[i] - ref_x;
            double shift_y = pts_y[i] - ref_y;
            pts_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            pts_y[i] = shift_y * cos(-ref_yaw) + shift_x * sin(-ref_yaw);
            if (i > 0 && pts_x[i] < pts_x[i - 1]) {
              int k = 0;
              for (int j = i - 1; j >= 0; j--) {
                pts_x[j] = pts_x[i] - ++k;
              }
            }
          }
          //          for (int i = 0; i < pts_x.size(); i++) {
          //            cout << pts_x[i] << ", ";
          //          }
          //          cout << endl;
          // Spline
          tk::spline s;
          s.set_points(pts_x, pts_y);
          // calulate how to break up spline point so that we travel at our desired reference velocity
          double target_x = 30.;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double x_add_on = 0;
          double step = target_dist / (.02 * ref_velocity / 2.24);
          
          // now we have a spline to fill the actual x, y points to use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // start with all previous points (the points car actually mooved)
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // then fill up the rest of our path planner(here we'll always output 50 points)
          // Note: previous path is how many points car actually mooved
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
          
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          
          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });
  
  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });
  
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
















































































