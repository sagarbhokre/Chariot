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

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

#define MPS2MPH              (2.23694)
#define SPEW_LVL             (1)
#define LANE_CHANGE_DIST_THRESHOLD (40)
#define COLLISION_THRESHOLD  (30)
#define BACK_COLLISION_THRESHOLD (1)
#define LOOK_AHEAD_THRESHOLD (50)
#define CAR_DIM              (5.0)
#define SLOWDOWN_RANGE       (0)
#define COOLDOWN_RANGE       (10)
#define MAX_ACCELERATION     (7*MPS2MPH*MPS2MPH)
#define NUM_POINTS           (50)

double MIN_SPEED = 20.0;
double MAX_SPEED = 49.5;
double ref_speed = 1; //MIN_SPEED;
int LANE_ID = -1;
int desired_lane = -1;
bool initialized =  false;

typedef struct sensor_data_t_ {
    uint32_t id;
    double gx, gy, v, s, d;
} sensor_data_t;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = abs(theta-heading);

    if(angle > pi()/4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
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

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
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

void get_next_wp_coords(double car_x, double car_y, double car_yaw, vector<double> map_waypoints_x,
            vector<double> map_waypoints_y, double &wp_x, double &wp_y)
{
    int next_wp = NextWaypoint(car_x,car_y, car_yaw, map_waypoints_x, map_waypoints_y);

    if(next_wp >= map_waypoints_y.size()) {
        next_wp = 0;
    }

    wp_x = map_waypoints_x[next_wp];
    wp_y = map_waypoints_y[next_wp];
}

double compute_dest_angle(double car_x, double car_y, double wp_x, double wp_y, int lane_id)
{
    double dst_x = wp_x;
    double dst_y = wp_y - ((lane_id*4)+2);
    return atan2( (dst_y-car_y),(dst_x-car_x) );
}

void append_pts(vector<double> &sx, vector<double> &sy, double x, double y)
{
    sx.push_back(x);
    sy.push_back(y);
}

void print_vec(vector<double> &sx, vector<double> &sy)
{
    cout << "<------" << endl;
    for(int i=0; i<sx.size(); i++) {
        cout << "x: " << sx[i] << " y: " << sy[i] << endl;
    }
    cout << "------>" << endl;
}

void convert2local(double gx, double gy, double rx, double ry, double ryaw, double &lx, double &ly)
{
    double shift_x = gx - rx;
    double shift_y = gy - ry;
    lx = shift_x*cos(0-ryaw) - shift_y*sin(0-ryaw);
    ly = shift_x*sin(0-ryaw) + shift_y*cos(0-ryaw);
}
void convert2local_vec(vector<double> &sx, vector<double> &sy, double x, double y, double yaw)
{
    for(int i = 0; i < sx.size(); i++) {
        double shift_x = sx[i] - x;
        double shift_y = sy[i] - y;
        sx[i] = shift_x*cos(0-yaw) - shift_y*sin(0-yaw);
        sy[i] = shift_x*sin(0-yaw) + shift_y*cos(0-yaw);
    }
}

void update_ref_vel(double val,
                    double min_speed = MIN_SPEED,
                    double max_speed = MAX_SPEED)
{
    double sign = signbit(val) ? -1.0 : 1.0;
    ref_speed += MAX_ACCELERATION*0.02*sign;

    if(SPEW_LVL > 4) cout << "Add term : " << MAX_ACCELERATION*0.02*sign << endl;

    if(ref_speed > max_speed) ref_speed = max_speed;
    if((ref_speed < min_speed) && initialized) ref_speed = min_speed;
    if(ref_speed >= max_speed) initialized = true;
}

bool check_lane_change(vector<sensor_data_t> sensor_data, vector<double> n_x, vector<double> n_y,
                       vector<double> map_waypoints_x, vector<double> map_waypoints_y,
                       vector<double> map_waypoints_s,
                       double car_s, double car_d, double car_speed)
{
    bool change_lane = false;
    vector<sensor_data_t> l_sensor_data[3];

    // Segregate sensor data into lane specific data
    for (int i = 0; i < sensor_data.size(); i++) {
        double est_lane = sensor_data[i].d;
        if((est_lane >= 0) && (est_lane < 5)) {
            l_sensor_data[0].push_back(sensor_data[i]);
        }
        if((est_lane >= 3) && (est_lane < 9)) {
            l_sensor_data[1].push_back(sensor_data[i]);
        }
        if((est_lane >= 7) && (est_lane < 13)) {
            l_sensor_data[2].push_back(sensor_data[i]);
        }
    }

    bool safe[3];
    double coll_dist[3];
    double coll_ndist[3];
    double coll_vel[3];
    double coll_nvel[3];
    static int cooldown = 200, slowdown = 0;

    // Find vel, distance of objects in all lanes
    for(int lane = 0; lane < 3; lane++) {
        safe[lane] = true;
        coll_dist[lane] = COLLISION_THRESHOLD*10; //Large number
        coll_ndist[lane] = -COLLISION_THRESHOLD*10; //Large number
        coll_vel[lane] = MIN_SPEED;
        for(int idx = 0; idx < l_sensor_data[lane].size(); idx++) {
            double diff = l_sensor_data[lane][idx].s - car_s;
            double vel = l_sensor_data[lane][idx].v;
            double rel_vel = vel - car_speed;
            if (rel_vel < 0.0) rel_vel = 0.0;

            if ((SPEW_LVL > 3) && (diff < 0) && (abs(LANE_ID -lane) == 1)) {
                cout << "["<<lane<<"] diff: " << diff << " rel_vel: " << rel_vel
                     << " " << CAR_DIM+BACK_COLLISION_THRESHOLD*rel_vel
                     << endl;
            }

            // Distance behind and ahead of car is acceptable?
            if (((diff > 0) && (diff < LANE_CHANGE_DIST_THRESHOLD)) ||
                ((diff < 0) && (-diff < (CAR_DIM + BACK_COLLISION_THRESHOLD*rel_vel)))) {
                safe[lane] = false;
            }

            // Least of all objects in the same lane
            if ((diff > 0) && (diff < coll_dist[lane])) {
                coll_dist[lane] = diff;
                // Consider measurements until 150m to avoid false/frequent lane changes
                if(coll_dist[lane] > 150) coll_dist[lane] = COLLISION_THRESHOLD*10;
                coll_vel[lane] = vel > MAX_SPEED ?  MAX_SPEED : vel;
            }
            else if((diff < 0) && (diff > coll_ndist[lane])) {
                coll_ndist[lane] = diff;
                coll_nvel[lane] = vel;
            }
        }

        // Check if we need to change lane due to object in front
        if ((LANE_ID == lane) && (coll_dist[lane] < COLLISION_THRESHOLD)) {
            change_lane = true;
        }
    }

    // Info print
    if(SPEW_LVL > 0) {
        printf("\n%2.2f [%d]", ref_speed, LANE_ID);
        for(int lane = 0; lane < 3; lane++) {
            printf("| %7.2f,%5.2f[%d][%s]  |", coll_dist[lane], coll_vel[lane],
                                               (int)l_sensor_data[lane].size(),
                                               safe[lane] ? "^" : "_");
        }

        printf("\n%2.2f [%d]", ref_speed, LANE_ID);
        for(int lane = 0; lane < 3; lane++) {
            printf("| %7.2f,%5.2f[%d][%s]  |", coll_ndist[lane], coll_nvel[lane],
                                               (int)l_sensor_data[lane].size(),
                                               safe[lane] ? "^" : "_");
        }
        printf("\n");
    }

    // cooldown ensures we allow the car to settle before switching lane again
    if((cooldown-- <= 0)) {
        int best_lane = LANE_ID;
        double best_dist = coll_dist[LANE_ID];
        for (int i = 0; i < 3; i++) {
            if ((coll_dist[i] > best_dist) && (abs(LANE_ID - i) <= 1) &&
                (safe[i] == true)) {

                best_lane = i;
                best_dist = coll_dist[i];
            }
        }

        //  Avoid changing lane if not needed
        if(abs(best_dist - coll_dist[LANE_ID]) < 2) {
            best_lane = LANE_ID;
        }

        // Set desired lane and wait for slowdown before switching lane.
        // Switching lanes at faster speed lead to acceleration overshoot
        if(LANE_ID != best_lane) {
            cout << "------------Switch from lane " << LANE_ID;
            desired_lane = best_lane;
            cooldown = COOLDOWN_RANGE;
            slowdown = SLOWDOWN_RANGE;
            cout << " to lane " << best_lane << endl;
        }
    }

    // update lane id after slowdown
    if(slowdown <= 0) {
        slowdown = 0;
        LANE_ID = desired_lane;
    }

    // Slowdown
    double dist = coll_dist[LANE_ID] > COLLISION_THRESHOLD ? COLLISION_THRESHOLD : coll_dist[LANE_ID];
    if (slowdown-- > 0) {
        printf("\n SD \n");
        update_ref_vel(-1.0, MIN_SPEED, MAX_SPEED);
        return true; //Flush prev val
    }
    else if ((change_lane == false)) {
        // Accelerate if we plan to stay in the same lane
        update_ref_vel(1.0, MIN_SPEED, MAX_SPEED);
    }
    else if((change_lane == true)) {
        double follow_vel = coll_vel[LANE_ID]*(dist/COLLISION_THRESHOLD);
        update_ref_vel(-1.0, MIN_SPEED, MAX_SPEED);
        return true; //Flush prev val
    }
    return false;
}

void analyze_path(vector<double> xs, vector<double> ys)
{
    vector<double> vs, as;

    for(int i=0; i<xs.size()-1; i++) {
        vs.push_back(distance(xs[i+1], ys[i+1], xs[i], ys[i])/0.02);
    }

    for(int i=0; i<vs.size()-1; i++) {
        as.push_back(abs((vs[i+1]- vs[i])/0.02));
    }

    cout << "XYs: ";
    for(int i=0; i<xs.size(); i++) {
        cout << "(" << xs[i] << ", " << ys[i] << ") ";
    }
    cout << endl;

    cout << "Vs: ";
    for(int i=0; i<vs.size(); i++) {
        cout << vs[i]*MPS2MPH << " ";
    }
    cout << endl;

    cout << "As: ";
    for(int i=0; i<as.size(); i++) {
        cout << as[i] << " ";
    }
    cout << endl;
    double max_acc = *std::max_element(as.begin(), as.end());
    cout << " Max : " << *std::max_element(as.begin(), as.end()) << endl;
    if(max_acc > MAX_ACCELERATION) exit(0);
}

void process_data(double &car_x, double &car_y,
          double &car_s, double &car_d,
          double &car_yaw, double &car_speed,
          vector<double> &previous_path_x, vector<double> &previous_path_y,
          double &end_path_s, double &end_path_d,
          vector<double> &map_waypoints_x, vector<double> &map_waypoints_y,
          vector<double> &map_waypoints_s,
          vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy,
          vector<double> &next_x_vals, vector<double> &next_y_vals,
          vector<sensor_data_t> sensor_data)
{
    double ref_x = car_x, ref_y = car_y, ref_yaw = deg2rad(car_yaw);
    double pos_x, pos_x2;
    double pos_y, pos_y2;
    double angle;
    vector<double> sx, sy;
    int path_size = previous_path_x.size();
    static bool flush_readings = false;

    if(flush_readings ==  true) {
        path_size = 2;
        if(SPEW_LVL > 3) cout << "-------------------Flush readings----------------" << endl;
        //ref_speed = car_speed;
        flush_readings = false;
    }

    // Initialize next vals with previously available points
    for(int i = 0; i < path_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Path size is 0/1, use car position and estimate previous position using car angle
    if(path_size < 2)
    {
        if(SPEW_LVL > 3) cout << "Empty path\n";
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);

        pos_x2 = car_x - cos(car_yaw);
        pos_y2 = car_y - sin(car_yaw);
    }
    // Previous path available. Estimate car angle using last 2 data points
    else
    {
        if(SPEW_LVL > 3) print_vec(previous_path_x, previous_path_y);
        pos_x = previous_path_x[path_size-1];
        pos_y = previous_path_y[path_size-1];

        pos_x2 = previous_path_x[path_size-2];
        pos_y2 = previous_path_y[path_size-2];
        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    }

    append_pts(sx, sy, pos_x2, pos_y2);
    append_pts(sx, sy, pos_x, pos_y);

    vector<double> xy;
    for(int i =0; i < 3; i++) {
        xy = getXY(car_s+LOOK_AHEAD_THRESHOLD*(i+1), 2+4*LANE_ID,
                   map_waypoints_s, map_waypoints_x, map_waypoints_y);
        append_pts(sx, sy, xy[0], xy[1]);
    }

    convert2local_vec(sx, sy, ref_x, ref_y, ref_yaw);
    if(SPEW_LVL > 1)
        cout << "x,y,yaw: " << pos_x << " " << pos_y << " " << rad2deg(angle) << endl;

    tk::spline f_s;
    f_s.set_points(sx, sy);

    double dist_inc = (ref_speed / MPS2MPH)/NUM_POINTS;
    double lx, ly, gx, gy;
    double tx1, tx2, ty2, ty1;
    convert2local(pos_x, pos_y, ref_x, ref_y, ref_yaw, lx, ly);
    convert2local(pos_x, pos_y, ref_x, ref_y, ref_yaw, tx1, ty1);
    convert2local(pos_x2, pos_y2, ref_x, ref_y, ref_yaw, tx2, ty2);
    pos_x = tx1; pos_y = ty1;
    angle = atan2(ty1-ty2,tx1-tx2);

    double prev_x = lx, prev_y = ly;
    for(int i = 0; i < NUM_POINTS-path_size; i++)
    {
        lx += dist_inc*abs(cos(angle)*cos(angle));
        ly = f_s(lx);

        gx = (lx)*cos(ref_yaw) - (ly)*sin(ref_yaw);
        gy = (lx)*sin(ref_yaw) + (ly)*cos(ref_yaw);

        gx += ref_x;
        gy += ref_y;
        if((SPEW_LVL > 3)) {
            cout << "dist_inc: " << dist_inc*NUM_POINTS*MPS2MPH
                 << " lx: " << lx << " ly: " << ly
                 << " gx: " << gx << " gy: " << gy << endl;
        }

        next_x_vals.push_back(gx);
        next_y_vals.push_back(gy);
        angle = atan2(ly-pos_y, lx-pos_x);
        pos_x = lx; pos_y = ly;
        prev_x = lx; prev_y = ly;
    }

    if(SPEW_LVL > 3) analyze_path(next_x_vals, next_y_vals);

    flush_readings = check_lane_change(sensor_data, next_x_vals, next_y_vals,
                                       map_waypoints_x, map_waypoints_y,
                                       map_waypoints_s, car_s, car_d, car_speed);

    return;
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

    static int count = 0;
    if(SPEW_LVL>3) {
        cout << count++ << " WP map: " << x << " " << y << " "
                                       << s << " " << d_x << " " << d_y << endl;
    }

    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

            json msgJson;

            vector<double> next_x_vals;
            vector<double> next_y_vals;
            double ref_x = car_x, ref_y = car_y, ref_yaw = car_yaw;

            // Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            // Initialize lane id
            if(LANE_ID == -1) {
                LANE_ID = (int)car_d/4;
                desired_lane = LANE_ID;
            }
            vector<double> ppath_x, ppath_y;
            for(int i=0; i<previous_path_x.size(); i++) {
                append_pts(ppath_x, ppath_y, previous_path_x[i], previous_path_y[i]);
            }

            vector<sensor_data_t> sensor_data_fused;
            for(int i = 0; i < sensor_fusion.size(); i++) {
                sensor_data_t data;
                data.id = sensor_fusion[i][0];
                data.gx = sensor_fusion[i][1];
                data.gy = sensor_fusion[i][2];
                double vx = sensor_fusion[i][3], vy = sensor_fusion[i][4];
                data.v = sqrt(vx*vx + vy*vy)*MPS2MPH;
                data.s  = sensor_fusion[i][5];
                data.d  = sensor_fusion[i][6];
                sensor_data_fused.push_back(data);
            }

            process_data(car_x, car_y, car_s, car_d, car_yaw, car_speed,
                 ppath_x, ppath_y, end_path_s, end_path_d,
                 map_waypoints_x, map_waypoints_y,
                 map_waypoints_s, map_waypoints_dx, map_waypoints_dy,
                 next_x_vals, next_y_vals, sensor_data_fused);

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
