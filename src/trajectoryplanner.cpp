#include "trajectoryplanner.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

TrajectoryPlanner::TrajectoryPlanner() {}

TrajectoryPlanner::~TrajectoryPlanner() {}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s,
                     vector<double> maps_x, vector<double> maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

vector<vector<double>> TrajectoryPlanner::getTrajectory(
    Vehicle vehicle, vector<double> previous_path_x,
    vector<double> previous_path_y, vector<double> map_waypoints_s,
    vector<double> map_waypoints_x, vector<double> map_waypoints_y) {
  vector<vector<double>> nextPath;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  int previous_path_size = previous_path_x.size();

  // Adding unused previous path points to the next path points
  for (int i = 0; i < previous_path_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Creatign anchor points for future path
  vector<double> anchor_x_vals;
  vector<double> anchor_y_vals;

  double reference_car_x = vehicle.x;
  double reference_car_y = vehicle.y;
  double reference_car_yawn = deg2rad(vehicle.yaw);

  if (previous_path_size < 2) {
    double previous_reference_car_x = vehicle.x - cos(vehicle.yaw);
    double previous_reference_car_y = vehicle.y - sin(vehicle.yaw);

    anchor_x_vals.push_back(previous_reference_car_x);
    anchor_x_vals.push_back(vehicle.x);

    anchor_y_vals.push_back(previous_reference_car_y);
    anchor_y_vals.push_back(vehicle.y);
  } else {
    reference_car_x = previous_path_x[previous_path_size - 1];
    reference_car_y = previous_path_y[previous_path_size - 1];

    double previous_reference_car_x = previous_path_x[previous_path_size - 2];
    double previous_reference_car_y = previous_path_y[previous_path_size - 2];

    reference_car_yawn = atan2(reference_car_y - previous_reference_car_y,
                               reference_car_x - previous_reference_car_x);

    anchor_x_vals.push_back(previous_reference_car_x);
    anchor_x_vals.push_back(reference_car_x);

    anchor_y_vals.push_back(previous_reference_car_y);
    anchor_y_vals.push_back(reference_car_y);
  }

  vector<double> anchor_point_30 =
      getXY(vehicle.s + 30, 2 + 4 * static_cast<int>(vehicle.get_lane()),
            map_waypoints_s, map_waypoints_x, map_waypoints_y);

  vector<double> anchor_point_60 =
      getXY(vehicle.s + 60, 2 + 4 * static_cast<int>(vehicle.get_lane()),
            map_waypoints_s, map_waypoints_x, map_waypoints_y);

  vector<double> anchor_point_90 =
      getXY(vehicle.s + 90, 2 + 4 * static_cast<int>(vehicle.get_lane()),
            map_waypoints_s, map_waypoints_x, map_waypoints_y);

  anchor_x_vals.push_back(anchor_point_30[0]);
  anchor_x_vals.push_back(anchor_point_60[0]);
  anchor_x_vals.push_back(anchor_point_90[0]);

  anchor_y_vals.push_back(anchor_point_30[1]);
  anchor_y_vals.push_back(anchor_point_60[1]);
  anchor_y_vals.push_back(anchor_point_90[1]);

  // Shifting anchor points into car reference points
  for (int i = 0; i < anchor_x_vals.size(); i++) {
    double shift_x = anchor_x_vals[i] - reference_car_x;
    double shift_y = anchor_y_vals[i] - reference_car_y;

    anchor_x_vals[i] = shift_x * cos(0 - reference_car_yawn) -
                       shift_y * sin(0 - reference_car_yawn);

    anchor_y_vals[i] = shift_x * sin(0 - reference_car_yawn) +
                       shift_y * cos(0 - reference_car_yawn);
  }

  // Creating spline through the anchor points
  tk::spline s;
  s.set_points(anchor_x_vals, anchor_y_vals);

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

  double x_add_on = 0;

  for (int i = 1; i <= 50 - previous_path_size; i++) {
    double N = target_dist / (0.02 * vehicle.suggested_speed / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    // We need to go back to global coordinates before we add the points
    // for the next path
    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(reference_car_yawn) - y_ref * sin(reference_car_yawn);

    y_point = x_ref * sin(reference_car_yawn) + y_ref * cos(reference_car_yawn);

    x_point += reference_car_x;
    y_point += reference_car_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  nextPath.push_back(next_x_vals);
  nextPath.push_back(next_y_vals);

  return nextPath;
}
