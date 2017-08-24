#include "behaviorplanner.h"
#include <algorithm>
#include <iostream>

using namespace std;

BehaviorPlanner::BehaviorPlanner(Vehicle *vehicle,
                                 PredictionPlanner *prediction_planner) {
  this->prediction_planner = prediction_planner;
  this->vehicle = vehicle;
}

BehaviorPlanner::~BehaviorPlanner() {}

double BehaviorPlanner::get_speed_cost() {
  double cost = 0;
  double stop_cost = 0.9;
  double speed_buffer = 0.5;

  double target_speed = SPEED_LIMIT - speed_buffer;

  if (this->vehicle->speed < target_speed) {
    cost = stop_cost * ((target_speed - this->vehicle->speed) / target_speed);
  } else if (this->vehicle->speed >= target_speed &&
             this->vehicle->speed < SPEED_LIMIT) {
    cost = (this->vehicle->speed - target_speed) / speed_buffer;
  } else if (this->vehicle->speed >= SPEED_LIMIT) {
    cost = 1.0;
  }
}

double BehaviorPlanner::get_distance_cost(
    double distance_from_neighbouring_vehicle) {
  return 1 / exp(distance_from_neighbouring_vehicle / 10.0);
}

void BehaviorPlanner::update_behavior(int previous_path_size) {
  Vehicle::Lane current_lane = this->vehicle->get_lane();
  double current_s = this->vehicle->s;

  bool should_slow_down = this->prediction_planner->should_slow_down(
      current_lane, current_s, previous_path_size);

  if (should_slow_down) {
    // Decrease speed
    this->vehicle->suggested_speed -= 0.25;

    double speed_cost = get_speed_cost();

    // Do not update target lane while we're already chaning lanes
    if (!is_changing_lane && speed_cost > 0.1) {
      // Based on cost functions, figure out target lane
      if (this->vehicle->get_lane() == Vehicle::Lane::Center) {
        vector<double> left_neighbouring_vehicles =
            this->prediction_planner->get_neighbouring_vehicle_distance(
                Vehicle::Lane::Left, current_s);

        vector<double> right_neighbouring_vehicles =
            this->prediction_planner->get_neighbouring_vehicle_distance(
                Vehicle::Lane::Right, current_s);

        double left_lane_cost_behind =
            get_distance_cost(left_neighbouring_vehicles[0]);
        double left_lane_cost_infront =
            get_distance_cost(left_neighbouring_vehicles[1]);
        double total_left_lane_cost =
            left_lane_cost_behind + left_lane_cost_infront;

        double right_lane_cost_behind =
            get_distance_cost(right_neighbouring_vehicles[0]);
        double right_lane_cost_infront =
            get_distance_cost(right_neighbouring_vehicles[1]);
        double total_right_lane_cost =
            right_lane_cost_behind + right_lane_cost_infront;

        if (total_left_lane_cost > total_right_lane_cost &&
            right_lane_cost_behind < BEHIND_LANE_MAXIMUM_COST &&
            right_lane_cost_infront < INFRONT_LANE_MAXIMUM_COST) {
          std::cout << "Taking right lane." << std::endl;

          is_changing_lane = true;
          target_lane = Vehicle::Lane::Right;
        } else if (total_left_lane_cost < total_right_lane_cost &&
                   left_lane_cost_behind < BEHIND_LANE_MAXIMUM_COST &&
                   left_lane_cost_infront < INFRONT_LANE_MAXIMUM_COST) {
          std::cout << "Taking left lane." << std::endl;

          is_changing_lane = true;
          target_lane = Vehicle::Lane::Left;
        }
      } else if (this->vehicle->get_lane() == Vehicle::Lane::Left) {
        vector<double> center_neighbouring_vehicles =
            this->prediction_planner->get_neighbouring_vehicle_distance(
                Vehicle::Lane::Center, current_s);

        double center_lane_cost_behind =
            get_distance_cost(center_neighbouring_vehicles[0]);
        double center_lane_cost_infront =
            get_distance_cost(center_neighbouring_vehicles[1]);

        if (center_lane_cost_behind < BEHIND_LANE_MAXIMUM_COST &&
            center_lane_cost_infront < INFRONT_LANE_MAXIMUM_COST) {
          std::cout << "Taking center lane." << std::endl;

          is_changing_lane = true;
          target_lane = Vehicle::Lane::Center;
        }
      } else if (this->vehicle->get_lane() == Vehicle::Lane::Right) {
        vector<double> center_neighbouring_vehicles =
            this->prediction_planner->get_neighbouring_vehicle_distance(
                Vehicle::Lane::Center, current_s);

        double center_lane_cost_behind =
            get_distance_cost(center_neighbouring_vehicles[0]);
        double center_lane_cost_infront =
            get_distance_cost(center_neighbouring_vehicles[1]);

        if (center_lane_cost_behind < BEHIND_LANE_MAXIMUM_COST &&
            center_lane_cost_infront < INFRONT_LANE_MAXIMUM_COST) {
          std::cout << "Taking center lane." << std::endl;

          is_changing_lane = true;
          target_lane = Vehicle::Lane::Center;
        }
      }
    }

    // Only change flag once we're sure we're in the right target lane
    double lane_change_mean_squared_error = sqrt(
        pow((static_cast<int>(target_lane) * 4 + 2) - this->vehicle->d, 2));

    if (lane_change_mean_squared_error < 0.2) {
      is_changing_lane = false;
    }

    // Update vehicle's target lane
    this->vehicle->set_lane(target_lane);
  } else if (vehicle->suggested_speed < SPEED_LIMIT) {
    // Increase speed
    this->vehicle->suggested_speed += 0.7;
  }
}
