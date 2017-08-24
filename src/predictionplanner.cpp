#include "predictionplanner.h"

using namespace std;

PredictionPlanner::PredictionPlanner(SensorFusion *sensor_fusion) {
  this->sensor_fusion = sensor_fusion;
}

PredictionPlanner::~PredictionPlanner() {}

vector<double> PredictionPlanner::get_neighbouring_vehicle_distance(
    Vehicle::Lane lane, double current_s) {
  double closest_behind = 0;
  double closest_infront = INT_MAX;

  for (int i = 0; i < this->sensor_fusion->vehicles.size(); i++) {
    if (this->sensor_fusion->vehicles[i].get_lane() == lane) {
      if (sensor_fusion->vehicles[i].s >= current_s &&
          sensor_fusion->vehicles[i].s <= closest_infront) {
        closest_infront = sensor_fusion->vehicles[i].s - current_s;
      } else if (sensor_fusion->vehicles[i].s < current_s &&
                 sensor_fusion->vehicles[i].s > closest_behind) {
        closest_behind = current_s - sensor_fusion->vehicles[i].s;
      }
    }
  }

  return {closest_behind, closest_infront};
}

bool PredictionPlanner::should_slow_down(Vehicle::Lane current_lane,
                                         double current_s,
                                         int previous_path_size) {
  bool should_slow_down = false;

  for (int i = 0; i < this->sensor_fusion->vehicles.size(); i++) {
    if (current_lane == this->sensor_fusion->vehicles[i].get_lane()) {
      double future_s =
          (previous_path_size * 0.02 * this->sensor_fusion->vehicles[i].speed) +
          this->sensor_fusion->vehicles[i].s;

      if (future_s > current_s && (future_s - current_s < 20)) {
        should_slow_down = true;
      }
    }
  }

  return should_slow_down;
}