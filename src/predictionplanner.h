#ifndef PREDICTIONPLANNER_H
#define PREDICTIONPLANNER_H

#include <climits>
#include <vector>
#include "sensorfusion.h"
#include "vehicle.h"

using namespace std;

class PredictionPlanner {
 private:
  SensorFusion *sensor_fusion;

 public:
  PredictionPlanner(SensorFusion *sensor_fusion);
  virtual ~PredictionPlanner();

  vector<double> get_neighbouring_vehicle_distance(Vehicle::Lane lane,
                                                   double current_s);
  bool should_slow_down(Vehicle::Lane current_lane, double current_s, int previous_path_size);
};

#endif