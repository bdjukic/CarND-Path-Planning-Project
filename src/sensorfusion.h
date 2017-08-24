
#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include <vector>

#include "vehicle.h"

using namespace std;

class SensorFusion {
 public:
  SensorFusion();
  virtual ~SensorFusion();

  vector<Vehicle> vehicles;

  void update_traffic(vector<vector<double>> sensor_fusion);
};

#endif
