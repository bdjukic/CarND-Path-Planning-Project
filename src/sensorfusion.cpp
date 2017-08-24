#include <fstream>
#include <iostream>

#include "sensorfusion.h"

using namespace std;

SensorFusion::SensorFusion() {}

SensorFusion::~SensorFusion() {}

void SensorFusion::update_traffic(vector<vector<double>> sensor_fusion) {
  // Clearing up the previos traffic state
  this->vehicles.clear();

  for (auto &senor_data : sensor_fusion) {
    int id = senor_data[0];
    double x = senor_data[1];
    double y = senor_data[2];
    double dx = senor_data[3];
    double dy = senor_data[4];
    double s = senor_data[5];
    double d = senor_data[6];

    Vehicle vehicle = Vehicle(id, x, y, dx, dy, s, d);

    this->vehicles.push_back(vehicle);
  }
}
