#include "sensorfusion.h"

class BehaviorPlanner {
 private:
  double SPEED_LIMIT = 48.0;
  double BEHIND_LANE_MAXIMUM_COST = 0.001;
  double INFRONT_LANE_MAXIMUM_COST = 0.01;

  bool is_changing_lane = false;
  Vehicle::Lane target_lane;

  Vehicle *vehicle;
  SensorFusion *sensor_fusion;

  double get_speed_cost();
  double get_distance_cost(double distance_from_neighbouring_vehicle);

  vector<double> get_neighbouring_vehicle_distance(Vehicle::Lane lane);

 public:
  BehaviorPlanner(Vehicle *vehicle, SensorFusion *sensor_fusion);
  virtual ~BehaviorPlanner();

  void update_behavior(int previous_path_size);
};
