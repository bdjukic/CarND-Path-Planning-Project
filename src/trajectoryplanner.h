#include <vector>
#include "vehicle.h"

using namespace std;

class TrajectoryPlanner {
public:
  virtual ~TrajectoryPlanner();
  TrajectoryPlanner();

  vector<vector<double>>
  getTrajectory(Vehicle vehicle, vector<double> previous_path_x,
                vector<double> previous_path_y, vector<double> map_waypoints_s,
                vector<double> map_waypoints_x, vector<double> map_waypoints_y);
};
