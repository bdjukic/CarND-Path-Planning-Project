#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {

public:
  enum Lane { Left = 0, Center = 1, Right = 2, Overlapping };

  // Unique id
  int id;

  // Position in map's coordinates
  double x;
  double y;
  double dx;
  double dy;

  // Position in Frenet's coordinates
  double s;
  double d;

  double yaw;
  double speed;
  double suggested_speed = 0;

  Vehicle();
  Vehicle(int id, double x, double y, double dx, double dy, double s, double d);
  Vehicle(double x, double y, double s, double d, double yaw, double speed);

  virtual ~Vehicle();

  Vehicle::Lane get_lane();
  void set_lane(Vehicle::Lane lane);

  void update_localization(double x, double y, double s, double d, double yaw,
                           double speed);
};

#endif
