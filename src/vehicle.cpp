#include "vehicle.h"
#include <math.h>

Vehicle::Vehicle() {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw,
                 double speed) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed; 
}

Vehicle::Vehicle(int id, double x, double y, double dx, double dy, double s,
                 double d) {
  this->id = id;
  this->x = x;
  this->y = y;
  this->dx = dx;
  this->dy = dy;
  this->s = s;
  this->d = d;

  // Calcuate speed and yaw based on dx and dy
  this->yaw = atan2(this->dy, this->dx);
  this->speed = sqrt(pow(this->dx, 2) + pow(this->dy, 2));
}

Vehicle::~Vehicle() {}

Vehicle::Lane Vehicle::get_lane() {
  if (this->d >= 0 && this->d < 4) {
    return Lane::Left;
  } else if (this->d >= 4 && this->d < 8) {
    return Lane::Center;
  } else if (this->d >= 8 && this->d < 12) {
    return Lane::Right;
  }
}

void Vehicle::set_lane(Vehicle::Lane lane) {
  switch (lane) {
  case Lane::Left: {
    this->d = 2;
    break;
  }
  case Lane::Center: {
    this->d = 6;
    break;
  }
  case Lane::Right: {
    this->d = 10;
    break;
  }
  }
}

void Vehicle::update_localization(double x, double y, double s, double d,
                                  double yaw, double speed) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
  this->speed = speed;
}
