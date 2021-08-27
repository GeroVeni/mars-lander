// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

double start_altitude = 10000;
double mass();

void altitude_control() {
  double target_altitude = 500;

  double r = MARS_RADIUS + target_altitude;
  double m = mass();
  double f = GRAVITY * MARS_MASS * m / (r * r);

  double climb_speed_ = velocity * position.norm();
  double h = position.abs() - MARS_RADIUS;

  double e = target_altitude - h;
  double de = -climb_speed_;

  double zeta = 4, omega = 0.125;
  double kp = m * omega * omega / MAX_THRUST, kd = 2 * zeta * omega * m / MAX_THRUST;

  stabilized_attitude = true;
  stabilized_attitude_angle = 0;

  throttle = f / MAX_THRUST + kp * e + kd * de;
}

double target_speed(vector3d pos) {
  double Kh = 0.017;
  double h = pos.abs() - MARS_RADIUS;
  return -(0.5 + Kh * h);
}

void orbital_injection(void) {
  static char stage = 0;
  double h = position.abs() - MARS_RADIUS;
  stabilized_attitude = true;
  if (h < 1.5 * EXOSPHERE) {
    stage = 1;
  } else {
    double climb_speed_ = velocity * position.norm();
    double ground_speed_ = (velocity - climb_speed_ * position.norm()).abs();
    if (ground_speed_ < 3500 && stage != 3) stage = 2;
    else stage = 3;
  }
  switch (stage) {
    case 1:
      stabilized_attitude_angle = 0;
      throttle = 1.0;
      break;
    case 2:
      stabilized_attitude_angle = 90;
      throttle = 1.0;
      break;
    case 3:
      throttle = 0.0;
      break;
  }
}

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  if (scenario == 7) {
    orbital_injection();
    return;
  }
  if (scenario == 8) {
    altitude_control();
    return;
  }

  stabilized_attitude = true;
  stabilized_attitude_angle = 0;

  // Throttle
  double Kp = 0.3, delta = 0.333;
  double h = position.abs() - MARS_RADIUS;
  double err = target_speed(position) - velocity * position.norm();
  double p_out = Kp * err;
  throttle = delta + p_out;
  if (throttle < 0) throttle = 0;
  if (throttle > 1) throttle = 1;

  // Parachute
  if (safe_to_deploy_parachute() &&
      target_speed(position) > -MAX_PARACHUTE_SPEED &&
      h <= EXOSPHERE) {
    parachute_status = DEPLOYED;
  }
}

vector3d gravity(double M, vector3d r) {
  return (-GRAVITY * M / r.abs2()) * r.norm();
}

vector3d drag(double density, double C_drag, double projected_area, vector3d v) {
  return (-0.5 * density * C_drag * projected_area * v.abs2()) * v.norm();
}

double mass() {
  return UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
}

double lander_area() {
  return M_PI * LANDER_SIZE * LANDER_SIZE;
}

double chute_area() {
  return 5.0 * (2.0 *LANDER_SIZE) * (2.0 * LANDER_SIZE);
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  static vector3d prev_position = vector3d();

  double m = mass();

  // Gravity
  vector3d gravity_force = gravity(MARS_MASS, position) * m;

  // Drag
  vector3d drag_force = drag(atmospheric_density(position),
      DRAG_COEF_LANDER, lander_area(), velocity);

  if (parachute_status == DEPLOYED) {
    drag_force += drag(atmospheric_density(position),
        DRAG_COEF_CHUTE, chute_area(), velocity);
  }

  // Thrust
  vector3d thrust_force = thrust_wrt_world() + 100 * cos(0.1 * simulation_time) * position.norm();
  
  // Total forces
  vector3d total = gravity_force + drag_force + thrust_force;
  
  if (simulation_time == 0) {
    // First step - initialise prev_position
    prev_position = position - velocity * delta_t;
  }
  
  // Verlet update
  vector3d new_position = 2 * position - prev_position + (delta_t * delta_t / m) * total;
  velocity = (new_position - prev_position) / (2.0 * delta_t);
  prev_position = position;
  position = new_position;

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

double circ_velocity(double M, double r) {
  return std::sqrt(GRAVITY * M / r);
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "areostationary orbit";
  scenario_description[7] = "orbital injection";
  scenario_description[8] = "altitude control";
  scenario_description[9] = "";

  double R;

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    // an aerostationary equatorial orbit
    R = std::cbrt(GRAVITY * MARS_MASS *  MARS_DAY * MARS_DAY / (4 * M_PI * M_PI));
    position = vector3d(R, 0.0, 0.0);
    velocity = vector3d(0.0, circ_velocity(MARS_MASS, R), 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 7:
    // Orbital injection
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    throttle = 1.0;
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 8:
    // Altitude control
    position = vector3d(0.0, 0.0, MARS_RADIUS + start_altitude);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.01;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = true;
    break;

  case 9:
    break;

  }
}
