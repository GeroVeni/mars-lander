// Mars lander simulator
// Version 1.9
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2016

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

// #define EULER
#define EXTENSION_5

double target_speed(double Kh, double h)
    // Function to determine target speed (and adjust autopilot throttle respectively)
{
    static char target = 0;
    static double Kc = 0.17e-3;

    switch (target)
    {
        case 1:
        // Parabolic target speed curve
        return -(0.5 + Kc * Kh * h * h);
        break;

        default:
        // Linear target speed curve
        return -(0.5 + Kh * h);
        break;
    }
}

void autopilot (void)
    // Autopilot to adjust the engine throttle, parachute and attitude control
{
    // Working triples (default target speed)
    // Kh       Kp      Delta   Cases
    // 0.018    1.0     0.2     10km
    // 0.0161   0.3     0.2     200km
    // 0.017    1.0     0.333   Both
    // 0.017    0.3     0.333   Both (+ Parachute)

    static double Kh = 0.017, Kp = 0.3, Delta = 0.333;
    double h = position.abs() - MARS_RADIUS;
    double err = target_speed(Kh, h) - velocity * position.norm();
    double p_out = Kp * err;
    double res = p_out + Delta;
    if (res <= 0) throttle = 0.0;
    else if (res < 1) throttle = res;
    else throttle = 1.0;

#ifdef EXTENSION_0
    if (safe_to_deploy_parachute()
        && h <= EXOSPHERE 
        && 0.5 + Kh * h < MAX_PARACHUTE_SPEED
        && true)
    {
        parachute_status = DEPLOYED;
    }
#endif

#ifdef EXTENSION_1
	throttle = 747.1 / MAX_THRUST;
#endif
	
#ifdef EXTENSION_2
	double kp = 0.01;
	double target_altitude = 500.0;
	throttle = 747.1 / MAX_THRUST + kp * (target_altitude - h);
	if (throttle < 0.0) throttle = 0.0;
	if (throttle > 1.0) throttle = 1.0;
#endif

#ifdef EXTENSION_3
	double kp = 0.01, kd = 0.01;
	double target_altitude = 500.0;
	throttle = 747.1 / MAX_THRUST + kp * (target_altitude - h) - kd * velocity * position.norm();
#endif

#ifdef EXTENSION_4
	double kp = 0.091, kd = 0.178;
	double target_altitude = 500.0;
	throttle = 747.1 / MAX_THRUST + kp * (target_altitude - h) - kd * velocity * position.norm();
#endif
	
#ifdef EXTENSION_5
	double kp = 0.091, kd = 0.178;
	double target_altitude = 500.0;
	throttle = 747.1 / MAX_THRUST + kp * (target_altitude - h) - kd * velocity * position.norm();
#endif

    
	attitude_stabilization();

    // Logging proportional control details
    if (prop_ctrl_log)
    {
        prop_ctrl_log << simulation_time << ' ' << h << ' ' << -(0.5 + Kh * h) << ' ' << velocity * position.norm() << endl;
    }
}

void numerical_dynamics (void)
    // This is the function that performs the numerical integration to update the
    // lander's pose. The time step is delta_t (global variable).
{
    double mass = UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
    double v2 = velocity.abs2();
    double p2 = position.abs2();
    double rho = atmospheric_density(position);
    double area_lander = PI * LANDER_SIZE * LANDER_SIZE;
    double area_chute = 5.0 * (2.0 * LANDER_SIZE) * (2.0 * LANDER_SIZE);
    vector3d a_gravity = (-1.0 * GRAVITY * MARS_MASS / p2) * position.norm();
    vector3d a_thrust = thrust_wrt_world() / mass;
#ifdef EXTENSION_5
	a_thrust += 100.0 * cos(0.1 * simulation_time) * position.norm() / mass;
#endif
    vector3d a_drag = (-0.5 * rho * DRAG_COEF_LANDER * area_lander * v2 / mass) * velocity.norm();
    if (parachute_status == DEPLOYED) a_drag += (-0.5 * rho * DRAG_COEF_CHUTE * area_chute * v2 / mass) * velocity.norm();
    vector3d a = a_gravity + a_thrust + a_drag;

    #ifdef EULER
    // Perform Euler integration

    position += delta_t * velocity;
    velocity += delta_t * a;

    #else // EULER
    // Perform Verlet integration

    static vector3d prev_position;
    
    if (simulation_time == 0.0)
        // Perform euler step for the first iteration
    {
        prev_position = position;
        position += delta_t * velocity;
        velocity += delta_t * a;
    }
    else
    {
        vector3d new_position = 2 * position - prev_position + delta_t * delta_t * a;
        velocity = (new_position - prev_position) / (2 * delta_t);
        prev_position = position;
        position = new_position;
    }

    #endif // EULER

    // Here we can apply an autopilot to adjust the thrust, parachute and attitude
    if (autopilot_enabled) autopilot();

    // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();
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
    scenario_description[7] = "";
    scenario_description[8] = "";
    scenario_description[9] = "";

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
        // areostationary orbit
        double omega, r3, as_rad;
        omega = 2.0 * PI / MARS_DAY;
        r3 = GRAVITY * MARS_MASS / (omega * omega);
        as_rad = cbrt(r3);
        position = vector3d(as_rad, 0.0, 0.0);
        velocity = vector3d(0.0, omega * as_rad, 0.0);
        orientation = vector3d(0.0, 90.0, 0.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        break;

        case 7:
 		// a descent from rest at the edge of the exosphere
        position = vector3d(0.0, -(MARS_RADIUS + 700), 0.0);
        velocity = vector3d(0.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = true;
        autopilot_enabled = false;
       	break;

        case 8:
        break;

        case 9:
        break;

    }
}
