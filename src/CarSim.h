#ifndef CARSIM_H
#define CARSIM_H

#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>


class CarSim
{
public:
    double initialVelocity = 16.66; // mps
    double velocity = 0.0; // mps
    double acceleration = 0.0; //mps^2
    double distance = 0.0; // m
    double mass = 1200; // kg
    double fR = 0.007;
    double lambda = 1.0;
    double yawMoment = 617.0; //kg . m^2
    double Mz = 0.0;

    double numberOfWheels = 4.0;
    double track_width = 2.0; // m

    double brakePower = 0.0; // kW
    double prevBrakePower = 0.0; // kW
    double throtlePower = 0.0; // kW
    double steeringAngle = 0.0; // deg
    double lastAngle = 0.0;

    double longitudnalVelocity = 0.0;
    double lateralVelocity = 0.0;

    double yawRate = 0.0;
    double dotYawRate = 0.0;

    double maxBrakePower = 1500.0; // kW
    double maxThrotlePower = 1000.0; // kW
    double maxSteeringAngle = 60.0; // deg

    double carFx = 0.0;
    double carFy = 0.0;

    // car parameters
    double lf = 1.108;
    double lr = 1.402;

    // params
    double Accmax = 80.0; // m/s²
    double slip = 0.0;
    double sideSlip = 0.0;

    std::vector<double> calculate_slip_angle(double velocity_x, double velocity_y, double steering_angle, double yaw_rate);
};


#endif // CARSIM_H
