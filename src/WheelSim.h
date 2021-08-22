#ifndef WHEELSIM_H
#define WHEELSIM_H

#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>

class WheelSim
{

public:
    std::vector<double> lateralForces;
    std::vector<double> longitudnalForces;
    double Vmax = 288.14;

    double tireRadius = 0.2;
    double tireWidth = 0.05;

    double tirePressure = 69.0; // kPa at 20 deg

    double F_z0 = 600.0; // Nominal vertical force (N)
    double B_fy = 10.0; // maxmimum stiffness value

    double tempRef = 50.0;
    double tempThread = 30.0;
    double tempCarcass = 30.0;
    double tempGas = 30.0;
    double tempThreadDot = 0.0;
    double tempCarcassDot = 0.0;
    double tempGasDot = 0.0;

    // Parameters
    double tempAmbient = 30.0; // degrees
    double temperature = 30.0; //current temperature
    double tempRoad = 45.0;

    // Magic Formula params
    double B = 1.0; // stiffness factor
    double C = 1.5; // shape factor
    double D = 1100.0; // peak value
    double E = -2.0; // curvature factor
    double Sh = 0.0; // horizontal shift
    double Sv = 0.0; // vertical shift

    // efficiency params
    double Ex = 0.001; // carcass longitudnal force efficiency factor
    double Ey = 0.01; // carcass lateral force efficiency factor
    double Ez = 0.01; // carcass vertical force efficiency factor
    double mu_d; // dynamic friction coefficient

    // thermal model params
    double Q_thread = 0.0;
    double Q_sliding = 0.0; // heat flow due to sliding
    double Q_damp = 0.0; // heat flow due to carcass deflection
    double Q_thread_road = 0.0;
    double Q_carcass_thread = 0.0;
    double Q_carcass_gas = 0.0;

    double Q_thread_amb = 0.0;
    double Q_carcass_amb = 0.0;

    double H_carcass_thread = 1.0;
    double H_carcass_gas = 1.0;
    double H_thread_road = 2.0;

    double S_thread = 1.88; // specific heat of thread
    double S_carcass = 1.88; // specific heat of carcass
    double S_gas = 0.910; //specific heat of gas

    double M_thread = 100.0; // mass of thread
    double M_carcass = 100.0; // mass of carcass
    double M_gas = 100.0; // mass of inflation gas

    // dynamic friction params
    double mu_base = 0.13;

    // peak friction coefficient tuning params
    double a1 = 0.0;
    double a2 = 0.01;
    double a3 = 1.0;

    // dimensionless h tuning params
    double b1 = 0.1;
    double b2 = 0.1;

    // patch area params
    double patch_a = 0.05;
    double patch_b = 0.05;

    // longitudnal forces and temperature adjustment parameters
    double T_x1 = -0.25;
    double T_x2 = 0.15;
    double T_x3 = 0.25;
    double T_x4 = -0.1;


    // lateral forces and temperature adjustment parameters
    double T_y1 = -0.25;
    double T_y2 = 0.15;
    double T_y3 = 0.25;
    double T_y4 = -0.1;

    // car parameters
    double lf = 1.108;
    double lr = 1.402;
    double track_width = 2.1;

    void setParams(double B = 1.0, double C = 1.0, double D = 1.0,
                   double E = 1.0, double Sh = 0.0, double Sv = 0.0);
    double TyreModel(double input);

    void updateTimeDelta(double deltaTime);
    void tyreThermalModel(std::vector<double> forcesVector3D, double longitudeVelocity, double slideVelocity);

    void dynamicFrictionModel(double mu_peak, double dim_less_h, double slideVelocity, double K_shift);

    double calculate_Mu_Peak();
    double calculate_DimLess_H();

    double heatCarcassDeflection(std::vector<double> forcesVector3D, double longitudeVelocity);
    double heatSlidingContact(std::vector<double> forcesVector3D, double slideVelocity);

    double TempXTyreModel(double input);
    double TempYTyreModel(double input, double Fz);

    std::vector<double> calculate_slip_angle(double velocity_x, double velocity_y, double steering_angle, double yaw_rate);
};
#endif // WHEELSIM_H
