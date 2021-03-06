#include "WheelSim.h"


void WheelSim::setParams(double B, double C, double D, double E, double Sh, double Sv)
{
    this->B = B;
    this->C = C;
    this->D = D;
    this->E = E;
    this->Sh = Sh;
    this->Sv = Sv;
}


double WheelSim::TyreModel(double input)
{
    double x = input + Sh;
    double y = D * sin(C * atan((B * x) - (E * (B * x - atan(B * x)))));
    return (y + Sv);
}


double WheelSim::TempXTyreModel(double input)
{
    double deltaTemp = (temperature - tempAmbient) / tempAmbient;
    double Dx = (1 + (T_x3 * deltaTemp) + (T_x4 * pow(deltaTemp, 2.0))) * D;
    double Bx = (1 + (T_x1 * deltaTemp) + (T_x2 * pow(deltaTemp, 2.0))) * B;

    double x = input + Sh;
    double y = Dx * sin(C * atan((Bx * x) - (E * (Bx * x - atan(Bx * x)))));
    return (y + Sv);
}


double WheelSim::TempYTyreModel(double input, double Fz)
{
    double P_BY1 = -85; //B_fy / F_z0;
    double P_BY2 = 5; //B_fy * F_z0;

    double deltaTemp = (temperature - tempAmbient) / tempAmbient;
    double Dy = (1 + (T_y3 * deltaTemp) + (T_y4 * pow(deltaTemp, 2.0))) * D;
    double By = (1 + (T_y1 * deltaTemp)) * P_BY1 * F_z0 * sin(atan2(Fz, (P_BY2 * F_z0 * (1 + T_y2 * deltaTemp))));

    double x = input + Sh;
    double y = Dy * sin(C * atan((By * x) - (E * (By * x - atan(By * x)))));
    return (y + Sv);
}


void WheelSim::updateTimeDelta(double deltaTime)
{
    tempThread += tempThreadDot * deltaTime;
    tempCarcass += tempCarcassDot * deltaTime;
    tempGas += tempGasDot * deltaTime;
//    std::cout << "Temperature thread: " << tempThread << std::endl;

    temperature = tempThread;
}

void WheelSim::tyreThermalModel(std::vector<double> forcesVector3D, double longitudeVelocity, double slideVelocity)
{
    Q_damp = heatCarcassDeflection(forcesVector3D, longitudeVelocity);
    Q_sliding = heatSlidingContact(forcesVector3D, slideVelocity);

    double H_carcass_amb = 1.0; // some constant
    double H_thread_amb = 2.0 * longitudeVelocity + 10.0; // heat flow coefficient

    Q_thread_amb = H_thread_amb * (tempThread - tempAmbient);
    Q_carcass_amb = H_carcass_amb * (tempCarcass - tempGas);

    Q_carcass_thread = H_carcass_thread * (tempCarcass - tempThread);
    Q_carcass_gas = H_carcass_gas * (tempThread - tempGas);

    double p_bar = tirePressure * 1000.0 * ((tempGas + 273.0) / (tempAmbient + 273.0));
    double Acp = 0.12 * pow(p_bar,-0.7) * pow((forcesVector3D[2] / 3000),0.7) * patch_b; // contact patch area

    Q_thread_road = H_thread_road * Acp * (tempThread - tempRoad);

    tempThreadDot = ((Q_sliding - Q_thread_road) + (Q_carcass_thread - Q_thread_amb)) / (S_thread * M_thread);
    tempCarcassDot = ((Q_damp - Q_carcass_thread) - (Q_carcass_amb - Q_carcass_gas)) / (S_carcass * M_carcass);
    tempGasDot = (Q_carcass_gas) / (S_gas * M_gas);
}

double WheelSim::heatCarcassDeflection(std::vector<double> forcesVector3D, double longitudeVelocity)
{
    return ((Ex * abs(forcesVector3D[0])) + (Ey * abs(forcesVector3D[1])) + (Ez * abs(forcesVector3D[2]))) * longitudeVelocity;
}


double WheelSim::heatSlidingContact(std::vector<double> forcesVector3D, double slideVelocity)
{
    double mu_peak = calculate_Mu_Peak();
    double dim_less_h = calculate_DimLess_H();
    dynamicFrictionModel(mu_peak, dim_less_h, slideVelocity, 0.1);

    // mu_d = 1.3;
    return mu_d * forcesVector3D[2] * abs(slideVelocity);
}

void WheelSim::dynamicFrictionModel(double mu_peak, double dim_less_h, double slideVelocity, double K_shift)
{
    mu_d = mu_base + (mu_peak - mu_base) * exp(-pow(((dim_less_h * log((abs(slideVelocity) + 0.0000001) / Vmax)) - (K_shift * (tempThread - tempAmbient))), 2.0));
}

double WheelSim::calculate_Mu_Peak()
{
    return ((a1 * pow(tempThread,2)) + (a2 * tempThread) + a3);
}

double WheelSim::calculate_DimLess_H()
{
    return b1 * (exp(b2 * tempThread) / exp(b2 * tempAmbient));
}

