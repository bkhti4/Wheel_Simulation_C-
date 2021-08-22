#include "CarSim.h"


std::vector<double> CarSim::calculate_slip_angle(double velocity_x, double velocity_y, double steering_angle, double yaw_rate)
{
    std::vector<double> slipVector;

    double slip_front_left = -steering_angle + atan2((velocity_y + (yaw_rate * lf)), (velocity_x - (yaw_rate * track_width / 2)));
    slipVector.push_back(slip_front_left);

    double slip_front_right = -steering_angle + atan2((velocity_y + (yaw_rate * lf)), (velocity_x + (yaw_rate * track_width / 2)));
    slipVector.push_back(slip_front_right);

    double slip_rear_left = atan2((velocity_y - (yaw_rate * lr)), (velocity_x - (yaw_rate * track_width / 2)));
    slipVector.push_back(slip_rear_left);

    double slip_rear_right = atan2((velocity_y - (yaw_rate * lr)), (velocity_x + (yaw_rate * track_width / 2)));
    slipVector.push_back(slip_rear_right);

    return slipVector;
}
