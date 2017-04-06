#include <iostream>
#include "tools.h"
#include <cmath>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
    int estimationSize = estimations.size();
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // verifiy inputs
    if (estimationSize == 0 || estimationSize != ground_truth.size())
    {
        cout << "Cannot calculate RMSE, invalid input vectors." << endl;
        return rmse;
    }
    
    // accululate the squares of residuals
    for (int i=0; i<estimationSize; i++)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    
    rmse /= estimationSize;
    rmse = rmse.array().sqrt();
    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
    Eigen::MatrixXd Hj(3, 4);
    
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);
    
    double minValue = 1e-3;
    
    if (fabs(px) < minValue && fabs(py) < minValue)
    {
        px = minValue;
        py = minValue;
    }
    
    // compute the denominators
    double mag2 = px*px + py*py;
    double mag = sqrt(mag2);
    double mag32 = mag*mag2;
    
    if(fabs(mag2) < 0.0001)
    {
        cout << "CalculateJacobian() - Denominator is too small." << endl;
        return Hj;
    }
    
    Hj <<   px/mag,                 py/mag,                 0,      0,
            -py/mag2,               px/mag2,                0,      0,
            py*(vx*py-vy*px)/mag32, px*(vy*px-vx*py)/mag32, px/mag, py/mag;
    
    return Hj;
}

VectorXd Tools::CartesianToPolar(const Eigen::VectorXd& state)
{
    double px = state(0);
    double py = state(1);
    double vx = state(2);
    double vy = state(3);
    
    // px == 0 will cause bad things to happen so we fix it up here
    if (fabs(px) < 1e-3)
    {
        px = 1e-3;
    }
    
    // convert to polar
    double ro = sqrt(px * px + py * py);
    double phi = atan2(py,px);
    double phi_dot = (px*vx + py*vy)/ro;
    
    VectorXd polar(3);
    polar << ro, phi, phi_dot;
    
    return polar;
}
