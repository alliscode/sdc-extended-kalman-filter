#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;
    previous_timestamp_ = 0;
    
    // initializing matrices
    F_ = MatrixXd(4, 4);
    Q_ = MatrixXd(4, 4);
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    
    // initializze the state transition matrix F
    F_ <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    H_laser_ <<
    1, 0, 0, 0,
    0, 1, 0, 0;
    
    H_laser_t_ = H_laser_.transpose();
    
    //measurement covariance matrix - laser
    R_laser_ <<
    0.0225, 0,
    0, 0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ <<
    0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_)
    {
        // We need to initialize the state variables in ekf but the correct way to do this depends
        // on the type of reading that we have...
        
        VectorXd x(4);
        MatrixXd p = MatrixXd::Identity(4, 4);
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            // The mean vector will be initialized by converting the polar radar readings to cartesian coordinates.
            // Since the radar measurement gives us position and velocity readings directly, the state covariance
            // matrix will be initialized with the identity matrix as created above.
            
            float ro = measurement_pack.raw_measurements_[0];
            float phi = measurement_pack.raw_measurements_[1];
            float ro_dot = measurement_pack.raw_measurements_[2];
            
            x << ro * cos(phi), ro * sin(phi), ro_dot * cos(phi), ro_dot * sin(phi);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            // The mean vector will be initialized with the x and y position measurements provided by the lidar reading.
            // Since lidar does not give us velocity measurements directly, we need to increase the level of uncertainty in
            // the corresponding elements of our covariance matrix. 1000 has been chosen arbitrarily.
            
            x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
            p(2, 2) = 1000;
            p(3, 3) = 1000;
        }
        
        // Now we can initialize the extended Kalman filter with the state variables created above
        ekf_.Init(x, p);
        
        // initialize the timestamp
        previous_timestamp_ = measurement_pack.timestamp_;
        
        // done initializing, no need to predict or update until the next sample comes in
        is_initialized_ = true;
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    
    // get the time delta between this sample and the previous
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;
    
    // update the state transition matrix
    F_(0, 2) = dt;
    F_(1, 3) = dt;
    
    // update the process noise with noise_ax = noise_ay = 9
    float noise_ax = 9;
    float noise_ay = 9;
    
    float dt4 = pow(dt, 4) / 4.0;
    float dt3 = pow(dt, 3) / 2.0;
    float dt2 = pow(dt, 2);
    
    Q_ <<
    dt4*noise_ax, 0, dt3*noise_ax, 0,
    0, dt4*noise_ay, 0, dt3*noise_ay,
    dt3*noise_ax, 0, dt2*noise_ax, 0,
    0, dt3*noise_ay, 0, dt2*noise_ay;
    
    // Step 1. Predict the state using the updated state transition function and process noise
    ekf_.Predict(F_, Q_);
    
    // Step 2. Update the state using the current reading, the details of this depend on the sensor type...
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // Radar updates
        ekf_.UpdateEKF(measurement_pack.raw_measurements_, Tools::CalculateJacobian, Tools::CartesianToPolar, R_radar_);
        
    } else
    {
        // Laser updates
        ekf_.Update(measurement_pack.raw_measurements_, H_laser_, H_laser_t_, R_laser_);
    }
    
    // save the current timestamp for the go round
    previous_timestamp_ = measurement_pack.timestamp_;
}

