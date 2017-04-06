#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter
{
    
private:
    
    Eigen::MatrixXd stateIdentity_;
    
public:
    
    // state vector
    Eigen::VectorXd x_;
    
    // state covariance matrix
    Eigen::MatrixXd P_;
    
    /**
     * Constructor
     */
    KalmanFilter();
    
    /**
     * Destructor
     */
    virtual ~KalmanFilter();
    
    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in);
    
    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param F the state transition matrix
     * @param Q the process noise matrix
     */
    void Predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q);
    
    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     * @param H The observation model
     * @param Ht the transpose of the observation model
     * @param R The measurement noise
     */
    void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &Ht, const Eigen::MatrixXd &R);
    
    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     * @param Hj A function that calculates the observation Jacobian from the current state
     * @param HoX The observation model function - converts the current state into the measurement space
     * @param R The measurement noise
     */
    void UpdateEKF(const Eigen::VectorXd &z, Eigen::MatrixXd (*Hj)(const Eigen::VectorXd& in), Eigen::VectorXd (*HoX)(const Eigen::VectorXd &in), const Eigen::MatrixXd &R);
    
};

#endif /* KALMAN_FILTER_H_ */
