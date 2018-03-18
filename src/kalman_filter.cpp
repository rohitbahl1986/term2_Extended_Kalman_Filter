/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "kalman_filter.h"

#include <iostream>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**************************************************************************************************
 *  CONSTANT DATA
 *************************************************************************************************/
const uint32_t KalmanFilter::numStates = 4;

/**************************************************************************************************
 *  CLASS MEMBER FUNCTIONS
 *************************************************************************************************/

/**************************************************************************************************
 *  @brief Constructor
 *************************************************************************************************/
KalmanFilter::KalmanFilter()
{
}

/**************************************************************************************************
 *  @brief Destructor
 *************************************************************************************************/
KalmanFilter::~KalmanFilter()
{
}

/**************************************************************************************************
 *  @brief Initialize the state of Kalman Filter
 *************************************************************************************************/
void KalmanFilter::Init(
        VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in,
        MatrixXd &Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

/**************************************************************************************************
 *  @brief Predicts the state and the state covariance using the process model
 *************************************************************************************************/
void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

/**************************************************************************************************
 *  @brief Updates the state by using Kalman Filter equations for Laser
 *************************************************************************************************/
void KalmanFilter::UpdateLaser(const VectorXd &z)
{
    // Projection of the predicted state on the measurement space
    VectorXd z_pred = H_ * x_;

    Update(z, z_pred);
}

/**************************************************************************************************
 *  @brief Updates the state by using Kalman Filter equations for Radar
 *************************************************************************************************/
void KalmanFilter::UpdateRadar(const VectorXd &z)
{
    // Convert cartesian co-ordinates into polar.

    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
    double px_2 = px * px;
    double py_2 = py * py;

    // Ensure numerical stability
    if (px_2 < 0.0001)
    {
        px_2 = 0.0001;
    }

    if (py_2 < 0.0001)
    {
        px_2 = 0.0001;
    }

    double rho, phi, rho_dot;
    rho = sqrt(px_2 + py_2);
    phi = atan2(py, px);

    rho_dot = (px * vx + py * vy) / rho;

    bringInRange(phi);

    // Projection of the predicted state on the measurement space
    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;

    Update(z, z_pred);
}

/**************************************************************************************************
 *  @brief Helper function to carry out the sensor independent part of the update step.
 *************************************************************************************************/
void KalmanFilter::Update(const VectorXd &z, const VectorXd &z_pred)
{
    // Error between the actual measurement and the predicted measurement
    VectorXd y = z - z_pred;
    bringInRange(y[1]);
    MatrixXd Ht = H_.transpose();

    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;

    // Calculate Kalman Gain
    MatrixXd K = PHt * Si;

    // Update estimate of the state mean vector
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    // Update the estimate of the state co-varinace matrix
    P_ = (I - K * H_) * P_;
}

/**************************************************************************************************
 *  @brief Bring angle in range of pi to -pi
 *************************************************************************************************/
void KalmanFilter::bringInRange(double &phi)
{
    while (phi > M_PI || phi < -M_PI)
    {
        double delta = (phi > M_PI) ? -M_PI : M_PI;
        phi +=delta;
    }
}
