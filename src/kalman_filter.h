/**************************************************************************************************
 *  INCLUDE GUARD
 *************************************************************************************************/
#if !defined(KALMAN_FILTER_H_)
#define KALMAN_FILTER_H_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "Eigen/Dense"

#include <stdint.h>

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class KalmanFilter
{
    public:

        // state vector
        Eigen::VectorXd x_;

        // state covariance matrix
        Eigen::MatrixXd P_;

        // state transition matrix
        Eigen::MatrixXd F_;

        // process covariance matrix
        Eigen::MatrixXd Q_;

        // measurement matrix
        Eigen::MatrixXd H_;

        // measurement covariance matrix
        Eigen::MatrixXd R_;

        static const uint32_t numStates;
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
         * @param F_in Transition matrix
         * @param H_in Measurement matrix
         * @param R_in Measurement covariance matrix
         * @param Q_in Process covariance matrix
         */
        void Init(
                Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

        /**
         * @brief Prediction Predicts the state and the state covariance using the process model
         * @param delta_T Time between k and k+1 in s
         */
        void Predict();

        /**
         * @brief Updates the state by using standard Kalman Filter equations for Laser
         * @param z The measurement at k+1
         */
        void UpdateLaser(const Eigen::VectorXd &z);

        /**
         * @brief Updates the state by using Extended Kalman Filter equations for Radar
         * @param z The measurement at k+1
         */
        void UpdateRadar(const Eigen::VectorXd &z);

    private:
        /**
         * @brief Helper function to carry out the sensor independent part of the update step.
         */
        void Update(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred);

        /**
         *  @brief Bring angle in range of pi to -pi
         */
        void bringInRange(double &phi);
};

#endif /* KALMAN_FILTER_H_ */
