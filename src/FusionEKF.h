/**************************************************************************************************
 *  INCLUDE GUARD
 *************************************************************************************************/
#if !defined(FusionEKF_H_)
#define FusionEKF_H_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

#include "Eigen/Dense"
#include <fstream>
#include <string>
#include <vector>

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class FusionEKF
{
    public:

        /**
         * @brief Constructor
         */
        FusionEKF();

        /**
         * @brief Destructor
         */
        virtual ~FusionEKF();

        /**
         * @brief Run the whole flow of the Kalman Filter from here.
         */
        void ProcessMeasurement(const MeasurementPackage &measurement_pack);

        // Kalman Filter update and prediction math lives in here.
        KalmanFilter m_ekf;

    private:
        // check whether the tracking toolbox was initialized or not (first measurement)
        bool m_is_initialized;

        float m_noise_ax;
        float m_noise_ay;

        /**
         * @brief Initialize the state vector with Laser data.
         */
        void initWithLaserData(const MeasurementPackage &measurement_pack);

        /**
         * @brief Initialize the state vector with Radar data.
         */
        void initWithRadarData(const MeasurementPackage &measurement_pack);

        // previous timestamp
        long long m_previous_timestamp;

        // tool object used to compute Jacobian and RMSE
        Tools tools;
        Eigen::MatrixXd m_R_laser;
        Eigen::MatrixXd m_R_radar;
        Eigen::MatrixXd m_H_laser;
        Eigen::MatrixXd m_H_jacob;
};

#endif /* FusionEKF_H_ */
