/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "FusionEKF.h"

#include "tools.h"

#include "Eigen/Dense"

#include <iostream>
#include <stdint.h>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**************************************************************************************************
 *  CLASS MEMBER FUNCTIONS
 *************************************************************************************************/

/**************************************************************************************************
 *  @brief Constructor
 *************************************************************************************************/
FusionEKF::FusionEKF()
{
    m_is_initialized = false;

    m_previous_timestamp = 0;

    // initializing matrices
    m_R_laser = MatrixXd(2, 2);
    m_R_radar = MatrixXd(3, 3);
    m_H_laser = MatrixXd(2, 4);
    m_H_jacob = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    m_R_laser << 0.0225, 0,
                 0, 0.0225;

    //measurement covariance matrix - radar
    m_R_radar << 0.09, 0, 0,
                 0, 0.0009, 0,
                 0, 0, 0.09;

    // state vector
    m_ekf.x_ = VectorXd(KalmanFilter::numStates);

    // state covariance matrix
    m_ekf.P_ = MatrixXd(KalmanFilter::numStates, KalmanFilter::numStates);
    m_ekf.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

    // state transition matrix
    m_ekf.F_ = MatrixXd(KalmanFilter::numStates, KalmanFilter::numStates);
    m_ekf.F_ << 1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;

    // process covariance matrix
    m_ekf.Q_ = MatrixXd(KalmanFilter::numStates, KalmanFilter::numStates);

    //measurement matrix
    m_H_laser = MatrixXd(2, KalmanFilter::numStates);
    m_H_laser << 1, 0, 0, 0,
                 0, 1, 0, 0;

    m_noise_ax = 9;
    m_noise_ay = 9;
}

/**************************************************************************************************
 *  @brief Destructor
 *************************************************************************************************/
FusionEKF::~FusionEKF()
{
}

/**************************************************************************************************
 *  @brief Start the measurement process
 *************************************************************************************************/
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    // Initialization
    if (m_is_initialized == false)
    {
        // first measurement
        cout << "EKF: " << endl;
        m_ekf.x_ = VectorXd(4);

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            initWithRadarData(measurement_pack);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            initWithLaserData(measurement_pack);
        }

        // Initialize the previous_timestamp with the stamp of the first measurement.
        m_previous_timestamp = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        m_is_initialized = true;

        return;
    }
    else
    {
        // Prediction Step
        //compute the time elapsed between the current and previous measurements
        double dt = (measurement_pack.timestamp_ - m_previous_timestamp) / 1000000.0;
        m_previous_timestamp = measurement_pack.timestamp_;

        // Prepare the F matrix
        m_ekf.F_(0, 2) = dt;
        m_ekf.F_(1, 3) = dt;

        double dt_square = dt * dt;
        double dt_cube = dt_square * dt;
        double dt_quad = dt_cube * dt;

        //set the process covariance matrix Q
        m_ekf.Q_ = MatrixXd(4, 4);
        m_ekf.Q_ << dt_quad / 4 * m_noise_ax, 0, dt_cube / 2 * m_noise_ax, 0, 0,
                    dt_quad / 4 * m_noise_ay, 0, dt_cube / 2 * m_noise_ay, dt_cube / 2 * m_noise_ax,
                    0, dt_square * m_noise_ax, 0, 0,
                    dt_cube / 2 * m_noise_ay,0, dt_square * m_noise_ay;

        m_ekf.Predict();

        // Update Step
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            // Radar updates
            m_ekf.H_.resize(3, 4);
            m_ekf.H_ = tools.CalculateJacobian(m_ekf.x_);
            m_ekf.R_.resize(3, 3);
            m_ekf.R_ = m_R_radar;
            m_ekf.UpdateRadar(measurement_pack.raw_measurements_);
        }
        else
        {
            // Laser updates
            m_ekf.H_.resize(2, 4);
            m_ekf.H_ = m_H_laser;
            m_ekf.R_.resize(2, 2);
            m_ekf.R_ = m_R_laser;
            m_ekf.UpdateLaser(measurement_pack.raw_measurements_);
        }
    }

    // print the output
    cout << "x_ = " << m_ekf.x_ << endl;
    cout << "P_ = " << m_ekf.P_ << endl;
}

/**************************************************************************************************
 *  @brief Initialize the state vector with Radar data.
 *************************************************************************************************/
void FusionEKF::initWithRadarData(const MeasurementPackage &measurement_pack)
{
    // Convert radar from polar to Cartesian coordinates and initialize state.
    double rho = measurement_pack.raw_measurements_[0];
    double phi = measurement_pack.raw_measurements_[1];
    double rho_dot = measurement_pack.raw_measurements_[2];

    // X-co-ordinate is the projection of the radial distance on the X axis
    double position_x = rho * cos(phi);

    // Ensure numerical stability while computing Jacobian
    if (fabs(position_x) < 0.0001)
    {
        position_x = 0.0001;
    }

    //Y-co-ordinate is the projection of the radial distance on the Y axis
    double position_y = rho * sin(phi);

    // Ensure numerical stability while computing Jacobian
    if (fabs(position_y) < 0.0001)
    {
        position_y = 0.0001;
    }

    double velocity_x = rho_dot * cos(phi);
    double velocity_y = rho_dot * sin(phi);

    m_ekf.x_ << position_x, position_y, velocity_x, velocity_y;

    cout << "Init done with Radar data" << endl;
}

/**************************************************************************************************
 *  @brief Initialize the state vector with Laser data.
 *************************************************************************************************/
void FusionEKF::initWithLaserData(const MeasurementPackage &measurement_pack)
{

    // Initialize state for Laser data.
    m_ekf.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    cout << "Init done with Laser data" << endl;
}
