/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "tools.h"

#include <assert.h>
#include <iostream>
#include <stdint.h>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

/**************************************************************************************************
 *  @brief Constructor
 *************************************************************************************************/
Tools::Tools()
{
}

/**************************************************************************************************
 *  @brief Destructor
 *************************************************************************************************/
Tools::~Tools()
{
}

/**************************************************************************************************
 *  @brief A helper method to calculate RMSE.
 *************************************************************************************************/
VectorXd Tools::CalculateRMSE(
        const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) const
{
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (uint32_t i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = (estimations[i] - ground_truth[i]);
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

/**************************************************************************************************
 *  @brief A helper method to calculate Jacobians.
 *************************************************************************************************/
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) const
{
    MatrixXd Hj(3, 4);

    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    double c1 = px * px + py * py;
    double c2 = sqrt(c1);
    double c3 = (c1 * c2);

    // Ensure numerical stability.
    if (fabs(c1) < 0.0001)
    {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        assert(0);
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0,
          -(py / c1), (px / c1), 0, 0,
          py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;
}
