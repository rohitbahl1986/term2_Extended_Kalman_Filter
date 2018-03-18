/**************************************************************************************************
 *  INCLUDE GUARD
 *************************************************************************************************/
#if !defined(TOOLS_H_)
#define TOOLS_H_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "Eigen/Dense"
#include <vector>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class Tools
{
    public:
        /**
         * @brief Constructor.
         */
        Tools();

        /**
         * @brief Destructor.
         */
        virtual ~Tools();

        /**
         * @brief A helper method to calculate RMSE.
         */
        VectorXd CalculateRMSE(
                const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) const;

        /**
         * @brief A helper method to calculate Jacobians.
         */
        MatrixXd CalculateJacobian(const VectorXd& x_state) const;

};

#endif /* TOOLS_H_ */
