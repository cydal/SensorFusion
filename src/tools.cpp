#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */

    // TODO: YOUR CODE HERE

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    // ... your code here
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero

    if ((estimations.size() == 0) || (ground_truth.size() == 0)) {
        std::cout << "Either vector may not be empty" << std::endl;
        return rmse;
    }
    //  * the estimation vector size should equal ground truth vector size
    // ... your code here

    if (estimations.size() != ground_truth.size()) {
        std::cout << "Both vector size must match" << std::endl;
        return rmse;
    }


    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    // ... your code here

    rmse = rmse / estimations.size();

    //calculate the squared root
    // ... your code here

    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    /**
    TODO:
      * Calculate a Jacobian here.
    */

    MatrixXd Hj(3,4);
    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    //TODO: YOUR CODE HERE

    //check division by zero

    double ps = pow(px,2) + pow(py, 2);

    if ((pow(px,2) + pow(py, 2)) < 0.001) {
        ps = 1;
    }

    Hj(0, 0) = px / sqrt(ps);
    Hj(0, 1) = py / sqrt(ps);
    Hj(0, 2) = 0;
    Hj(0, 3) = 0;
    Hj(1, 0) = -(py/ps);
    Hj(1, 1) = (px/ps);
    Hj(1, 2) = 0;
    Hj(1, 3) = 0;
    Hj(2, 0) = (py * ((vx*py)- (vy*px))) / pow(ps, 1.5);
    Hj(2, 1) = (px * ((vy*px)- (vx*py))) / pow(ps, 1.5);
    Hj(2, 2) = px / sqrt(ps);
    Hj(2, 3) = py / sqrt(ps);

    //compute the Jacobian matrix

    return Hj;}