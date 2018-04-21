#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  if (estimations.size() != ground_truth.size()){
    cout << "estimations is of size " << estimations.size()
         << " but ground_truth is of size " << estimations.size() << endl;
    return rmse;
  }
  if (estimations.size() == 0){
    cout << "estimtions has size 0." << endl;
    return rmse;
  }

  for (int i = 0; i < estimations.size(); i++){
    VectorXd e = estimations[i];
    VectorXd g = ground_truth[i];
    VectorXd squared = (e - g).array() * (e - g).array();
    rmse += squared;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float squared_sum = px * px + py * py;

  //check division by zero
  if(squared_sum < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  Hj << px / sqrt(squared_sum), py / sqrt(squared_sum), 0, 0,
        -py / squared_sum, px / squared_sum, 0, 0,
        py * (vx * py - vy * px) / pow(squared_sum, 1.5),
          px * (vy * px - vx * py) / pow(squared_sum, 1.5),
          px / sqrt(squared_sum),
          py / sqrt(squared_sum);

   return Hj;
}
