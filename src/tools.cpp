#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
	rmse << 0,0,0,0;
  
    if ((estimations.size() != ground_truth.size()) && (!(ground_truth.size()>0)))
    {
        cout << "Invalid estimation size or ground truth size!" << endl;
        return rmse;
    }
	//accumulate squared residuals
	VectorXd rmse_sum_squared(4);
	VectorXd val_diff(4);
	rmse_sum_squared << 0,0,0,0;
	for(int i=0; i < estimations.size(); ++i)
  {
	    val_diff = estimations[i] - ground_truth[i];
      rmse_sum_squared = rmse_sum_squared.array() + (val_diff.array()*val_diff.array());
	}

	//calculate the mean
	VectorXd rmse_sum_squared_mean(4);
  rmse_sum_squared_mean = rmse_sum_squared.array()/estimations.size();

	//calculate the squared root
  rmse = rmse_sum_squared_mean.array().sqrt();
	
  //return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  // Jacobian is
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  float px2py2 = px*px+py*py;
  float px2py2_3 = px2py2*px2py2*px2py2;
	//check division by zero
	if (px2py2 < 1e-6) {
	    cout << "Error, division by zero!" << endl;
	}
	else {
	//compute the Jacobian matrix
	Hj << px/(sqrt(px2py2)), py/(sqrt(px2py2)), 0 ,0,
	     -py/px2py2,         px/px2py2,         0, 0,
	      py*(vx*py-vy*px)/sqrt(px2py2_3), px*(vy*px-vx*py)/sqrt(px2py2_3), px/(sqrt(px2py2)), py/(sqrt(px2py2));
	}
	return Hj;
}
