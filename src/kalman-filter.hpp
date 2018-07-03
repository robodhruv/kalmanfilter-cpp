/**
* Kalman filter header file.
*
* @author: Dhruv Shah, Hayk Martirosyan
* @date: 07/03/2018
*/

#include <Eigen/Dense>

#pragma once

class KalmanFilter {

public:

	/**
	* Create a Kalman filter with the specified matrices.
	*   A - System dynamics matrix
	*   B - Input matrix
	*   C - Output matrix
	*   Q - Process noise covariance
	*   R - Measurement noise covariance
	*   P - Estimate error covariance
	*/
	KalmanFilter(
			const Eigen::MatrixXd& A,
			const Eigen::MatrixXd& B,
			const Eigen::MatrixXd& C,
			const Eigen::MatrixXd& Q,
			const Eigen::MatrixXd& R,
			const Eigen::MatrixXd& P
	);

	/**
	* Initialize the filter with initial states as zero.
	*/
	void init();

	/**
	* Initialize the filter with a guess for initial states.
	*/
	void init(const Eigen::VectorXd& x0);

	/**
	* Update the prediction based on control input.
	*/
	void predict(const Eigen::VectorXd& u);

	/**
	* Update the estimated state based on measured values.
	*/
	void update(const Eigen::VectorXd& y);

	/**
	* Update the dynamics matrix.
	*/
	void update_dynamics(const Eigen::MatrixXd A);

	/**
	* Update the output matrix.
	*/
	void update_output(const Eigen::MatrixXd C);

	/**
	* Return the current state.
	*/
	Eigen::VectorXd state() { return x_hat; };

private:

	// Matrices for computation
	Eigen::MatrixXd A, B, C, Q, R, P, K, P0;

	// System dimensions
	int m, n, c;

	// Is the filter initialized?
	bool initialized = false;

	// n-size identity
	Eigen::MatrixXd I;

	// Estimated states
	Eigen::VectorXd x_hat;
};
