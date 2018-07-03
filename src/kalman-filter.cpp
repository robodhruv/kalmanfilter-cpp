/**
* Implementation of KalmanFilter class.
*
* @author: Dhruv Shah, Hayk Martirosyan
* @date: 07/03/2018
*/

#include <iostream>
#include <iostream>

#include "kalman-filter.hpp"

KalmanFilter::KalmanFilter(
		const Eigen::MatrixXd& A,
		const Eigen::MatrixXd& B,
		const Eigen::MatrixXd& C,
		const Eigen::MatrixXd& Q,
		const Eigen::MatrixXd& R,
		const Eigen::MatrixXd& P)
	: A(A), B(B), C(C), Q(Q), R(R), P0(P),
		m(C.rows()), n(A.rows()), c(B.cols()), initialized(false),
		I(n, n), x_hat(n)
{
	I.setIdentity();
}

void KalmanFilter::init(const Eigen::VectorXd& x0) {

	x_hat = x0;
	P = P0;
	initialized = true;
}

void KalmanFilter::init() {

	x_hat.setZero();
	P = P0;
	initialized = true;
}

void KalmanFilter::predict(const Eigen::VectorXd& u) {

	if(!initialized) {
		std::cout << "Filter is not initialized! Initializing with trivial state.";
		init();
	}

	x_hat = A*x_hat + B*u;
	P = A*P*A.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

	K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
	x_hat += K * (y - C*x_hat);
	P = (I - K*C)*P;
}

void KalmanFilter::update_dynamics(const Eigen::MatrixXd A) {

	this->A = A;
}

void KalmanFilter::update_output(const Eigen::MatrixXd C) {

	this->C = C;
}