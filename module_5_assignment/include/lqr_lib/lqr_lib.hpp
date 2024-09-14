#ifndef LQR_HPP
#define LQR_HPP

#include <Eigen/Dense>
#include <iostream>


class LQR {
  public:
    // Eigen Matrix Initialize
    using StateMatrix = Eigen::MatrixXd;
    using InputMatrix = Eigen::MatrixXd;
    using StateVector = Eigen::VectorXd;
    using InputVector = Eigen::VectorXd;

    // Constructor to initialize LQR with cost matricds Q, R, and predict horizon
    LQR(StateMatrix const& Q, InputMatrix const& R, int horizon);
    StateMatrix getA(double yaw, double v, double dt);
    InputMatrix getB(double yaw, double dt);
    void updateMatrices(StateMatrix const &A, InputMatrix const& B);

    // Compute the Riccatti equation solution to update the gain matrix K
    void computeRiccati(InputMatrix B, StateMatrix A);
    InputVector computeOptimalInput(StateVector const& state_error);
    StateMatrix K_;
    ~LQR();
  
  private:
    StateMatrix A_;
    InputMatrix B_;
    StateMatrix Q_;
    InputMatrix R_;
    StateMatrix P_;
    int horizon_;
};

#endif