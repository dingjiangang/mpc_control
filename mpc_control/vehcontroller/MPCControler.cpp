#include "MPCControler.h"
#include "mpc_solver/mpc_solver.h"
#include <cmath>
#include "Eigen/LU"
#include "common/log.h"

using Matrix = Eigen::MatrixXd;
MPCController::MPCController()
{
    wheelbase_ = lf_ + lr_;
    // Matrix init operations.
    matrix_a_  = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    matrix_a_(4, 5) = 1.0;
    matrix_a_(5, 5) = 0.0;
    matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(2, 3) = 1.0;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
    matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
    matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_b_(4, 1) = 0.0;
    matrix_b_(5, 1) = -1.0;
    matrix_bd_ = matrix_b_ * ts_;
    matrix_c_ = Matrix::Zero(basic_state_size_, 1);
    matrix_c_(5, 0) = 1.0;
    matrix_cd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_state_ = Matrix::Zero(basic_state_size_, 1);
    matrix_r_ = Matrix::Identity(controls_, controls_);
    matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    double qarray[] = {2.0, 2.0, 2.0, 2.0, 0.0, 0.0};
    for (int i = 0; i < 6; ++i) {
        matrix_q_(i, i) = qarray[i];
    }
}
void MPCController::ComputeLongitudinalLateralErrors(double t, VehicleState* state, DesireTrajectory* traj)
{
    auto reference_point = traj->QueryPathPointAtTime(t);
    double xTarget = reference_point.x;
    double yTarget = reference_point.y;
    double dxTarget = state->xpos - xTarget;
    double dyTarget = state->ypos - yTarget;
    double KParam = 1.0;
    speed_error = reference_point.v - state->speed*cos(heading_error)/KParam;
    station_error   = -(dxTarget*cos(reference_point.phi) + dyTarget*sin(reference_point.phi));
    heading_error      = state->phi - reference_point.phi;
    double dphiDes = reference_point.v*reference_point.Curvature;
    heading_error_rate = state->dphi - dphiDes;
    lateral_error      = dyTarget*cos(reference_point.phi) - dxTarget*sin(reference_point.phi);
    lateral_error_rate = state->speed*sin(heading_error);

}
void MPCController::UpdateStateAnalyticalMatching()
{
    matrix_state_(0, 0) = lateral_error;
    matrix_state_(1, 0) = lateral_error_rate;
    matrix_state_(2, 0) = heading_error;
    matrix_state_(3, 0) = heading_error_rate;
    matrix_state_(4, 0) = station_error;
    matrix_state_(5, 0) = speed_error;
}
void MPCController::UpdateMatrix(VehicleState* state) {
    double v = state->speed;
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

    Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
    matrix_ad_ = (matrix_i + ts_ * 0.5 * matrix_a_) *
                 (matrix_i - ts_ * 0.5 * matrix_a_).inverse();

    matrix_c_(1, 0) = (lr_ * cr_ - lf_ * cf_) / mass_ / v - v;
    matrix_c_(3, 0) = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_ / v;
    matrix_cd_ = matrix_c_ * heading_error_rate * ts_;
}
void MPCController::ComputeControlCommand(double t, VehicleState* state, DesireTrajectory* traj, ControlCommand *cmd)
{
    ComputeLongitudinalLateralErrors(t, state, traj);
    UpdateStateAnalyticalMatching();
    UpdateMatrix(state);
    Eigen::MatrixXd control_matrix(controls_, 1);
    control_matrix << 0, 0;

    Eigen::MatrixXd reference_state(basic_state_size_, 1);
    reference_state << 0, 0, 0, 0, 0, 0;

    std::vector<Eigen::MatrixXd> reference(horizon_, reference_state);

    Eigen::MatrixXd lower_bound(controls_, 1);
    lower_bound << -100, -10;

    Eigen::MatrixXd upper_bound(controls_, 1);
    upper_bound << 100, 10;

    std::vector<Eigen::MatrixXd> control(horizon_, control_matrix);

    if (SolveLinearMPC(
            matrix_ad_, matrix_bd_, matrix_cd_, matrix_q_,
            matrix_r_, lower_bound, upper_bound, matrix_state_, reference,
            mpc_eps_, mpc_max_iteration_, &control) != true) {
        AERROR << "MPC solver failed";
    } else {
        //AINFO << "MPC problem solved! ";
    }
    double steer_angle_feedback = control[0](0, 0);
    double steer_angle_feedforwardterm_updated_ = 0.0;
    double steer_angle =
            steer_angle_feedback + steer_angle_feedforwardterm_updated_;
    double acceleration_reference = 0.0;
    double acceleration_cmd = control[0](1, 0);
    cmd->steer = steer_angle;
    cmd->acc = acceleration_cmd;
}
