#ifndef MPC_CONTROL_MPCCONTROLER_H
#define MPC_CONTROL_MPCCONTROLER_H
#include "planning/DesireTrajectory.h"
#include "Eigen/Core"
struct ControlCommand
{
    double steer;
    double acc;
};
struct VehicleState
{
    double xpos;
    double ypos;
    double phi;
    double dphi;
    double speed;
};
class MPCController {
private:
    double lateral_error = 0.0;
    double lateral_error_rate = 0.0;
    double heading_error = 0.0;
    double heading_error_rate = 0.0;
    double station_error = 0.0;
    double speed_error = 0.0;
    // control time interval
    double ts_ = 0.01;
    // corner stiffness; front
    double cf_ = 150000;
    // corner stiffness; rear
    double cr_ = 90000;
    // distance between front and rear wheel center
    double wheelbase_ = 2.3;
    // mass of the vehicle
    double mass_ = 2500;
    // distance from front wheel center to COM
    double lf_ = 0.8;
    // distance from rear wheel center to COM
    double lr_ = 1.5;
    // rotational inertia
    double iz_ = 3000;
    // number of states, includes
    // lateral error, lateral error rate, heading error, heading error rate,
    // station error, velocity error,
    const int basic_state_size_ = 6;
    const int controls_ = 2;
    const int horizon_ = 10;
    // vehicle state matrix
    Eigen::MatrixXd matrix_a_;
    // vehicle state matrix (discrete-time)
    Eigen::MatrixXd matrix_ad_;
    // control matrix
    Eigen::MatrixXd matrix_b_;
    // control matrix (discrete-time)
    Eigen::MatrixXd matrix_bd_;
    // offset matrix
    Eigen::MatrixXd matrix_c_;
    // offset matrix (discrete-time)
    Eigen::MatrixXd matrix_cd_;
    // updated control authority weighting matrix
    Eigen::MatrixXd matrix_r_;
    // state weighting matrix
    Eigen::MatrixXd matrix_q_;
    // vehicle state matrix coefficients
    Eigen::MatrixXd matrix_a_coeff_;
    // 4 by 1 matrix; state matrix
    Eigen::MatrixXd matrix_state_;
    // parameters for mpc solver; number of iterations
    int mpc_max_iteration_ = 150;
    // parameters for mpc solver; threshold for computation
    double mpc_eps_ = 0.01;

public:
    MPCController();
    void ComputeLongitudinalLateralErrors(double t, VehicleState* state, DesireTrajectory* traj);
    void UpdateStateAnalyticalMatching();
    void UpdateMatrix(VehicleState* state);
    void ComputeControlCommand(double t, VehicleState* state, DesireTrajectory* traj, ControlCommand *cmd);
};


#endif //MPC_CONTROL_MPCCONTROLER_H
