#include <render/OpenGLPlot.h>
#include "mpc_solver/mpc_solver.h"
#include "common/log.h"
// 双积分系统
// d2y = u
// d [ x ] = [0  1] [x ] + [0] u
//   [dx ]   [0  0] [dx]   [1]
//   y   =  [ 1  0][ x]
//                 [dx]
int main() {
    const int STATES = 2;
    int CONTROLS = 1;
    const int HORIZON = 10;
    const double EPS = 0.01;
    const int MAX_ITER = 100;
    const double Ts = 0.1;
    Eigen::MatrixXd A(STATES, STATES);
    A << 0, 1, 0, 0;

    Eigen::MatrixXd B(STATES, CONTROLS);
    B << 0, 1;

    Eigen::MatrixXd C(STATES, 1);
    C << 0, 0;

    Eigen::MatrixXd Q(STATES, STATES);
    Q << 1, 0, 0, 1;

    Eigen::MatrixXd R(CONTROLS, CONTROLS);
    R << 1;

    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(STATES, STATES) + A*Ts;
    Eigen::MatrixXd Bd = B * Ts;
    Eigen::MatrixXd lower_bound(CONTROLS, 1);
    lower_bound << -100;

    Eigen::MatrixXd upper_bound(CONTROLS, 1);
    upper_bound << 100;

    Eigen::MatrixXd initial_state(STATES, 1);
    initial_state << 0, 0;

    Eigen::MatrixXd reference_state(STATES, 1);
    reference_state << 1, 0;

    std::vector<Eigen::MatrixXd> reference(HORIZON, reference_state);

    Eigen::MatrixXd control_matrix(CONTROLS, 1);
    control_matrix << 0;
    std::vector<Eigen::MatrixXd> control(HORIZON, control_matrix);

    double t = 0.0;
    Eigen::MatrixXd X = initial_state;
    std::vector<double> ts;
    std::vector<double> xs;
    for(int i = 0; i < 100; ++i)
    {
        xs.push_back(X(0));
        ts.push_back(t);

        SolveLinearMPC(Ad, Bd, C, Q, R, lower_bound, upper_bound, X,
                       reference, EPS, MAX_ITER, &control);
        double u = control[0](0, 0);
        AINFO <<t<<","<<X(0)<<","<<X(1)<<","<<u;
        X = Ad*X +Bd*u;
        t += Ts;
    }
    OpenGLPlot plot;
    plot.AddCurve(Curve2D(ts, xs));
    plot.AddCurve(Curve2D(std::vector<double>{0,10}, std::vector<double>{1,1},1.f,0.f,0.f));
    plot.MainLoop();
    return 0;
}
