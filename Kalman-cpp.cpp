#include <iostream>
#include <stdexcept>

#include <Eigen/Dense>


class KalmanFilter {

public:

    /**
    * Create a Kalman filter with the specified matrices.
    *   A - System dynamics matrix
    *   C - Output matrix
    *   Q - Process noise covariance
    *   R - Measurement noise covariance
    *   P - Estimate error covariance
    */
    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& B,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P)
        : A(A), C(C), Q(Q), B(B), R(R), P0(P),
        m(C.rows()), n(A.rows()), dt(dt), initialized(false),
        I(n, n), x_hat(n), x_hat_new(n)
    {
        I.setIdentity();
    }



    /**
    * Return the current state and time.
    */
    Eigen::VectorXd state() { return x_hat; };
    double time() { return t; };


    /**
    * Initialize the filter with a guess for initial states.
    */
    void init(double t0, const Eigen::VectorXd& x0) {
        x_hat = x0;
        P = P0;
        this->t0 = t0;
        t = t0;
        initialized = true;
    }

    void init() {
        x_hat.setZero();
        P = P0;
        t0 = 0;
        t = t0;
        initialized = true;
    }

    /**
    * Update the estimated state based on measured values. The
    * time step is assumed to remain constant.
    */
    void update(const Eigen::VectorXd& u, const Eigen::VectorXd& y) {

        if (!initialized)
            throw std::runtime_error("Filter is not initialized!");

        //prediction
        x_hat_new = (A * x_hat) + (B * u);
        P = A * P * A.transpose() + Q;

        //correction and update
        K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
        x_hat_new += K * (y - C * x_hat_new);
        P = (I - K * C) * P;
        x_hat = x_hat_new;

        t += dt;
    }

    void update(const Eigen::VectorXd& u, const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

        this->A = A;
        this->dt = dt;
        update(u, y);
    }

    

private:

    // Matrices for computation
    Eigen::MatrixXd A, C, Q, R, B, P, K, P0;
    
    int m, n; // System dimensions
    
    double t0, t; // Initial and current time
    
    double dt; // Discrete time step
    
    bool initialized; // Is the filter initialized?

    Eigen::MatrixXd I; // n-size identity
    
    Eigen::VectorXd x_hat, x_hat_new; // Estimated states


};


int main() {

    int n = 2; // Number of states
    int m = 1; // Number of measurements
    double dt = 0.5; // Time step

    Eigen::MatrixXd A(n, n); // System dynamics matrix - prediction matrix
    Eigen::MatrixXd C(m, n); // Output matrix - sensor matrix
    Eigen::MatrixXd B(n, m); 
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance


    
    C << 1.0, 0.0;
    A << 1.0, dt, 0.0, 1.0;
    Q << .1, 0, 0, .1;
    R << 0.05;
    P << 0.01, 0, 0, 1;
    B << 0, dt;

    std::cout << "A: \n" << A << std::endl;
    std::cout << "C: \n" << C << std::endl;
    std::cout << "B: \n" << B << std::endl;
    std::cout << "Q: \n" << Q << std::endl;
    std::cout << "R: \n" << R << std::endl;
    std::cout << "P: \n" << P << std::endl;

    KalmanFilter kf(dt, A, C, Q, B, R, P);

    Eigen::MatrixXd x0(n, m);
    x0 << 0, 5;

    double t = 0;
    kf.init(t, x0);


    Eigen::VectorXd y(m);
    Eigen::VectorXd u(m);

    std::cout << "t = " << t << ", " << "x_hat[0]: " << kf.state().transpose() << std::endl;

    y << 2.2;
    u << -2;
    kf.update(u, y);   

    std::cout << "t = " << t << ", " << ", x_hat[" << 1 << "] = " << kf.state().transpose() << std::endl;

    return 0;
}