#pragma once

#include <Eigen/Dense>

// ---------- Base KalmanFilter class ----------
class KalmanFilter {
public:
    KalmanFilter(const Eigen::VectorXf& initial_state, const Eigen::MatrixXf& error_covariance)
        : state_(initial_state), P_(error_covariance) {}

protected:
    Eigen::VectorXf state_;
    Eigen::MatrixXf P_;  // Covariance matrix
    Eigen::MatrixXf R_;  // Process noise covariance
    Eigen::MatrixXf Q_;  // Measurement noise covariance
};

// ---------- ExtendedKalmanFilter class ----------
class ExtendedKalmanFilter : public KalmanFilter {
public:
    ExtendedKalmanFilter(const Eigen::VectorXf& initial_state,
                         const Eigen::MatrixXf& error_covariance,
                         const Eigen::MatrixXf& process_noise,
                         const Eigen::MatrixXf& measurement_noise);

    Eigen::VectorXf getState() const { return state_; }
    Eigen::MatrixXf getCovariance() const { return P_; }
    void setState(const Eigen::VectorXf& new_state) { state_ = new_state; }
    void setCovariance(const Eigen::MatrixXf& new_P) { P_ = new_P; }
    float dt_;

    // Main EKF step function
    void step(const Eigen::VectorXf& control_input, const Eigen::VectorXf& measurement, float dt);
    
private:
    Eigen::VectorXf motionModel(const Eigen::VectorXf& prev_state, const Eigen::VectorXf& control_input);
    Eigen::MatrixXf computeJacobianG();
    Eigen::RowVector2f computeJacobianH();
    Eigen::MatrixXf predictCovariance(const Eigen::MatrixXf& G, const Eigen::MatrixXf& P, const Eigen::MatrixXf& R);
    Eigen::VectorXf measurementModel(const Eigen::VectorXf& predicted_state); // <-- Fixed return type
    Eigen::MatrixXf computeKalmanGain(const Eigen::MatrixXf& cov_matrix, const Eigen::MatrixXf& H, const Eigen::MatrixXf& Q);
    Eigen::VectorXf updateState(const Eigen::VectorXf& predicted_state,
                                const Eigen::MatrixXf& K,
                                const Eigen::VectorXf& measurement,
                                const Eigen::VectorXf& predicted_measurement);
    Eigen::MatrixXf updateCovariance(const Eigen::MatrixXf& K,
                                     const Eigen::MatrixXf& H,
                                     const Eigen::MatrixXf& predicted_cov);
};
