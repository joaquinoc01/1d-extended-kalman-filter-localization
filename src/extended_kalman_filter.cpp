#include <iostream>
#include <random>

#include "extended_kalman_filter.hpp"

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::VectorXf& initial_state,
    const Eigen::MatrixXf& error_covariance,
    const Eigen::MatrixXf& process_noise,
    const Eigen::MatrixXf& measurement_noise)
    : KalmanFilter(initial_state, error_covariance), dt_(1.0f)
{
    state_ = initial_state;
    P_ = error_covariance;
    R_ = process_noise;
    Q_ = measurement_noise;
}

// Motion model: predicts the next state
Eigen::VectorXf ExtendedKalmanFilter::motionModel(const Eigen::VectorXf& prev_state, const Eigen::VectorXf& control_input)
{
    float acceleration = control_input[0];

    Eigen::VectorXf state(2);
    state[0] = prev_state[0] + prev_state[1] * dt_ + 0.5f * acceleration * dt_ * dt_;
    state[1] = prev_state[1] + acceleration * dt_;

    return state;
}

// Jacobian of the motion model
Eigen::MatrixXf ExtendedKalmanFilter::computeJacobianG()
{
    Eigen::Matrix2f G;
    G << 1.0f, dt_,
         0.0f, 1.0f;
    return G;
}

// Jacobian of the measurement model
Eigen::RowVector2f ExtendedKalmanFilter::computeJacobianH()
{
    Eigen::RowVector2f H;
    H << 1.0f, 0.0f;
    return H;
}

// Covariance prediction
Eigen::MatrixXf ExtendedKalmanFilter::predictCovariance(const Eigen::MatrixXf& G, const Eigen::MatrixXf& P, const Eigen::MatrixXf& R)
{
    return G * P * G.transpose() + R;
}

// Measurement model: returns predicted measurement
Eigen::VectorXf ExtendedKalmanFilter::measurementModel(const Eigen::VectorXf& predicted_state)
{
    Eigen::VectorXf z_pred(1);
    z_pred << predicted_state[0];  // Only position measured
    return z_pred;
}

// Kalman gain calculation
Eigen::MatrixXf ExtendedKalmanFilter::computeKalmanGain(const Eigen::MatrixXf& cov_matrix, const Eigen::MatrixXf& H, const Eigen::MatrixXf& Q)
{
    Eigen::MatrixXf S = H * cov_matrix * H.transpose() + Q;
    return cov_matrix * H.transpose() * S.inverse();
}

// State update
Eigen::VectorXf ExtendedKalmanFilter::updateState(
    const Eigen::VectorXf& predicted_state,
    const Eigen::MatrixXf& K,
    const Eigen::VectorXf& measurement,
    const Eigen::VectorXf& predicted_measurement)
{
    return predicted_state + K * (measurement - predicted_measurement);
}

// Covariance update
Eigen::MatrixXf ExtendedKalmanFilter::updateCovariance(
    const Eigen::MatrixXf& K,
    const Eigen::MatrixXf& H,
    const Eigen::MatrixXf& predicted_cov)
{
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(predicted_cov.rows(), predicted_cov.cols());
    return (I - K * H) * predicted_cov;
}

void ExtendedKalmanFilter::step(const Eigen::VectorXf& control_input, const Eigen::VectorXf& measurement, float dt) {
    // Predict
    Eigen::VectorXf predicted_state = motionModel(state_, control_input);
    Eigen::MatrixXf G = computeJacobianG();
    Eigen::MatrixXf predicted_cov = predictCovariance(G, P_, R_);

    // Measurement prediction
    Eigen::VectorXf z_pred = measurementModel(predicted_state);
    Eigen::MatrixXf H = computeJacobianH();
    Eigen::MatrixXf K = computeKalmanGain(predicted_cov, H, Q_);

    // Update
    Eigen::VectorXf updated_state = updateState(predicted_state, K, measurement, z_pred);
    Eigen::MatrixXf updated_cov = updateCovariance(K, H, predicted_cov);

    // Save
    state_ = updated_state;
    P_ = updated_cov;
}

#include <iostream>
#include <random>
#include "extended_kalman_filter.hpp"

int main() {
    // Constants
    const float dt = 1.0f;
    const int steps = 10;

    // Initial state and covariance
    Eigen::Vector2f initial_state(0.0f, 0.0f);  // [position, velocity]
    Eigen::Matrix2f P = Eigen::Matrix2f::Identity() * 1.0f;

    // Process noise (acceleration uncertainty)
    Eigen::Matrix2f R = Eigen::Matrix2f::Identity() * 0.1f;

    // Measurement noise (position measurement only)
    Eigen::MatrixXf Q(1, 1);
    Q << 0.5f;

    // Create EKF instance
    ExtendedKalmanFilter ekf(initial_state, P, R, Q);

    // True motion simulation setup
    float true_position = 0.0f;
    float true_velocity = 1.0f;
    float acceleration = 0.1f;

    std::default_random_engine gen;
    std::normal_distribution<float> noise(0.0f, std::sqrt(Q(0, 0)));

    for (int t = 0; t < steps; ++t) {
        // Simulate true motion
        true_position += true_velocity * dt + 0.5f * acceleration * dt * dt;
        true_velocity += acceleration * dt;

        // Create control input vector
        Eigen::VectorXf u(1);
        u << acceleration;

        // Simulate measurement with noise
        float noisy_measurement = true_position + noise(gen);
        Eigen::VectorXf z(1);
        z << noisy_measurement;

        // Run EKF update
        ekf.step(u, z, dt);

        // Output results
        Eigen::VectorXf estimate = ekf.getState();
        std::cout << "Step " << t << ":\n";
        std::cout << "  True position:      " << true_position << "\n";
        std::cout << "  Noisy measurement:  " << noisy_measurement << "\n";
        std::cout << "  Estimated position: " << estimate[0] << "\n";
        std::cout << "  Estimated velocity: " << estimate[1] << "\n\n";
    }

    return 0;
}
