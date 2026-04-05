/*
 * kalman_predictor.cpp
 * Author: Satyam Kumar
 * Date: 2026-04-05
 *
 * Implements the KalmanPredictor class with a constant-velocity motion
 * model. Uses OpenCV's KalmanFilter to maintain state estimates and
 * predict future trolley positions for collision avoidance.
 *
 * State vector: [x, y, z, vx, vy, vz] — 3D position and velocity
 * Measurement:  [x, y, z]             — 3D position from ArUco tvecs
 *
 * The constant-velocity model assumes the trolley moves at a constant
 * speed with small random accelerations modelled as process noise.
 * This is appropriate for walking-speed trolley motion (~0.8 m/s)
 * over short prediction horizons (0.5 seconds).
 */

#include "kalman_predictor.hpp"
#include <cmath>
#include <iostream>

// ---------------------------------------------------------------------------
// Named constants
// ---------------------------------------------------------------------------

/// Number of state dimensions: [x, y, z, vx, vy, vz]
constexpr int STATE_DIM       = 6;

/// Number of measurement dimensions: [x, y, z]
constexpr int MEASUREMENT_DIM = 3;

/// No control input dimensions
constexpr int CONTROL_DIM     = 0;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

/**
 * @brief Construct a KalmanPredictor with timing and noise parameters.
 *
 * The filter is not initialised until the first measurement arrives
 * via update(). This avoids creating a filter with an arbitrary
 * initial position that could generate false collision warnings.
 *
 * @param dt Time step between consecutive frames in seconds.
 * @param processNoise Process noise covariance scalar (Q diagonal).
 * @param measurementNoise Measurement noise covariance scalar (R diagonal).
 */
KalmanPredictor::KalmanPredictor(float dt, float processNoise,
                                   float measurementNoise)
    : initialized_(false), dt_(dt),
      processNoise_(processNoise), measurementNoise_(measurementNoise)
{
    // Filter will be initialised on first measurement in update()
}

// ---------------------------------------------------------------------------
// init_() — One-time Kalman filter initialisation
// ---------------------------------------------------------------------------

/**
 * @brief Initialise the Kalman filter state with the first measurement.
 *
 * Sets up all filter matrices for the constant-velocity motion model:
 *   - Transition matrix F: implements x(t+dt) = x(t) + v(t)*dt
 *   - Measurement matrix H: extracts position from the state vector
 *   - Process noise Q: represents model uncertainty (random accelerations)
 *   - Measurement noise R: represents ArUco tvec sensor noise
 *   - Error covariance P: initial uncertainty (large = low confidence)
 *
 * @param p Initial 3D position from the first ArUco measurement.
 */
void KalmanPredictor::init_(const cv::Point3f& p) {
    // Create the Kalman filter with 6 state dimensions, 3 measurement
    // dimensions, and 0 control dimensions, using 32-bit floats.
    kf_ = cv::KalmanFilter(STATE_DIM, MEASUREMENT_DIM, CONTROL_DIM, CV_32F);

    // ---------------------------------------------------------------
    // Transition matrix F: constant-velocity kinematics
    //
    //   | 1  0  0  dt  0   0  |     x(t+dt)  = x(t)  + vx(t)*dt
    //   | 0  1  0  0   dt  0  |     y(t+dt)  = y(t)  + vy(t)*dt
    //   | 0  0  1  0   0   dt |     z(t+dt)  = z(t)  + vz(t)*dt
    //   | 0  0  0  1   0   0  |     vx(t+dt) = vx(t)  (constant)
    //   | 0  0  0  0   1   0  |     vy(t+dt) = vy(t)  (constant)
    //   | 0  0  0  0   0   1  |     vz(t+dt) = vz(t)  (constant)
    // ---------------------------------------------------------------
    cv::setIdentity(kf_.transitionMatrix);
    kf_.transitionMatrix.at<float>(0, 3) = dt_;  // x += vx * dt
    kf_.transitionMatrix.at<float>(1, 4) = dt_;  // y += vy * dt
    kf_.transitionMatrix.at<float>(2, 5) = dt_;  // z += vz * dt

    // ---------------------------------------------------------------
    // Measurement matrix H: we observe only position, not velocity
    //
    //   | 1  0  0  0  0  0 |     z_x = x
    //   | 0  1  0  0  0  0 |     z_y = y
    //   | 0  0  1  0  0  0 |     z_z = z
    // ---------------------------------------------------------------
    kf_.measurementMatrix = cv::Mat::zeros(MEASUREMENT_DIM, STATE_DIM, CV_32F);
    kf_.measurementMatrix.at<float>(0, 0) = 1.0f;
    kf_.measurementMatrix.at<float>(1, 1) = 1.0f;
    kf_.measurementMatrix.at<float>(2, 2) = 1.0f;

    // ---------------------------------------------------------------
    // Process noise covariance Q: how much we trust the motion model.
    // Small Q means we believe the constant-velocity model is accurate;
    // the filter will rely more on predictions and less on measurements.
    // ---------------------------------------------------------------
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar(processNoise_));

    // ---------------------------------------------------------------
    // Measurement noise covariance R: sensor noise of the ArUco tvec.
    // Represents the expected variance of the position measurement.
    // Typical ArUco tvec noise is 2-5 mm at 1 m distance, giving
    // a variance of approximately 5e-6 to 25e-6 m². We use 5e-3 as
    // a conservative estimate that allows the filter to smooth noise
    // while remaining responsive to genuine motion changes.
    // ---------------------------------------------------------------
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(measurementNoise_));

    // ---------------------------------------------------------------
    // Error covariance P: initial uncertainty. Set to 1.0 (large)
    // because we have no prior information about the trolley's
    // velocity at startup. The filter will converge within a few
    // frames as measurements arrive.
    // ---------------------------------------------------------------
    cv::setIdentity(kf_.errorCovPost, cv::Scalar(1.0));

    // Set initial state to the measured position with zero velocity
    kf_.statePost.at<float>(0) = p.x;   // x position
    kf_.statePost.at<float>(1) = p.y;   // y position
    kf_.statePost.at<float>(2) = p.z;   // z position
    kf_.statePost.at<float>(3) = 0.0f;  // vx = 0 (unknown at init)
    kf_.statePost.at<float>(4) = 0.0f;  // vy = 0 (unknown at init)
    kf_.statePost.at<float>(5) = 0.0f;  // vz = 0 (unknown at init)

    initialized_ = true;
    std::cout << "[KalmanPredictor] Initialised at position ("
              << p.x << ", " << p.y << ", " << p.z << ")\n";
}

// ---------------------------------------------------------------------------
// update() — Feed new marker detections into the filter
// ---------------------------------------------------------------------------

/**
 * @brief Update the Kalman filter with new ArUco marker detections.
 *
 * Computes the centroid of all detected marker tvecs (averaging x, y, z
 * across all marker IDs present) and uses it as the measurement for
 * the Kalman filter update step.
 *
 * If no markers are detected in this frame, only kf_.predict() is called
 * without a measurement correction step. This handles temporary occlusion
 * by extrapolating the last known position and velocity forward in time.
 * The error covariance P will grow during occlusion, naturally increasing
 * the uncertainty and making collision warnings more conservative.
 *
 * @param poses Vector of detected marker poses from ArucoDetector.
 */
void KalmanPredictor::update(const std::vector<MarkerPose>& poses) {
    if (poses.empty()) {
        // No markers detected — perform prediction-only step.
        // This handles temporary occlusion: the filter extrapolates
        // the trolley's trajectory based on the last known velocity.
        // The error covariance P will grow, reflecting increased
        // uncertainty during the occlusion period.
        if (initialized_) {
            kf_.predict();
        }
        return;
    }

    // Compute the centroid of all detected marker translation vectors.
    // Using the mean of multiple markers provides a more stable position
    // estimate than any single marker, as individual marker noise is
    // averaged out (standard error reduces by 1/sqrt(N)).
    cv::Point3f centroid(0.0f, 0.0f, 0.0f);
    for (const auto& mp : poses) {
        centroid.x += static_cast<float>(mp.tvec[0]);
        centroid.y += static_cast<float>(mp.tvec[1]);
        centroid.z += static_cast<float>(mp.tvec[2]);
    }
    float n = static_cast<float>(poses.size());
    centroid.x /= n;
    centroid.y /= n;
    centroid.z /= n;

    // Initialise the filter on the first valid measurement
    if (!initialized_) {
        init_(centroid);
        return;
    }

    // Standard Kalman filter predict-then-correct cycle:
    // 1. Predict: propagate state forward using the motion model
    kf_.predict();

    // 2. Correct: incorporate the new measurement to refine the estimate
    cv::Mat measurement = (cv::Mat_<float>(MEASUREMENT_DIM, 1)
        << centroid.x, centroid.y, centroid.z);
    kf_.correct(measurement);
}

// ---------------------------------------------------------------------------
// predict() — Look-ahead trajectory prediction
// ---------------------------------------------------------------------------

/**
 * @brief Predict the trolley's state at t + horizonSeconds from now.
 *
 * This method manually propagates the current state forward by N steps
 * (where N = round(horizonSeconds / dt_)) using only the transition
 * matrix, WITHOUT calling kf_.predict() which would corrupt the
 * internal filter state. The filter's state remains unchanged after
 * this call, preserving the correct state for future update() calls.
 *
 * @param horizonSeconds Look-ahead time in seconds (default: 0.5s).
 * @return PredictedState with predicted position, velocity, and uncertainty.
 */
PredictedState KalmanPredictor::predict(float horizonSeconds) const {
    PredictedState result;

    if (!initialized_) {
        // No data yet — return origin with maximum uncertainty
        result.position    = cv::Point3f(0.0f, 0.0f, 0.0f);
        result.velocity    = cv::Point3f(0.0f, 0.0f, 0.0f);
        result.uncertainty = 1e6f;  // Very large: no confidence
        return result;
    }

    // Number of time steps to propagate forward
    int N = static_cast<int>(std::round(horizonSeconds / dt_));
    if (N < 1) N = 1;

    // Start from the current post-update state
    cv::Mat state = kf_.statePost.clone();

    // Manually propagate forward N steps using the transition matrix.
    // We do NOT call kf_.predict() here because that would modify
    // the internal statePost and errorCovPost, corrupting future
    // update() calls. Instead, we perform the propagation on a copy
    // of the state vector.
    for (int i = 0; i < N; ++i) {
        state = kf_.transitionMatrix * state;
    }

    // Extract predicted position and velocity from the propagated state
    result.position = cv::Point3f(state.at<float>(0),
                                    state.at<float>(1),
                                    state.at<float>(2));
    result.velocity = cv::Point3f(state.at<float>(3),
                                    state.at<float>(4),
                                    state.at<float>(5));

    // Compute uncertainty as the trace of the position covariance block.
    // The trace of the 3x3 upper-left block of the error covariance
    // matrix represents the total spatial uncertainty (sum of variances
    // in x, y, z). Higher values indicate lower confidence in the
    // position estimate.
    const cv::Mat& P = kf_.errorCovPost;
    result.uncertainty = P.at<float>(0, 0)     // var(x)
                       + P.at<float>(1, 1)     // var(y)
                       + P.at<float>(2, 2);    // var(z)

    return result;
}

// ---------------------------------------------------------------------------
// isInitialized()
// ---------------------------------------------------------------------------

bool KalmanPredictor::isInitialized() const {
    return initialized_;
}
