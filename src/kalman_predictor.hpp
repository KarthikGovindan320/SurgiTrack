/*
 * kalman_predictor.hpp
 * Author: Satyam Kumar
 * Date: 2026-04-05
 *
 * Declares the KalmanPredictor class that encapsulates a 6-state linear
 * Kalman filter for predicting the 3D trajectory of a medicine trolley.
 * The state vector is [x, y, z, vx, vy, vz] representing position and
 * velocity in the camera coordinate frame.
 */

#pragma once
#include <opencv2/video/tracking.hpp>
#include <vector>
#include "aruco_detector.hpp"

/**
 * @brief Holds the predicted 3D state of the trolley.
 */
struct PredictedState {
    cv::Point3f position;    ///< Predicted position in metres
    cv::Point3f velocity;    ///< Predicted velocity in m/s
    float       uncertainty; ///< Trace of position covariance block
};

/**
 * @brief Maintains a constant-velocity Kalman filter to predict trolley
 *        trajectory from ArUco marker pose measurements.
 */
class KalmanPredictor {
public:
    /**
     * @brief Construct a KalmanPredictor with timing and noise parameters.
     * @param dt Time step between frames in seconds (default: 1/30 for 30 FPS).
     * @param processNoise Process noise covariance scalar (model trust).
     * @param measurementNoise Measurement noise covariance scalar (sensor trust).
     */
    explicit KalmanPredictor(float dt = 1.0f/30.0f,
                             float processNoise   = 1e-4f,
                             float measurementNoise = 5e-3f);

    /**
     * @brief Feed a new set of detected marker poses; updates the filter.
     * @param poses Vector of detected marker poses from ArucoDetector.
     */
    void update(const std::vector<MarkerPose>& poses);

    /**
     * @brief Predict state at t+horizonSeconds from now.
     * @param horizonSeconds Look-ahead time in seconds (default: 0.5s).
     * @return PredictedState with position, velocity, and uncertainty.
     */
    PredictedState predict(float horizonSeconds = 0.5f) const;

    /** @brief Check if the Kalman filter has been initialized with a measurement. */
    bool isInitialized() const;

private:
    cv::KalmanFilter kf_;           ///< OpenCV Kalman filter instance
    bool             initialized_ = false;  ///< Whether first measurement received
    float            dt_;           ///< Time step in seconds
    float            processNoise_;      ///< Process noise parameter
    float            measurementNoise_;  ///< Measurement noise parameter
    void init_(const cv::Point3f& initialPos);
};
