/*
 * test_kalman_predictor.cpp
 * Author: Mukul Dahiya
 * Date: 2026-04-05
 *
 * Unit tests for the KalmanPredictor class. Validates filter initialisation,
 * constant-velocity prediction, occlusion handling, and uncertainty growth.
 * Uses assert-based testing (no external test framework dependency).
 */

#include "kalman_predictor.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

// ---------------------------------------------------------------------------
// Named constants for test parameters
// ---------------------------------------------------------------------------

/// Time step matching 30 FPS capture rate
constexpr float TEST_DT = 1.0f / 30.0f;

/// Tolerance for position comparisons (metres)
constexpr float POSITION_TOLERANCE = 0.1f;

/// Tolerance for velocity comparisons (m/s)
constexpr float VELOCITY_TOLERANCE = 0.5f;

// ---------------------------------------------------------------------------
// Helper: Create a MarkerPose at a given 3D position
// ---------------------------------------------------------------------------

static MarkerPose makeMarkerPose(int id, double x, double y, double z) {
    MarkerPose mp;
    mp.id = id;
    mp.rvec = cv::Vec3d(0, 0, 0);
    mp.tvec = cv::Vec3d(x, y, z);
    mp.corners = {{0, 0}, {100, 0}, {100, 100}, {0, 100}};
    mp.detectionConfidence = 1.0;
    return mp;
}

// ---------------------------------------------------------------------------
// Test: Uninitialised predictor returns safe defaults
// ---------------------------------------------------------------------------

/**
 * @brief Verify that predict() before any update() returns origin with
 *        maximum uncertainty.
 */
void test_uninitialised_predict() {
    std::cout << "[TEST] test_uninitialised_predict... ";

    KalmanPredictor predictor(TEST_DT);

    assert(!predictor.isInitialized());

    PredictedState state = predictor.predict();
    assert(state.position.x == 0.0f);
    assert(state.position.y == 0.0f);
    assert(state.position.z == 0.0f);
    assert(state.uncertainty > 1e5f);  // Very large uncertainty

    std::cout << "PASSED" << std::endl;
}

// ---------------------------------------------------------------------------
// Test: First update initialises the filter
// ---------------------------------------------------------------------------

/**
 * @brief Verify that the first update() call initialises the filter
 *        at the measured position.
 */
void test_first_update_initialises() {
    std::cout << "[TEST] test_first_update_initialises... ";

    KalmanPredictor predictor(TEST_DT);

    std::vector<MarkerPose> poses = {makeMarkerPose(0, 0.5, 0.0, 1.0)};
    predictor.update(poses);

    assert(predictor.isInitialized());

    PredictedState state = predictor.predict(0.0f);
    // Position should be near the first measurement
    assert(std::abs(state.position.x - 0.5f) < POSITION_TOLERANCE);
    assert(std::abs(state.position.z - 1.0f) < POSITION_TOLERANCE);

    std::cout << "PASSED" << std::endl;
}

// ---------------------------------------------------------------------------
// Test: Constant-velocity prediction
// ---------------------------------------------------------------------------

/**
 * @brief Feed constant-velocity measurements and verify the prediction
 *        estimates the correct future position.
 *
 * Simulates a trolley moving at 0.5 m/s along the Z axis.
 */
void test_constant_velocity_prediction() {
    std::cout << "[TEST] test_constant_velocity_prediction... ";

    KalmanPredictor predictor(TEST_DT);

    // Simulate 30 frames of constant velocity motion (0.5 m/s along Z)
    constexpr float SPEED = 0.5f;      // m/s
    constexpr int   N_FRAMES = 30;     // 1 second of data

    for (int i = 0; i < N_FRAMES; ++i) {
        float z = 1.0f + SPEED * i * TEST_DT;
        std::vector<MarkerPose> poses = {makeMarkerPose(0, 0.0, 0.0, z)};
        predictor.update(poses);
    }

    // Predict 0.5 seconds ahead
    PredictedState state = predictor.predict(0.5f);

    // Expected position: current Z + speed * 0.5
    float currentZ = 1.0f + SPEED * (N_FRAMES - 1) * TEST_DT;
    float expectedZ = currentZ + SPEED * 0.5f;

    // The Kalman filter should estimate a position close to expected
    assert(std::abs(state.position.z - expectedZ) < POSITION_TOLERANCE);

    // Velocity Z should be close to 0.5 m/s
    assert(std::abs(state.velocity.z - SPEED) < VELOCITY_TOLERANCE);

    std::cout << "PASSED" << std::endl;
}

// ---------------------------------------------------------------------------
// Test: Multi-marker centroid computation
// ---------------------------------------------------------------------------

/**
 * @brief Verify that multiple markers produce a centroid position.
 */
void test_multi_marker_centroid() {
    std::cout << "[TEST] test_multi_marker_centroid... ";

    KalmanPredictor predictor(TEST_DT);

    // Two markers at different positions
    std::vector<MarkerPose> poses = {
        makeMarkerPose(0, -0.5, 0.0, 1.0),
        makeMarkerPose(1,  0.5, 0.0, 1.0)
    };
    predictor.update(poses);

    PredictedState state = predictor.predict(0.0f);

    // Centroid X should be approximately 0.0 (average of -0.5 and 0.5)
    assert(std::abs(state.position.x) < POSITION_TOLERANCE);
    // Centroid Z should be approximately 1.0
    assert(std::abs(state.position.z - 1.0f) < POSITION_TOLERANCE);

    std::cout << "PASSED" << std::endl;
}

// ---------------------------------------------------------------------------
// Test: Occlusion handling (predict-only step)
// ---------------------------------------------------------------------------

/**
 * @brief Verify that empty pose vectors trigger prediction-only steps
 *        without crashing, and uncertainty increases.
 */
void test_occlusion_handling() {
    std::cout << "[TEST] test_occlusion_handling... ";

    KalmanPredictor predictor(TEST_DT);

    // Initialise with a measurement
    std::vector<MarkerPose> poses = {makeMarkerPose(0, 0.0, 0.0, 1.0)};
    predictor.update(poses);

    float uncertaintyBefore = predictor.predict(0.0f).uncertainty;

    // Simulate 10 frames of occlusion (no markers detected)
    for (int i = 0; i < 10; ++i) {
        predictor.update({});  // Empty poses
    }

    float uncertaintyAfter = predictor.predict(0.0f).uncertainty;

    // Uncertainty should increase during occlusion
    assert(uncertaintyAfter >= uncertaintyBefore);

    // Filter should still be initialised
    assert(predictor.isInitialized());

    std::cout << "PASSED" << std::endl;
}

// ---------------------------------------------------------------------------
// Main test runner
// ---------------------------------------------------------------------------

int main() {
    std::cout << "================================" << std::endl;
    std::cout << "KalmanPredictor Unit Tests" << std::endl;
    std::cout << "================================" << std::endl;

    test_uninitialised_predict();
    test_first_update_initialises();
    test_constant_velocity_prediction();
    test_multi_marker_centroid();
    test_occlusion_handling();

    std::cout << "================================" << std::endl;
    std::cout << "All KalmanPredictor tests PASSED" << std::endl;
    std::cout << "================================" << std::endl;

    return 0;
}
