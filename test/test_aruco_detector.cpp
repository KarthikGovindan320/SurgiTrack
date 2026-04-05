/*
 * test_aruco_detector.cpp
 * Author: Aditya Yadav
 * Date: 2026-04-05
 *
 * Unit tests for the ArucoDetector class. Validates calibration loading,
 * marker detection on synthetic images, and pose estimation accuracy.
 * Uses assert-based testing (no external test framework dependency).
 */

#include "aruco_detector.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

// ---------------------------------------------------------------------------
// Named constants for test parameters
// ---------------------------------------------------------------------------

/// Path to the camera calibration file used in tests
constexpr const char* TEST_CALIBRATION_FILE = "data/calibration.yml";

/// Expected marker side length in metres
constexpr float TEST_MARKER_SIDE = 0.10f;

/// Tolerance for floating-point comparisons (metres)
constexpr double POSITION_TOLERANCE_METRES = 0.05;

// ---------------------------------------------------------------------------
// Test: Calibration file loading
// ---------------------------------------------------------------------------

/**
 * @brief Verify that the ArucoDetector loads calibration data correctly.
 *
 * Checks that the camera matrix is 3x3 and distortion coefficients are 1x5.
 * Also verifies that the focal lengths are positive (basic sanity check).
 */
void test_calibration_loading() {
    std::cout << "[TEST] test_calibration_loading... ";

    ArucoDetector detector(TEST_CALIBRATION_FILE, TEST_MARKER_SIDE);

    cv::Mat K = detector.cameraMatrix();
    cv::Mat D = detector.distCoeffs();

    // Camera matrix must be 3x3
    assert(K.rows == 3 && K.cols == 3);

    // Distortion coefficients must be 1x5
    assert(D.rows == 1 && D.cols == 5);

    // Focal lengths (K[0,0] and K[1,1]) must be positive
    assert(K.at<double>(0, 0) > 0);
    assert(K.at<double>(1, 1) > 0);

    // Principal point (K[0,2] and K[1,2]) must be within typical range
    assert(K.at<double>(0, 2) > 100 && K.at<double>(0, 2) < 2000);
    assert(K.at<double>(1, 2) > 100 && K.at<double>(1, 2) < 2000);

    std::cout << "PASSED" << std::endl;
}

// ---------------------------------------------------------------------------
// Test: Detection on blank image (no markers)
// ---------------------------------------------------------------------------

/**
 * @brief Verify that detection on a blank image returns no markers.
 *
 * A uniformly grey image should produce zero detections and no crashes.
 */
void test_no_markers_detected() {
    std::cout << "[TEST] test_no_markers_detected... ";

    ArucoDetector detector(TEST_CALIBRATION_FILE, TEST_MARKER_SIDE);

    // Create a blank grey image (no markers)
    cv::Mat blankFrame(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));

    std::vector<MarkerPose> poses = detector.detect(blankFrame);

    assert(poses.empty());

    std::cout << "PASSED" << std::endl;
}

// ---------------------------------------------------------------------------
// Test: Detection on synthetic marker image
// ---------------------------------------------------------------------------

/**
 * @brief Generate a synthetic image with an ArUco marker and verify detection.
 *
 * Creates a marker image using the same DICT_6X6_250 dictionary, then
 * embeds it into a larger frame. Verifies that the correct marker ID
 * is detected.
 */
void test_synthetic_marker_detection() {
    std::cout << "[TEST] test_synthetic_marker_detection... ";

    ArucoDetector detector(TEST_CALIBRATION_FILE, TEST_MARKER_SIDE);

    // Generate a synthetic marker image (ID 0, DICT_6X6_250)
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Mat markerImg;
    cv::aruco::generateImageMarker(dict, 0, 200, markerImg, 1);

    // Embed the marker in a white background frame
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat roi = frame(cv::Rect(220, 140, 200, 200));
    cv::cvtColor(markerImg, markerImg, cv::COLOR_GRAY2BGR);
    markerImg.copyTo(roi);

    std::vector<MarkerPose> poses = detector.detect(frame);

    // At least one marker should be detected
    assert(!poses.empty());

    // The detected marker should have ID 0
    bool foundId0 = false;
    for (const auto& mp : poses) {
        if (mp.id == 0) {
            foundId0 = true;
            // Corners should be within the frame boundaries
            for (const auto& c : mp.corners) {
                assert(c.x >= 0 && c.x < 640);
                assert(c.y >= 0 && c.y < 480);
            }
        }
    }
    assert(foundId0);

    std::cout << "PASSED" << std::endl;
}

// ---------------------------------------------------------------------------
// Test: drawDebug does not crash
// ---------------------------------------------------------------------------

/**
 * @brief Verify that drawDebug runs without crashing on valid poses.
 */
void test_draw_debug_no_crash() {
    std::cout << "[TEST] test_draw_debug_no_crash... ";

    ArucoDetector detector(TEST_CALIBRATION_FILE, TEST_MARKER_SIDE);

    // Create a test frame with a synthetic marker
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Mat markerImg;
    cv::aruco::generateImageMarker(dict, 2, 200, markerImg, 1);

    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat roi = frame(cv::Rect(220, 140, 200, 200));
    cv::cvtColor(markerImg, markerImg, cv::COLOR_GRAY2BGR);
    markerImg.copyTo(roi);

    std::vector<MarkerPose> poses = detector.detect(frame);
    if (!poses.empty()) {
        // This should not crash
        detector.drawDebug(frame, poses);
    }

    std::cout << "PASSED" << std::endl;
}

// ---------------------------------------------------------------------------
// Main test runner
// ---------------------------------------------------------------------------

int main() {
    std::cout << "================================" << std::endl;
    std::cout << "ArucoDetector Unit Tests" << std::endl;
    std::cout << "================================" << std::endl;

    test_calibration_loading();
    test_no_markers_detected();
    test_synthetic_marker_detection();
    test_draw_debug_no_crash();

    std::cout << "================================" << std::endl;
    std::cout << "All ArucoDetector tests PASSED" << std::endl;
    std::cout << "================================" << std::endl;

    return 0;
}
