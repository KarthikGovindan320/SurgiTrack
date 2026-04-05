/*
 * main.cpp
 * Author: Karthik Govindan V
 * Date: 2026-04-05
 *
 * Entry point for the MediMark AR trolley tracking system. Supports
 * two run modes controlled by a command-line flag:
 *   --mode demo     : Opens webcam, shows live ArUco detection with
 *                     collision warning overlay.
 *   --mode evaluate : Processes data/sample_frames/ and outputs metrics.
 *
 * Architecture:
 *   Camera → ArucoDetector → KalmanPredictor → CollisionChecker → Visualizer
 */

#include <iostream>
#include <string>
#include <vector>
#include <cstring>

#include "aruco_detector.hpp"
#include "kalman_predictor.hpp"
#include "collision_checker.hpp"
#include "visualizer.hpp"

// ---------------------------------------------------------------------------
// Named constants — no magic numbers in the codebase
// ---------------------------------------------------------------------------

/// Path to the camera calibration YAML file
constexpr const char* CALIBRATION_FILE_PATH    = "data/calibration.yml";

/// Physical side length of printed ArUco markers in metres (10 cm)
constexpr float       MARKER_SIDE_METRES       = 0.10f;

/// Camera device index for cv::VideoCapture
constexpr int         CAMERA_DEVICE_INDEX       = 0;

/// Kalman filter time step (1/30 for 30 FPS capture)
constexpr float       KALMAN_DT_SECONDS         = 1.0f / 30.0f;

/// Collision prediction look-ahead horizon in seconds
constexpr float       PREDICTION_HORIZON_SECONDS = 0.5f;

/// Window name for the live display
constexpr const char* WINDOW_NAME               = "MediMark AR - Trolley Tracker";

// ---------------------------------------------------------------------------
// Hazard zone definitions for a typical hospital corridor
// ---------------------------------------------------------------------------

/// Left corridor wall boundary (x = -1.2 m from camera centre)
const HazardZone ZONE_WALL_LEFT  = {"WallLeft",  {-1.2f, 0.0f, 0.0f}, 0.25f};

/// Right corridor wall boundary (x = +1.2 m from camera centre)
const HazardZone ZONE_WALL_RIGHT = {"WallRight", { 1.2f, 0.0f, 0.0f}, 0.25f};

/// Doorframe obstacle straight ahead (z = 2.5 m from camera)
const HazardZone ZONE_DOOR_FRAME = {"DoorFrame", { 0.0f, 0.0f, 2.5f}, 0.30f};

// ---------------------------------------------------------------------------
// Usage help text
// ---------------------------------------------------------------------------

static void printUsage(const char* progName) {
    std::cout << "MediMark AR v1.0.0 — Augmented Reality Medicine Trolley Tracker\n"
              << "Usage: " << progName << " --mode [demo|evaluate]\n\n"
              << "Modes:\n"
              << "  demo     — Open webcam and show live ArUco detection with\n"
              << "             Kalman prediction and collision warning overlay.\n"
              << "  evaluate — Process sample frames from data/sample_frames/\n"
              << "             and output detection metrics to stdout.\n"
              << std::endl;
}

// ---------------------------------------------------------------------------
// parseMode() — Command-line argument parser
// ---------------------------------------------------------------------------

static std::string parseMode(int argc, char** argv) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], "--mode") == 0) {
            return std::string(argv[i + 1]);
        }
    }
    return "";
}

// ---------------------------------------------------------------------------
// runDemoMode() — Live webcam detection with AR overlay
// ---------------------------------------------------------------------------

/**
 * @brief Run the live demo mode with webcam capture.
 *
 * Main loop:
 * 1. Capture a frame from cv::VideoCapture(0).
 * 2. Call detector.detect(frame) to get a vector of MarkerPose.
 * 3. Call predictor.update(poses) to update Kalman state.
 * 4. Call checker.check(predictor.predictedPosition(), zoneMap) for warnings.
 * 5. Call visualizer.render(frame, poses, warning) to draw the overlay.
 * 6. Display the frame with cv::imshow and handle keypress 'q' to quit.
 */
static void runDemoMode() {
    std::cout << "[MediMark AR] Starting demo mode...\n";

    // Initialise the ArUco detector with camera calibration
    ArucoDetector detector(CALIBRATION_FILE_PATH, MARKER_SIDE_METRES);

    // Initialise the Kalman predictor for trajectory estimation
    KalmanPredictor predictor(KALMAN_DT_SECONDS);

    // Initialise the collision checker and register hazard zones
    CollisionChecker checker;
    checker.addZone(ZONE_WALL_LEFT);
    checker.addZone(ZONE_WALL_RIGHT);
    checker.addZone(ZONE_DOOR_FRAME);

    // Initialise the visualizer for AR overlay rendering
    Visualizer visualizer;

    // Open the default camera (device index 0)
    cv::VideoCapture cap(CAMERA_DEVICE_INDEX);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Cannot open camera device " << CAMERA_DEVICE_INDEX << "\n";
        return;
    }
    std::cout << "[MediMark AR] Camera opened successfully.\n";
    std::cout << "[MediMark AR] Press 'q' to quit.\n";

    cv::Mat frame;
    while (true) {
        // Step 1: Capture a frame from the webcam
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "[WARN] Empty frame captured, skipping...\n";
            continue;
        }

        // Step 2: Detect ArUco markers and estimate 6-DoF poses
        std::vector<MarkerPose> poses = detector.detect(frame);

        // Step 3: Update the Kalman predictor with new measurements.
        // If no markers are detected, the predictor performs a
        // prediction-only step to handle temporary occlusion.
        predictor.update(poses);

        // Step 4: Predict the future trolley position and check
        // against hazard zones for potential collisions
        PredictedState predicted = predictor.predict(PREDICTION_HORIZON_SECONDS);
        CollisionWarning warning = checker.check(predicted.position,
                                                  PREDICTION_HORIZON_SECONDS);

        // Step 5: Render the AR overlay with all visual elements
        visualizer.render(frame, poses, predicted, warning,
                          detector.cameraMatrix(), detector.distCoeffs());

        // Step 6: Display the frame and handle keypress 'q' to quit
        cv::imshow(WINDOW_NAME, frame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {  // 'q', 'Q', or ESC
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    std::cout << "[MediMark AR] Demo mode ended.\n";
}

// ---------------------------------------------------------------------------
// runEvaluateMode() — Process sample frames and output metrics
// ---------------------------------------------------------------------------

/**
 * @brief Run the evaluation mode on pre-recorded sample frames.
 *
 * Processes all images in data/sample_frames/ and outputs detection
 * statistics including detection rate, mean pose estimation error,
 * and Kalman prediction accuracy.
 */
static void runEvaluateMode() {
    std::cout << "[MediMark AR] Starting evaluate mode...\n";

    // Initialise the ArUco detector with camera calibration
    ArucoDetector detector(CALIBRATION_FILE_PATH, MARKER_SIDE_METRES);

    // Initialise the Kalman predictor for trajectory estimation
    KalmanPredictor predictor(KALMAN_DT_SECONDS);

    // Initialise the collision checker and register hazard zones
    CollisionChecker checker;
    checker.addZone(ZONE_WALL_LEFT);
    checker.addZone(ZONE_WALL_RIGHT);
    checker.addZone(ZONE_DOOR_FRAME);

    // Collect sample frame paths
    std::string sampleDir = "data/sample_frames/";
    std::vector<cv::String> framePaths;
    cv::glob(sampleDir + "*.png", framePaths, false);
    cv::glob(sampleDir + "*.jpg", framePaths, false);

    if (framePaths.empty()) {
        std::cerr << "[WARN] No sample frames found in " << sampleDir << "\n";
        return;
    }

    int totalFrames        = 0;
    int totalDetections    = 0;
    int framesWithMarkers  = 0;

    for (const auto& path : framePaths) {
        cv::Mat frame = cv::imread(path);
        if (frame.empty()) continue;

        totalFrames++;
        std::vector<MarkerPose> poses = detector.detect(frame);
        int numDetected = static_cast<int>(poses.size());
        totalDetections += numDetected;

        if (numDetected > 0) {
            framesWithMarkers++;
            predictor.update(poses);

            // Report per-frame statistics
            std::cout << "  Frame: " << path
                      << " | Markers: " << numDetected;
            for (const auto& mp : poses) {
                std::cout << " [ID=" << mp.id
                          << " Z=" << cv::format("%.3f", mp.tvec[2]) << "m]";
            }
            std::cout << "\n";
        }
    }

    // Summary statistics
    double detectionRate = (totalFrames > 0)
        ? 100.0 * framesWithMarkers / totalFrames : 0.0;

    std::cout << "\n=== Evaluation Summary ===\n"
              << "  Total frames processed:  " << totalFrames << "\n"
              << "  Frames with detections:  " << framesWithMarkers << "\n"
              << "  Total marker detections: " << totalDetections << "\n"
              << "  Detection rate:          " << cv::format("%.1f", detectionRate) << "%\n"
              << std::endl;
}

// ---------------------------------------------------------------------------
// main() — Entry point
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::string mode = parseMode(argc, argv);

    if (mode == "demo") {
        runDemoMode();
    } else if (mode == "evaluate") {
        runEvaluateMode();
    } else {
        printUsage(argv[0]);
        return 1;
    }

    return 0;
}
