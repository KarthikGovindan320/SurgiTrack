#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <limits>

#include "instrument_detector.hpp"
#include "instrument_tracker.hpp"
#include "sterile_field_monitor.hpp"
#include "or_visualizer.hpp"

constexpr const char* CALIBRATION_FILE_PATH    = "data/calibration.yml";

constexpr float       MARKER_SIDE_METRES       = 0.10f;

constexpr int         CAMERA_DEVICE_INDEX       = 0;

constexpr float       KALMAN_DT_SECONDS         = 1.0f / 30.0f;

constexpr float       PREDICTION_HORIZON_SECONDS = 0.5f;

constexpr const char* WINDOW_NAME               = "SurgiTrack - Instrument Monitor";

const SterileZone ZONE_WALL_LEFT  = {"WallLeft",  {-1.2f, 0.0f, 0.0f}, 0.25f};

const SterileZone ZONE_WALL_RIGHT = {"WallRight", { 1.2f, 0.0f, 0.0f}, 0.25f};

const SterileZone ZONE_DOOR_FRAME = {"DoorFrame", { 0.0f, 0.0f, 2.5f}, 0.30f};

static void printUsage(const char* progName) {
    std::cout << "SurgiTrack v1.0.0 — Augmented Reality Medicine Instrument Monitor\n"
              << "Usage: " << progName << " --mode [demo|evaluate]\n\n"
              << "Modes:\n"
              << "  demo     — Open webcam and show live ArUco detection with\n"
              << "             Kalman prediction and sterile field breach warning overlay.\n"
              << "  evaluate — Process sample frames from data/sample_frames/\n"
              << "             and output detection metrics to stdout.\n"
              << std::endl;
}

static std::string parseMode(int argc, char** argv) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], "--mode") == 0) {
            return std::string(argv[i + 1]);
        }
    }
    return "";
}

static void runDemoMode() {
    std::cout << "[SurgiTrack] Starting demo mode...\n";

    InstrumentDetector detector(CALIBRATION_FILE_PATH, MARKER_SIDE_METRES);

    InstrumentTracker predictor(KALMAN_DT_SECONDS);

    SterileFieldMonitor checker;
    checker.addSterileZone(ZONE_WALL_LEFT);
    checker.addSterileZone(ZONE_WALL_RIGHT);
    checker.addSterileZone(ZONE_DOOR_FRAME);

    ORVisualizer visualizer;

    cv::VideoCapture cap(CAMERA_DEVICE_INDEX);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] Cannot open camera device " << CAMERA_DEVICE_INDEX << "\n";
        return;
    }
    std::cout << "[SurgiTrack] Camera opened successfully.\n";
    std::cout << "[SurgiTrack] Press 'q' to quit.\n";

    cv::Mat frame;
    int emptyFrameCount = 0;
    constexpr int MAX_EMPTY_FRAMES = 30;  // ~1 second at 30 FPS
    while (true) {

        cap >> frame;
        if (frame.empty()) {
            std::cerr << "[WARN] Empty frame captured, skipping...\n";
            if (++emptyFrameCount >= MAX_EMPTY_FRAMES) {
                std::cerr << "[ERROR] Camera appears disconnected after "
                          << MAX_EMPTY_FRAMES << " consecutive empty frames. Exiting.\n";
                break;
            }
            continue;
        }
        emptyFrameCount = 0;

        std::vector<InstrumentPose> poses = detector.detect(frame);

        predictor.update(poses);

        // Per-instrument breach check — alert on the soonest predicted breach.
        FieldBreachAlert warning;
        warning.active              = false;
        warning.timeToImpactSeconds = std::numeric_limits<float>::max();
        warning.distanceMetres      = std::numeric_limits<float>::max();

        for (int id : predictor.trackedIds()) {
            TrackedInstrumentState cur  = predictor.predict(id, 0.0f);
            TrackedInstrumentState pred = predictor.predict(id, PREDICTION_HORIZON_SECONDS);
            FieldBreachAlert alert = checker.check(cur.position, cur.velocity,
                                                   pred.position, PREDICTION_HORIZON_SECONDS);
            if (alert.active && alert.timeToImpactSeconds < warning.timeToImpactSeconds) {
                warning = alert;
            }
        }

        // Centroid state used for the AR velocity arrow overlay.
        TrackedInstrumentState centroid = predictor.predictCentroid(PREDICTION_HORIZON_SECONDS);

        visualizer.render(frame, poses, centroid, warning,
                          detector.cameraMatrix(), detector.distCoeffs());

        cv::imshow(WINDOW_NAME, frame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {  
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    std::cout << "[SurgiTrack] Demo mode ended.\n";
}

static void runEvaluateMode() {
    std::cout << "[SurgiTrack] Starting evaluate mode...\n";

    InstrumentDetector detector(CALIBRATION_FILE_PATH, MARKER_SIDE_METRES);

    InstrumentTracker predictor(KALMAN_DT_SECONDS);

    SterileFieldMonitor checker;
    checker.addSterileZone(ZONE_WALL_LEFT);
    checker.addSterileZone(ZONE_WALL_RIGHT);
    checker.addSterileZone(ZONE_DOOR_FRAME);

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
        std::vector<InstrumentPose> poses = detector.detect(frame);
        int numDetected = static_cast<int>(poses.size());
        totalDetections += numDetected;

        if (numDetected > 0) {
            framesWithMarkers++;
            predictor.update(poses);

            std::cout << "  Frame: " << path
                      << " | Markers: " << numDetected;
            for (const auto& mp : poses) {
                std::cout << " [ID=" << mp.id
                          << " Z=" << cv::format("%.3f", mp.tvec[2]) << "m]";
            }
            std::cout << "\n";
        }
    }

    double detectionRate = (totalFrames > 0)
        ? 100.0 * framesWithMarkers / totalFrames : 0.0;

    std::cout << "\n=== Evaluation Summary ===\n"
              << "  Total frames processed:  " << totalFrames << "\n"
              << "  Frames with detections:  " << framesWithMarkers << "\n"
              << "  Total marker detections: " << totalDetections << "\n"
              << "  Detection rate:          " << cv::format("%.1f", detectionRate) << "%\n"
              << std::endl;
}

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