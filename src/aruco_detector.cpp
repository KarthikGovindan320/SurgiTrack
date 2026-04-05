/*
 * aruco_detector.cpp
 * Author: Karthik Govindan V
 * Date: 2026-04-05
 *
 * Implements the ArucoDetector class. Handles camera calibration loading,
 * ArUco marker detection using DICT_6X6_250, and 6-DoF pose estimation
 * via solvePnP. The detector is tuned for hospital fluorescent lighting
 * conditions with increased adaptive threshold window sizes and sub-pixel
 * corner refinement for maximum pose accuracy.
 */

#include "aruco_detector.hpp"
#include <stdexcept>
#include <iostream>

// ---------------------------------------------------------------------------
// Distance threshold for close-range / far-range colour coding.
// Markers closer than this value (in metres) are coloured green to indicate
// the trolley is near a patient bed zone where extra caution is needed.
// Markers beyond this distance are coloured blue (general corridor zone).
// ---------------------------------------------------------------------------
constexpr double CLOSE_RANGE_THRESHOLD_METRES = 1.5;

// Axis length for the 3D coordinate frame drawn on each marker (metres)
constexpr float  DEBUG_AXIS_LENGTH_METRES     = 0.05f;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

/**
 * @brief Construct an ArucoDetector with camera calibration data.
 *
 * Loads the intrinsic camera matrix and distortion coefficients from a
 * YAML file, then configures the ArUco detector with DICT_6X6_250 and
 * parameters tuned for hospital fluorescent lighting.
 *
 * @param calibFile Path to the YAML file containing camera_matrix and dist_coeffs.
 * @param side      Physical side length of the printed markers in metres.
 */
ArucoDetector::ArucoDetector(const std::string& calibFile, float side)
    : markerSide_(side)
{
    // Load camera intrinsics from the calibration YAML file
    loadCalibration_(calibFile);

    // DICT_6X6_250 chosen: 36-bit codes with minimum Hamming distance
    // of 12 between any two valid codewords, giving robust detection
    // even under partial occlusion (up to 5 bit errors tolerated).
    // This dictionary supports 250 unique marker IDs, which is more
    // than sufficient for a multi-marker trolley tracking system.
    auto dict   = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    auto params = cv::aruco::DetectorParameters();

    // Increase adaptiveThreshWinSizeMax for hospital fluorescent
    // lighting which causes large uniform regions in the image.
    // The default value (23) is too small for high-intensity overhead
    // lights that wash out local contrast; 53 captures wider regions.
    params.adaptiveThreshWinSizeMax = 53;

    // Enable sub-pixel corner refinement for improved pose accuracy.
    // CORNER_REFINE_SUBPIX uses cv::cornerSubPix internally, which
    // iteratively refines corner positions to sub-pixel accuracy
    // (typically 0.1 to 0.3 px RMS), directly improving solvePnP results.
    params.cornerRefinementMethod   =
        cv::aruco::CORNER_REFINE_SUBPIX;

    // Construct the ArUco detector with the configured dictionary and parameters
    detector_ = cv::aruco::ArucoDetector(dict, params);
}

// ---------------------------------------------------------------------------
// detect() — Marker detection and pose estimation
// ---------------------------------------------------------------------------

/**
 * @brief Detect ArUco markers in the input frame and estimate their 6-DoF pose.
 *
 * The detection pipeline operates in four stages:
 * 1. Adaptive thresholding to find candidate quadrilaterals
 * 2. Contour-based corner detection with sub-pixel refinement
 * 3. Perspective-corrected bit decoding of the marker interior
 * 4. Dictionary lookup to verify and assign marker IDs
 *
 * After detection, solvePnP is called for each marker to estimate the
 * rotation (rvec) and translation (tvec) relative to the camera frame.
 *
 * @param frame Input BGR camera frame (not modified).
 * @return Vector of MarkerPose structs, one per detected marker.
 */
std::vector<MarkerPose> ArucoDetector::detect(const cv::Mat& frame) {
    std::vector<int>                     ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    // Step 1: Detect marker candidates and decode IDs.
    // The detectMarkers function performs adaptive thresholding,
    // contour extraction, perspective correction, bit decoding,
    // and dictionary matching in a single call.
    detector_.detectMarkers(frame, corners, ids, rejected);

    std::vector<MarkerPose> poses;
    if (ids.empty()) return poses;

    // Step 2: Estimate 6-DoF pose for each detected marker.
    // markerSide_ is the physical size of the printed marker in
    // metres; this determines the scale of tvec. Internally, this
    // function constructs 3D object points at ±side/2 in X and Y
    // (Z=0 in the marker plane) and calls cv::solvePnP with
    // SOLVEPNP_ITERATIVE (Levenberg-Marquardt optimisation) to
    // minimise reprojection error between projected 3D points
    // and the detected 2D corner pixels.
    int nMarkers = static_cast<int>(corners.size());
    std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    // Build the 3D object points for a single marker (four corners
    // at ±side/2 in XY plane, Z=0)
    float halfSide = markerSide_ / 2.0f;
    std::vector<cv::Point3f> objPoints = {
        {-halfSide,  halfSide, 0.0f},   // top-left
        { halfSide,  halfSide, 0.0f},   // top-right
        { halfSide, -halfSide, 0.0f},   // bottom-right
        {-halfSide, -halfSide, 0.0f}    // bottom-left
    };

    for (int i = 0; i < nMarkers; ++i) {
        cv::solvePnP(objPoints, corners[i], K_, D_,
                      rvecs[i], tvecs[i], false,
                      cv::SOLVEPNP_ITERATIVE);
    }

    // Step 3: Package results into MarkerPose structs
    for (size_t i = 0; i < ids.size(); ++i) {
        MarkerPose mp;
        mp.id      = ids[i];
        mp.rvec    = rvecs[i];
        mp.tvec    = tvecs[i];
        mp.corners = corners[i];
        // Detection confidence is set to 1.0 for the baseline detector.
        // In the enhanced version, this can be weighted by the ratio of
        // correctly decoded border bits from the detector parameters.
        mp.detectionConfidence = 1.0;
        poses.push_back(mp);
    }
    return poses;
}

// ---------------------------------------------------------------------------
// drawDebug() — Visual overlay for debugging and demonstration
// ---------------------------------------------------------------------------

/**
 * @brief Draw coordinate axes and marker IDs onto a colour frame.
 *
 * Renders a 3D axis tripod (RGB = XYZ) at each marker centre and overlays
 * the detected marker outlines with ID text. The axis colour scheme follows
 * clinical rationale:
 *   - GREEN axes for markers with tvec Z < 1.5 m (close range):
 *     indicates the trolley is near a patient bed zone where extra
 *     caution is needed.
 *   - BLUE axes for markers with tvec Z >= 1.5 m (far range):
 *     indicates general corridor distance, lower collision risk.
 *
 * @param frame Mutable BGR frame to draw on.
 * @param poses Vector of detected marker poses.
 */
void ArucoDetector::drawDebug(cv::Mat& frame,
                               const std::vector<MarkerPose>& poses) const {
    for (const auto& mp : poses) {
        // Draw the detected marker square outline and ID text
        // using OpenCV's built-in ArUco drawing utility
        std::vector<std::vector<cv::Point2f>> singleCorner = {mp.corners};
        std::vector<int> singleId = {mp.id};
        cv::aruco::drawDetectedMarkers(frame, singleCorner, singleId);

        // Draw 3D coordinate axes at the marker centre.
        // Axis length is 0.05 m (5 cm) for clear visibility without
        // cluttering the display.
        cv::drawFrameAxes(frame, K_, D_, mp.rvec, mp.tvec,
                          DEBUG_AXIS_LENGTH_METRES);

        // Determine colour based on Z-distance (clinical proximity logic):
        // Close-range (< 1.5 m) = green = near patient bed zone
        // Far-range (>= 1.5 m) = blue = general corridor
        double zDist = mp.tvec[2];
        cv::Scalar colour;
        if (zDist < CLOSE_RANGE_THRESHOLD_METRES) {
            colour = cv::Scalar(0, 255, 0);   // Green: close range, caution needed
        } else {
            colour = cv::Scalar(255, 0, 0);   // Blue: far range, lower risk
        }

        // Draw a filled circle at the projected marker centre for visibility
        cv::Point2f centre(0, 0);
        for (const auto& c : mp.corners) {
            centre.x += c.x;
            centre.y += c.y;
        }
        centre.x /= 4.0f;
        centre.y /= 4.0f;

        cv::circle(frame, cv::Point(static_cast<int>(centre.x),
                                     static_cast<int>(centre.y)),
                   8, colour, -1);

        // Display distance text near the marker
        std::string distText = cv::format("%.2fm", zDist);
        cv::putText(frame, distText,
                    cv::Point(static_cast<int>(centre.x) + 15,
                              static_cast<int>(centre.y) - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, colour, 2);
    }
}

// ---------------------------------------------------------------------------
// Accessor methods
// ---------------------------------------------------------------------------

cv::Mat ArucoDetector::cameraMatrix() const { return K_.clone(); }
cv::Mat ArucoDetector::distCoeffs()   const { return D_.clone(); }

// ---------------------------------------------------------------------------
// loadCalibration_() — Load camera intrinsics from YAML
// ---------------------------------------------------------------------------

/**
 * @brief Load camera matrix K and distortion coefficients D from a YAML file.
 *
 * The calibration file must contain two entries:
 *   - "camera_matrix": 3x3 intrinsic matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
 *   - "dist_coeffs": 1x5 distortion vector [k1, k2, p1, p2, k3]
 *
 * @param path Absolute or relative path to the calibration YAML file.
 * @throws std::runtime_error if the file cannot be opened or parsed.
 */
void ArucoDetector::loadCalibration_(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("ArucoDetector: cannot open calibration file: " + path);
    }

    fs["camera_matrix"] >> K_;
    fs["dist_coeffs"]   >> D_;

    if (K_.empty() || D_.empty()) {
        throw std::runtime_error("ArucoDetector: calibration file missing camera_matrix or dist_coeffs");
    }

    std::cout << "[ArucoDetector] Loaded calibration from " << path << std::endl;
    std::cout << "  Camera matrix: " << K_.size() << std::endl;
    std::cout << "  Distortion coeffs: " << D_.size() << std::endl;

    fs.release();
}
