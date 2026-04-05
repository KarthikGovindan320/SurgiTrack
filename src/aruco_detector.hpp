/*
 * aruco_detector.hpp
 * Author: Karthik Govindan V
 * Date: 2026-04-05
 *
 * Declares the ArucoDetector class responsible for detecting ArUco
 * fiducial markers in camera frames and estimating their 6-DoF pose
 * using OpenCV's ArUco module and solvePnP-based pose estimation.
 */

#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <vector>
#include <string>

/**
 * @brief Holds the detected pose and metadata of a single ArUco marker.
 */
struct MarkerPose {
    int id;                                ///< Unique marker ID from the dictionary
    cv::Vec3d rvec;                        ///< Rodrigues rotation vector
    cv::Vec3d tvec;                        ///< Translation vector (metres)
    std::vector<cv::Point2f> corners;      ///< Four corner points in image coordinates
    double detectionConfidence;            ///< Ratio of border bits correctly decoded
};

/**
 * @brief Detects ArUco markers and estimates their 6-DoF pose relative
 *        to a calibrated camera.
 */
class ArucoDetector {
public:
    /**
     * @brief Construct an ArucoDetector with camera calibration data.
     * @param calibrationFile Path to the YAML file containing camera_matrix and dist_coeffs.
     * @param markerSideMetres Physical side length of the printed markers in metres.
     */
    explicit ArucoDetector(const std::string& calibrationFile,
                           float markerSideMetres = 0.10f);

    /**
     * @brief Detect markers and estimate their 6-DoF pose.
     * @param frame Input BGR camera frame.
     * @return Vector of MarkerPose structs for each detected marker.
     */
    std::vector<MarkerPose> detect(const cv::Mat& frame);

    /**
     * @brief Draw axes and IDs onto a colour frame for visualisation.
     * @param frame Mutable BGR frame to draw on.
     * @param poses Vector of detected marker poses.
     */
    void drawDebug(cv::Mat& frame,
                   const std::vector<MarkerPose>& poses) const;

    /** @brief Return the loaded camera intrinsic matrix. */
    cv::Mat cameraMatrix() const;

    /** @brief Return the loaded distortion coefficients. */
    cv::Mat distCoeffs()   const;

private:
    cv::Mat             K_, D_;           ///< Camera intrinsics and distortion coefficients
    float               markerSide_;      ///< Physical marker side length in metres
    cv::aruco::ArucoDetector detector_;   ///< OpenCV ArUco detector instance
    void loadCalibration_(const std::string& path);
};
