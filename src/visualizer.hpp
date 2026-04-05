/*
 * visualizer.hpp
 * Author: Mukul Dahiya
 * Date: 2026-04-05
 *
 * Declares the Visualizer class responsible for rendering all AR overlay
 * elements onto the camera frame, including marker highlights, 3D axes,
 * velocity vectors, and collision warning indicators.
 */

#pragma once
#include <opencv2/opencv.hpp>
#include "aruco_detector.hpp"
#include "kalman_predictor.hpp"
#include "collision_checker.hpp"

/**
 * @brief Renders AR overlay elements onto camera frames for the
 *        MediMark AR trolley tracking system.
 */
class Visualizer {
public:
    /**
     * @brief Render all visual overlays onto the output frame.
     * @param frame Mutable BGR frame to draw on.
     * @param poses Detected marker poses for overlay rendering.
     * @param predicted Predicted state from the Kalman filter.
     * @param warning Current collision warning status.
     * @param cameraMatrix Camera intrinsic matrix for 3D projection.
     * @param distCoeffs Camera distortion coefficients.
     */
    void render(cv::Mat& frame,
                const std::vector<MarkerPose>& poses,
                const PredictedState& predicted,
                const CollisionWarning& warning,
                const cv::Mat& cameraMatrix,
                const cv::Mat& distCoeffs);

private:
    int frameCount_ = 0;  ///< Frame counter for flashing warning animation

    void drawMarkerOverlay_(cv::Mat& frame,
                            const std::vector<MarkerPose>& poses,
                            const cv::Mat& cameraMatrix,
                            const cv::Mat& distCoeffs);

    void drawVelocityArrow_(cv::Mat& frame,
                            const std::vector<MarkerPose>& poses,
                            const PredictedState& predicted,
                            const cv::Mat& cameraMatrix,
                            const cv::Mat& distCoeffs);

    void drawCollisionWarning_(cv::Mat& frame,
                               const CollisionWarning& warning);
};
