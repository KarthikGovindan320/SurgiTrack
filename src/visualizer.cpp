/*
 * visualizer.cpp
 * Author: Mukul Dahiya
 * Date: 2026-04-05
 *
 * Implements the Visualizer class. Renders semi-transparent marker overlays,
 * 3D coordinate axes, velocity vectors, and flashing collision warning
 * borders using only OpenCV drawing primitives (no external UI libraries).
 *
 * Visual elements rendered:
 *   1. Semi-transparent green polygon overlay on each marker (alpha=0.3)
 *   2. 3D axis tripod at each marker centre (5 cm axis length)
 *   3. Marker ID text with dark background rectangle
 *   4. Velocity vector arrow from centroid showing movement direction
 *   5. Red warning border and flashing text when collision predicted
 */

#include "visualizer.hpp"
#include <cmath>
#include <cfloat>

// ---------------------------------------------------------------------------
// Named constants for rendering parameters
// ---------------------------------------------------------------------------

/// Alpha blending factor for the semi-transparent marker overlay.
/// 0.3 provides visible highlighting without obscuring the marker content.
constexpr double MARKER_OVERLAY_ALPHA       = 0.3;

/// Length of the 3D coordinate axes drawn at each marker centre (metres)
constexpr float  AXIS_LENGTH_METRES         = 0.05f;

/// Thickness of the collision warning border rectangle (pixels)
constexpr int    WARNING_BORDER_THICKNESS   = 4;

/// Frame interval for flashing the collision warning text.
/// Warning text visibility is toggled every this many frames,
/// creating a visual pulse effect to attract attention.
constexpr int    WARNING_FLASH_INTERVAL     = 15;

/// Font scale for marker ID text
constexpr double ID_TEXT_FONT_SCALE         = 0.6;

/// Font scale for warning text
constexpr double WARNING_TEXT_FONT_SCALE    = 0.8;

/// Font scale for velocity label
constexpr double VELOCITY_TEXT_FONT_SCALE   = 0.5;

/// Minimum velocity magnitude (m/s) to draw the velocity arrow.
/// Below this threshold, the arrow is not drawn to avoid visual noise.
constexpr float  MIN_VELOCITY_FOR_ARROW     = 0.02f;

/// Scale factor for converting velocity (m/s) to arrow length (pixels).
/// A trolley moving at 0.8 m/s will produce an arrow 80 pixels long.
constexpr float  VELOCITY_ARROW_SCALE       = 100.0f;

// ---------------------------------------------------------------------------
// render() — Main rendering entry point
// ---------------------------------------------------------------------------

/**
 * @brief Render all AR overlay elements onto the output frame.
 *
 * Calls private methods in sequence to draw marker overlays, velocity
 * arrows, and collision warnings. All rendering uses OpenCV drawing
 * primitives only, with no external dependencies.
 *
 * @param frame        Mutable BGR frame to draw on.
 * @param poses        Detected marker poses for overlay rendering.
 * @param predicted    Predicted state from the Kalman filter.
 * @param warning      Current collision warning status.
 * @param cameraMatrix Camera intrinsic matrix for 3D-to-2D projection.
 * @param distCoeffs   Camera distortion coefficients.
 */
void Visualizer::render(cv::Mat& frame,
                         const std::vector<MarkerPose>& poses,
                         const PredictedState& predicted,
                         const CollisionWarning& warning,
                         const cv::Mat& cameraMatrix,
                         const cv::Mat& distCoeffs) {
    // Draw marker overlays (semi-transparent fill, axes, ID text)
    drawMarkerOverlay_(frame, poses, cameraMatrix, distCoeffs);

    // Draw velocity direction arrow from the trolley centroid
    drawVelocityArrow_(frame, poses, predicted, cameraMatrix, distCoeffs);

    // Draw collision warning border and text (if active)
    drawCollisionWarning_(frame, warning);

    // Increment frame counter for animation timing
    frameCount_++;
}

// ---------------------------------------------------------------------------
// drawMarkerOverlay_() — Semi-transparent fills, axes, and ID labels
// ---------------------------------------------------------------------------

/**
 * @brief Draw visual overlays on each detected marker.
 *
 * For each marker:
 *   - Fills the marker quadrilateral with a semi-transparent green polygon
 *     (alpha-blended using cv::addWeighted at 0.3 opacity)
 *   - Draws 3D coordinate axes at the marker centre (5 cm axis length)
 *   - Renders the marker ID as white text on a dark background rectangle
 *     at the top-left corner of the marker bounding box
 *
 * @param frame        BGR frame to draw on.
 * @param poses        Detected marker poses.
 * @param cameraMatrix Camera intrinsic matrix.
 * @param distCoeffs   Camera distortion coefficients.
 */
void Visualizer::drawMarkerOverlay_(cv::Mat& frame,
                                     const std::vector<MarkerPose>& poses,
                                     const cv::Mat& cameraMatrix,
                                     const cv::Mat& distCoeffs) {
    for (const auto& mp : poses) {
        // --- Semi-transparent green polygon overlay ---
        // Create a copy of the frame for blending
        cv::Mat overlay = frame.clone();

        // Convert float corners to integer points for fillConvexPoly
        std::vector<cv::Point> intCorners;
        for (const auto& c : mp.corners) {
            intCorners.emplace_back(static_cast<int>(c.x),
                                     static_cast<int>(c.y));
        }

        // Fill the marker area with green on the overlay
        cv::fillConvexPoly(overlay, intCorners, cv::Scalar(0, 255, 0));

        // Alpha-blend: result = alpha * overlay + (1-alpha) * original
        cv::addWeighted(overlay, MARKER_OVERLAY_ALPHA,
                        frame, 1.0 - MARKER_OVERLAY_ALPHA, 0, frame);

        // --- 3D coordinate axes ---
        // Draw the standard RGB axis tripod (X=red, Y=green, Z=blue)
        // at each marker centre. Axis length is 5 cm (0.05 m).
        cv::drawFrameAxes(frame, cameraMatrix, distCoeffs,
                          mp.rvec, mp.tvec, AXIS_LENGTH_METRES);

        // --- Marker ID text with dark background ---
        // Position text at the top-left corner of the marker bounding box
        cv::Point textPos(static_cast<int>(mp.corners[0].x),
                          static_cast<int>(mp.corners[0].y) - 10);

        std::string idText = "ID:" + std::to_string(mp.id);
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(idText, cv::FONT_HERSHEY_SIMPLEX,
                                             ID_TEXT_FONT_SCALE, 2, &baseline);

        // Draw dark background rectangle for text readability
        cv::rectangle(frame,
                      cv::Point(textPos.x - 2, textPos.y - textSize.height - 2),
                      cv::Point(textPos.x + textSize.width + 2, textPos.y + baseline + 2),
                      cv::Scalar(0, 0, 0), cv::FILLED);

        // Draw white text on the dark background
        cv::putText(frame, idText, textPos, cv::FONT_HERSHEY_SIMPLEX,
                    ID_TEXT_FONT_SCALE, cv::Scalar(255, 255, 255), 2);
    }
}

// ---------------------------------------------------------------------------
// drawVelocityArrow_() — Direction of movement indicator
// ---------------------------------------------------------------------------

/**
 * @brief Draw a velocity vector arrow from the trolley centroid.
 *
 * Projects the current centroid position to screen coordinates, then
 * draws an arrow in the direction of the predicted velocity vector.
 * Arrow length is proportional to speed (100 pixels per m/s).
 * The arrow is not drawn if velocity magnitude is below 0.02 m/s
 * to avoid visual noise when the trolley is stationary.
 *
 * @param frame        BGR frame to draw on.
 * @param poses        Detected marker poses (for computing screen centroid).
 * @param predicted    Predicted state containing velocity vector.
 * @param cameraMatrix Camera intrinsic matrix.
 * @param distCoeffs   Camera distortion coefficients.
 */
void Visualizer::drawVelocityArrow_(cv::Mat& frame,
                                     const std::vector<MarkerPose>& poses,
                                     const PredictedState& predicted,
                                     const cv::Mat& cameraMatrix,
                                     const cv::Mat& distCoeffs) {
    if (poses.empty()) return;

    // Compute the 2D screen centroid from marker corners
    cv::Point2f screenCentroid(0, 0);
    int totalCorners = 0;
    for (const auto& mp : poses) {
        for (const auto& c : mp.corners) {
            screenCentroid.x += c.x;
            screenCentroid.y += c.y;
            totalCorners++;
        }
    }
    if (totalCorners == 0) return;
    screenCentroid.x /= static_cast<float>(totalCorners);
    screenCentroid.y /= static_cast<float>(totalCorners);

    // Compute velocity magnitude
    float speed = std::sqrt(predicted.velocity.x * predicted.velocity.x +
                             predicted.velocity.y * predicted.velocity.y +
                             predicted.velocity.z * predicted.velocity.z);

    if (speed < MIN_VELOCITY_FOR_ARROW) return;  // Skip if essentially stationary

    // Project the velocity vector to 2D screen direction.
    // We use a simplified projection: scale velocity x and y
    // components by the arrow scale factor. The z component
    // affects arrow length but not 2D direction.
    float arrowDx = predicted.velocity.x * VELOCITY_ARROW_SCALE;
    float arrowDy = predicted.velocity.y * VELOCITY_ARROW_SCALE;

    // Scale arrow length by speed for proportional representation
    float arrowLength = speed * VELOCITY_ARROW_SCALE;
    float maxArrowLength = 150.0f;  // Cap at 150 pixels to avoid clutter
    if (arrowLength > maxArrowLength) {
        float scale = maxArrowLength / arrowLength;
        arrowDx *= scale;
        arrowDy *= scale;
    }

    cv::Point arrowStart(static_cast<int>(screenCentroid.x),
                          static_cast<int>(screenCentroid.y));
    cv::Point arrowEnd(static_cast<int>(screenCentroid.x + arrowDx),
                        static_cast<int>(screenCentroid.y + arrowDy));

    // Draw the velocity arrow in cyan with a filled arrowhead
    cv::arrowedLine(frame, arrowStart, arrowEnd,
                    cv::Scalar(255, 255, 0), 2, cv::LINE_AA, 0, 0.3);

    // Label the speed near the arrow tip
    std::string speedText = cv::format("%.2f m/s", speed);
    cv::putText(frame, speedText,
                cv::Point(arrowEnd.x + 5, arrowEnd.y - 5),
                cv::FONT_HERSHEY_SIMPLEX, VELOCITY_TEXT_FONT_SCALE,
                cv::Scalar(255, 255, 0), 1);
}

// ---------------------------------------------------------------------------
// drawCollisionWarning_() — Red border and flashing text
// ---------------------------------------------------------------------------

/**
 * @brief Draw collision warning indicators when a threat is predicted.
 *
 * When CollisionWarning.active is true:
 *   - Draws a red rectangle border around the entire frame (4 px thick)
 *   - Displays flashing text at the top of the frame:
 *     "WARNING: Predicted collision with [zoneName] in [X.X]s"
 *   - Text flashes by toggling visibility every 15 frames
 *
 * When inactive, no warning elements are drawn.
 *
 * @param frame   BGR frame to draw on.
 * @param warning Current collision warning status.
 */
void Visualizer::drawCollisionWarning_(cv::Mat& frame,
                                        const CollisionWarning& warning) {
    if (!warning.active) return;

    // --- Red border around the entire frame ---
    cv::rectangle(frame,
                  cv::Point(0, 0),
                  cv::Point(frame.cols - 1, frame.rows - 1),
                  cv::Scalar(0, 0, 255),  // Red in BGR
                  WARNING_BORDER_THICKNESS);

    // --- Flashing warning text ---
    // Toggle visibility every WARNING_FLASH_INTERVAL frames.
    // This creates a pulse effect that draws attention without
    // being constantly on-screen (which users would quickly habituate to).
    bool showText = (frameCount_ / WARNING_FLASH_INTERVAL) % 2 == 0;

    if (showText) {
        // Format the warning message
        std::string warningText;
        if (warning.timeToImpactSeconds < FLT_MAX) {
            warningText = cv::format("WARNING: Predicted collision with %s in %.1fs",
                                      warning.zoneName.c_str(),
                                      warning.timeToImpactSeconds);
        } else {
            warningText = "WARNING: Inside hazard zone " + warning.zoneName;
        }

        // Compute text size for background rectangle
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(warningText, cv::FONT_HERSHEY_SIMPLEX,
                                             WARNING_TEXT_FONT_SCALE, 2, &baseline);

        // Centre the text horizontally at the top of the frame
        int textX = (frame.cols - textSize.width) / 2;
        int textY = 40;  // Fixed Y position near the top

        // Draw semi-transparent dark background for text readability
        cv::Mat overlay = frame.clone();
        cv::rectangle(overlay,
                      cv::Point(textX - 10, textY - textSize.height - 10),
                      cv::Point(textX + textSize.width + 10, textY + baseline + 10),
                      cv::Scalar(0, 0, 0), cv::FILLED);
        cv::addWeighted(overlay, 0.7, frame, 0.3, 0, frame);

        // Draw bold red warning text
        cv::putText(frame, warningText,
                    cv::Point(textX, textY),
                    cv::FONT_HERSHEY_SIMPLEX, WARNING_TEXT_FONT_SCALE,
                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
}
