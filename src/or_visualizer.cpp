

#include "or_visualizer.hpp"
#include <cmath>
#include <cfloat>

constexpr double MARKER_OVERLAY_ALPHA       = 0.3;

constexpr float  AXIS_LENGTH_METRES         = 0.05f;

constexpr int    WARNING_BORDER_THICKNESS   = 4;

constexpr int    WARNING_FLASH_INTERVAL     = 15;

constexpr double ID_TEXT_FONT_SCALE         = 0.6;

constexpr double WARNING_TEXT_FONT_SCALE    = 0.8;

constexpr double VELOCITY_TEXT_FONT_SCALE   = 0.5;

constexpr float  MIN_VELOCITY_FOR_ARROW     = 0.02f;

constexpr float  VELOCITY_ARROW_SCALE       = 100.0f;

void ORVisualizer::render(cv::Mat& frame,
                         const std::vector<InstrumentPose>& poses,
                         const TrackedInstrumentState& predicted,
                         const FieldBreachAlert& warning,
                         const cv::Mat& cameraMatrix,
                         const cv::Mat& distCoeffs) {

    drawMarkerOverlay_(frame, poses, cameraMatrix, distCoeffs);

    drawVelocityArrow_(frame, poses, predicted, cameraMatrix, distCoeffs);

    drawFieldBreachAlert_(frame, warning);

    frameCount_++;
}

void ORVisualizer::drawMarkerOverlay_(cv::Mat& frame,
                                     const std::vector<InstrumentPose>& poses,
                                     const cv::Mat& cameraMatrix,
                                     const cv::Mat& distCoeffs) {
    for (const auto& mp : poses) {

        cv::Mat overlay = frame.clone();

        std::vector<cv::Point> intCorners;
        for (const auto& c : mp.corners) {
            intCorners.emplace_back(static_cast<int>(c.x),
                                     static_cast<int>(c.y));
        }

        cv::fillConvexPoly(overlay, intCorners, cv::Scalar(0, 255, 0));

        cv::addWeighted(overlay, MARKER_OVERLAY_ALPHA,
                        frame, 1.0 - MARKER_OVERLAY_ALPHA, 0, frame);

        cv::drawFrameAxes(frame, cameraMatrix, distCoeffs,
                          mp.rvec, mp.tvec, AXIS_LENGTH_METRES);

        cv::Point textPos(static_cast<int>(mp.corners[0].x),
                          static_cast<int>(mp.corners[0].y) - 10);

        std::string idText = "ID:" + std::to_string(mp.id);
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(idText, cv::FONT_HERSHEY_SIMPLEX,
                                             ID_TEXT_FONT_SCALE, 2, &baseline);

        cv::rectangle(frame,
                      cv::Point(textPos.x - 2, textPos.y - textSize.height - 2),
                      cv::Point(textPos.x + textSize.width + 2, textPos.y + baseline + 2),
                      cv::Scalar(0, 0, 0), cv::FILLED);

        cv::putText(frame, idText, textPos, cv::FONT_HERSHEY_SIMPLEX,
                    ID_TEXT_FONT_SCALE, cv::Scalar(255, 255, 255), 2);
    }
}

void ORVisualizer::drawVelocityArrow_(cv::Mat& frame,
                                     const std::vector<InstrumentPose>& poses,
                                     const TrackedInstrumentState& predicted,
                                     const cv::Mat& cameraMatrix,
                                     const cv::Mat& distCoeffs) {
    if (poses.empty()) return;

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

    float speed = std::sqrt(predicted.velocity.x * predicted.velocity.x +
                             predicted.velocity.y * predicted.velocity.y +
                             predicted.velocity.z * predicted.velocity.z);

    if (speed < MIN_VELOCITY_FOR_ARROW) return;  

    float arrowDx = predicted.velocity.x * VELOCITY_ARROW_SCALE;
    float arrowDy = predicted.velocity.y * VELOCITY_ARROW_SCALE;

    float arrowLength = speed * VELOCITY_ARROW_SCALE;
    float maxArrowLength = 150.0f;  
    if (arrowLength > maxArrowLength) {
        float scale = maxArrowLength / arrowLength;
        arrowDx *= scale;
        arrowDy *= scale;
    }

    cv::Point arrowStart(static_cast<int>(screenCentroid.x),
                          static_cast<int>(screenCentroid.y));
    cv::Point arrowEnd(static_cast<int>(screenCentroid.x + arrowDx),
                        static_cast<int>(screenCentroid.y + arrowDy));

    cv::arrowedLine(frame, arrowStart, arrowEnd,
                    cv::Scalar(255, 255, 0), 2, cv::LINE_AA, 0, 0.3);

    std::string speedText = cv::format("%.2f m/s", speed);
    cv::putText(frame, speedText,
                cv::Point(arrowEnd.x + 5, arrowEnd.y - 5),
                cv::FONT_HERSHEY_SIMPLEX, VELOCITY_TEXT_FONT_SCALE,
                cv::Scalar(255, 255, 0), 1);
}

void ORVisualizer::drawFieldBreachAlert_(cv::Mat& frame,
                                        const FieldBreachAlert& warning) {
    if (!warning.active) return;

    cv::rectangle(frame,
                  cv::Point(0, 0),
                  cv::Point(frame.cols - 1, frame.rows - 1),
                  cv::Scalar(0, 0, 255),  
                  WARNING_BORDER_THICKNESS);

    bool showText = (frameCount_ / WARNING_FLASH_INTERVAL) % 2 == 0;

    if (showText) {

        std::string warningText;
        if (warning.timeToImpactSeconds < FLT_MAX) {
            warningText = cv::format("WARNING: Predicted sterile field breach with %s in %.1fs",
                                      warning.zoneName.c_str(),
                                      warning.timeToImpactSeconds);
        } else {
            warningText = "WARNING: Inside sterile zone " + warning.zoneName;
        }

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(warningText, cv::FONT_HERSHEY_SIMPLEX,
                                             WARNING_TEXT_FONT_SCALE, 2, &baseline);

        int textX = (frame.cols - textSize.width) / 2;
        int textY = 40;  

        cv::Mat overlay = frame.clone();
        cv::rectangle(overlay,
                      cv::Point(textX - 10, textY - textSize.height - 10),
                      cv::Point(textX + textSize.width + 10, textY + baseline + 10),
                      cv::Scalar(0, 0, 0), cv::FILLED);
        cv::addWeighted(overlay, 0.7, frame, 0.3, 0, frame);

        cv::putText(frame, warningText,
                    cv::Point(textX, textY),
                    cv::FONT_HERSHEY_SIMPLEX, WARNING_TEXT_FONT_SCALE,
                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
}
