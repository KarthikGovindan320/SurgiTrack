

#pragma once
#include <opencv2/opencv.hpp>
#include "instrument_detector.hpp"
#include "instrument_tracker.hpp"
#include "sterile_field_monitor.hpp"

class ORVisualizer {
public:

    void render(cv::Mat& frame,
                const std::vector<InstrumentPose>& poses,
                const TrackedInstrumentState& predicted,
                const FieldBreachAlert& warning,
                const cv::Mat& cameraMatrix,
                const cv::Mat& distCoeffs);

private:
    int frameCount_ = 0;  

    void drawMarkerOverlay_(cv::Mat& frame,
                            const std::vector<InstrumentPose>& poses,
                            const cv::Mat& cameraMatrix,
                            const cv::Mat& distCoeffs);

    void drawVelocityArrow_(cv::Mat& frame,
                            const std::vector<InstrumentPose>& poses,
                            const TrackedInstrumentState& predicted,
                            const cv::Mat& cameraMatrix,
                            const cv::Mat& distCoeffs);

    void drawFieldBreachAlert_(cv::Mat& frame,
                               const FieldBreachAlert& warning);
};
