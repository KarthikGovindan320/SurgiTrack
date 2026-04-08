

#pragma once
#include <opencv2/video/tracking.hpp>
#include <vector>
#include "instrument_detector.hpp"

struct TrackedInstrumentState {
    cv::Point3f position;    
    cv::Point3f velocity;    
    float       uncertainty; 
};

#include <unordered_map>

struct TrackerData {
    cv::KalmanFilter kf;
    int framesMissing = 0;
};

class InstrumentTracker {
public:

    explicit InstrumentTracker(float dt = 1.0f/30.0f,
                             float processNoise   = 1e-4f,
                             float measurementNoise = 5e-3f,
                             int maxMissingFrames = 30);

    void update(const std::vector<InstrumentPose>& poses);

    TrackedInstrumentState predict(int id, float horizonSeconds = 0.5f) const;

    TrackedInstrumentState predictCentroid(float horizonSeconds = 0.5f) const;

    std::vector<int> trackedIds() const;

private:
    std::unordered_map<int, TrackerData> filters_;
    float            dt_;           
    float            processNoise_;      
    float            measurementNoise_;  
    int              maxMissingFrames_;
    
    void initFilter_(int id, const cv::Point3f& initialPos);
};
