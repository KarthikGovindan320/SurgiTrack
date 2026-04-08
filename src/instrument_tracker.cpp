#include "instrument_tracker.hpp"
#include <cmath>
#include <iostream>

constexpr int STATE_DIM       = 6;

constexpr int MEASUREMENT_DIM = 3;

constexpr int CONTROL_DIM     = 0;

InstrumentTracker::InstrumentTracker(float dt, float processNoise,
                                   float measurementNoise, int maxMissingFrames)
    : dt_(dt),
      processNoise_(processNoise), measurementNoise_(measurementNoise),
      maxMissingFrames_(maxMissingFrames)
{
}

void InstrumentTracker::initFilter_(int id, const cv::Point3f& p) {
    cv::KalmanFilter kf(STATE_DIM, MEASUREMENT_DIM, CONTROL_DIM, CV_32F);

    cv::setIdentity(kf.transitionMatrix);
    kf.transitionMatrix.at<float>(0, 3) = dt_;  
    kf.transitionMatrix.at<float>(1, 4) = dt_;  
    kf.transitionMatrix.at<float>(2, 5) = dt_;  

    kf.measurementMatrix = cv::Mat::zeros(MEASUREMENT_DIM, STATE_DIM, CV_32F);
    kf.measurementMatrix.at<float>(0, 0) = 1.0f;
    kf.measurementMatrix.at<float>(1, 1) = 1.0f;
    kf.measurementMatrix.at<float>(2, 2) = 1.0f;

    cv::setIdentity(kf.processNoiseCov, cv::Scalar(processNoise_));
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(measurementNoise_));
    cv::setIdentity(kf.errorCovPost, cv::Scalar(1.0));

    kf.statePost.at<float>(0) = p.x;   
    kf.statePost.at<float>(1) = p.y;   
    kf.statePost.at<float>(2) = p.z;   
    kf.statePost.at<float>(3) = 0.0f;  
    kf.statePost.at<float>(4) = 0.0f;  
    kf.statePost.at<float>(5) = 0.0f;  

    TrackerData td;
    td.kf = kf;
    td.framesMissing = 0;
    filters_[id] = td;
    
    std::cout << "[InstrumentTracker] Initialised ID " << id << " at position ("
              << p.x << ", " << p.y << ", " << p.z << ")\n";
}

void InstrumentTracker::update(const std::vector<InstrumentPose>& poses) {
    std::unordered_map<int, bool> matchedThisFrame;

    for (const auto& mp : poses) {
        matchedThisFrame[mp.id] = true;
        
        cv::Point3f centroid(
            static_cast<float>(mp.tvec[0]),
            static_cast<float>(mp.tvec[1]),
            static_cast<float>(mp.tvec[2])
        );

        if (filters_.find(mp.id) == filters_.end()) {
            initFilter_(mp.id, centroid);
        } else {
            auto& td = filters_[mp.id];
            td.framesMissing = 0;
            td.kf.predict();

            cv::Mat measurement = (cv::Mat_<float>(MEASUREMENT_DIM, 1)
                << centroid.x, centroid.y, centroid.z);
            td.kf.correct(measurement);
        }
    }

    // Handle missing items
    for (auto it = filters_.begin(); it != filters_.end(); ) {
        if (!matchedThisFrame[it->first]) {
            it->second.framesMissing++;
            if (it->second.framesMissing > maxMissingFrames_) {
                std::cout << "[InstrumentTracker] Dropping ID " << it->first << " (missing too long)\n";
                it = filters_.erase(it);
                continue;
            } else {
                it->second.kf.predict();  // Predict forward for missing frames
            }
        }
        ++it;
    }
}

TrackedInstrumentState InstrumentTracker::predict(int id, float horizonSeconds) const {
    TrackedInstrumentState result;
    result.position    = cv::Point3f(0.0f, 0.0f, 0.0f);
    result.velocity    = cv::Point3f(0.0f, 0.0f, 0.0f);
    result.uncertainty = 1e6f;  

    auto it = filters_.find(id);
    if (it == filters_.end()) {
        return result;
    }

    int N = static_cast<int>(std::round(horizonSeconds / dt_));
    if (N < 1) N = 1;

    cv::Mat state = it->second.kf.statePost.clone();
    for (int i = 0; i < N; ++i) {
        state = it->second.kf.transitionMatrix * state;
    }

    result.position = cv::Point3f(state.at<float>(0), state.at<float>(1), state.at<float>(2));
    result.velocity = cv::Point3f(state.at<float>(3), state.at<float>(4), state.at<float>(5));

    const cv::Mat& P = it->second.kf.errorCovPost;
    result.uncertainty = P.at<float>(0, 0) + P.at<float>(1, 1) + P.at<float>(2, 2);    

    return result;
}

TrackedInstrumentState InstrumentTracker::predictCentroid(float horizonSeconds) const {
    TrackedInstrumentState result;
    result.position    = cv::Point3f(0.0f, 0.0f, 0.0f);
    result.velocity    = cv::Point3f(0.0f, 0.0f, 0.0f);
    result.uncertainty = 1e6f;  
    
    if (filters_.empty()) return result;
    
    for (const auto& pair : filters_) {
        TrackedInstrumentState st = predict(pair.first, horizonSeconds);
        result.position.x += st.position.x;
        result.position.y += st.position.y;
        result.position.z += st.position.z;
        
        result.velocity.x += st.velocity.x;
        result.velocity.y += st.velocity.y;
        result.velocity.z += st.velocity.z;
    }
    
    float n = static_cast<float>(filters_.size());
    result.position.x /= n;
    result.position.y /= n;
    result.position.z /= n;
    
    result.velocity.x /= n;
    result.velocity.y /= n;
    result.velocity.z /= n;
    result.uncertainty = 0.0f; // Doesn't matter for centroid overlay
    
    return result;
}

std::vector<int> InstrumentTracker::trackedIds() const {
    std::vector<int> ids;
    ids.reserve(filters_.size());
    for (const auto& pair : filters_) {
        ids.push_back(pair.first);
    }
    return ids;
}
