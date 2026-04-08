#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>

struct SterileZone {
    std::string name;          
    cv::Point3f centre;        
    float       radius;        
};

struct FieldBreachAlert {
    bool        active;                
    std::string zoneName;              
    float       timeToImpactSeconds;   
    float       distanceMetres;        
};

class SterileFieldMonitor {
public:

    void addSterileZone(const SterileZone& zone);

    // currentPos     — position this frame (from Kalman statePost)
    // velocity       — current velocity estimate (m/s)
    // predictedPos   — extrapolated position at horizonSeconds ahead
    // horizonSeconds — prediction lookahead
    FieldBreachAlert check(const cv::Point3f& currentPos,
                           const cv::Point3f& velocity,
                           const cv::Point3f& predictedPos,
                           float              horizonSeconds) const;

private:
    std::vector<SterileZone> zones_;  
};