

#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/instrument_detector.hpp>
#include <vector>
#include <string>

struct InstrumentPose {
    int id;                                
    cv::Vec3d rvec;                        
    cv::Vec3d tvec;                        
    std::vector<cv::Point2f> corners;      
    double detectionConfidence;            
};

class InstrumentDetector {
public:

    explicit InstrumentDetector(const std::string& calibrationFile,
                           float markerSideMetres = 0.10f);

    std::vector<InstrumentPose> detect(const cv::Mat& frame);

    void drawDebug(cv::Mat& frame,
                   const std::vector<InstrumentPose>& poses) const;

    cv::Mat cameraMatrix() const;

    cv::Mat distCoeffs()   const;

private:
    cv::Mat             K_, D_;           
    float               markerSide_;      
    cv::aruco::InstrumentDetector detector_;   
    void loadCalibration_(const std::string& path);
};
