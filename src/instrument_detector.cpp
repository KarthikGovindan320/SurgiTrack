#include "instrument_detector.hpp"
#include <stdexcept>
#include <iostream>

constexpr double CLOSE_RANGE_THRESHOLD_METRES = 1.5;

constexpr float  DEBUG_AXIS_LENGTH_METRES     = 0.05f;

InstrumentDetector::InstrumentDetector(const std::string& calibFile, float side)
    : markerSide_(side)
{

    loadCalibration_(calibFile);

    auto dict   = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    auto params = cv::aruco::DetectorParameters();

    params.adaptiveThreshWinSizeMax = 53;

    params.cornerRefinementMethod   =
        cv::aruco::CORNER_REFINE_SUBPIX;

    detector_ = cv::aruco::ArucoDetector(dict, params);
}

std::vector<InstrumentPose> InstrumentDetector::detect(const cv::Mat& frame) {
    std::vector<int>                     ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    detector_.detectMarkers(frame, corners, ids, rejected);

    std::vector<InstrumentPose> poses;
    if (ids.empty()) return poses;

    int nMarkers = static_cast<int>(corners.size());
    std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    float halfSide = markerSide_ / 2.0f;
    std::vector<cv::Point3f> objPoints = {
        {-halfSide,  halfSide, 0.0f},   
        { halfSide,  halfSide, 0.0f},   
        { halfSide, -halfSide, 0.0f},   
        {-halfSide, -halfSide, 0.0f}    
    };

    for (int i = 0; i < nMarkers; ++i) {
        cv::solvePnP(objPoints, corners[i], K_, D_,
                      rvecs[i], tvecs[i], false,
                      cv::SOLVEPNP_ITERATIVE);
    }

    for (size_t i = 0; i < ids.size(); ++i) {
        InstrumentPose mp;
        mp.id      = ids[i];
        mp.rvec    = rvecs[i];
        mp.tvec    = tvecs[i];
        mp.corners = corners[i];

        mp.detectionConfidence = 1.0;
        poses.push_back(mp);
    }
    return poses;
}

void InstrumentDetector::drawDebug(cv::Mat& frame,
                               const std::vector<InstrumentPose>& poses) const {
    for (const auto& mp : poses) {

        std::vector<std::vector<cv::Point2f>> singleCorner = {mp.corners};
        std::vector<int> singleId = {mp.id};
        cv::aruco::drawDetectedMarkers(frame, singleCorner, singleId);

        cv::drawFrameAxes(frame, K_, D_, mp.rvec, mp.tvec,
                          DEBUG_AXIS_LENGTH_METRES);

        double zDist = mp.tvec[2];
        cv::Scalar colour;
        if (zDist < CLOSE_RANGE_THRESHOLD_METRES) {
            colour = cv::Scalar(0, 255, 0);   
        } else {
            colour = cv::Scalar(255, 0, 0);   
        }

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

        std::string distText = cv::format("%.2fm", zDist);
        cv::putText(frame, distText,
                    cv::Point(static_cast<int>(centre.x) + 15,
                              static_cast<int>(centre.y) - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, colour, 2);
    }
}

cv::Mat InstrumentDetector::cameraMatrix() const { return K_.clone(); }
cv::Mat InstrumentDetector::distCoeffs()   const { return D_.clone(); }

void InstrumentDetector::loadCalibration_(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("InstrumentDetector: cannot open calibration file: " + path);
    }

    fs["camera_matrix"] >> K_;
    fs["dist_coeffs"]   >> D_;

    if (K_.empty() || D_.empty()) {
        throw std::runtime_error("InstrumentDetector: calibration file missing camera_matrix or dist_coeffs");
    }

    std::cout << "[InstrumentDetector] Loaded calibration from " << path << std::endl;
    std::cout << "  Camera matrix: " << K_.size() << std::endl;
    std::cout << "  Distortion coeffs: " << D_.size() << std::endl;

    fs.release();
}