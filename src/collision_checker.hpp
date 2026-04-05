/*
 * collision_checker.hpp
 * Author: Aditya Yadav
 * Date: 2026-04-05
 *
 * Declares the CollisionChecker class that defines static hazard zones
 * in 3D space and checks whether the predicted trolley trajectory
 * intersects any zone, issuing warnings with estimated time-to-impact.
 */

#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>

/**
 * @brief Defines a spherical hazard zone in 3D camera/world coordinates.
 */
struct HazardZone {
    std::string name;          ///< Human-readable zone name, e.g. "PatientBed_Left"
    cv::Point3f centre;        ///< Zone centre position in metres
    float       radius;        ///< Zone radius in metres
};

/**
 * @brief Holds the result of a collision prediction check.
 */
struct CollisionWarning {
    bool        active;                ///< Whether a collision is predicted
    std::string zoneName;              ///< Name of the threatened zone
    float       timeToImpactSeconds;   ///< Estimated time to collision
    float       distanceMetres;        ///< Current distance to zone boundary
};

/**
 * @brief Checks predicted trolley positions against configured hazard
 *        zones and issues collision warnings.
 */
class CollisionChecker {
public:
    /**
     * @brief Add a hazard zone to the checker.
     * @param zone HazardZone struct defining the zone geometry.
     */
    void addZone(const HazardZone& zone);

    /**
     * @brief Check if the predicted position falls within any hazard zone.
     * @param predictedPos Predicted 3D position of the trolley centroid.
     * @param horizonSeconds Look-ahead time used for the prediction.
     * @return CollisionWarning with details of the nearest threat, if any.
     */
    CollisionWarning check(const cv::Point3f& predictedPos,
                           float              horizonSeconds) const;

private:
    std::vector<HazardZone> zones_;  ///< Registered hazard zones
};
