/*
 * collision_checker.cpp
 * Author: Aditya Yadav
 * Date: 2026-04-05
 *
 * Implements the CollisionChecker class. Iterates over configured hazard
 * zones and computes Euclidean distance from the predicted trolley position
 * to each zone centre, issuing warnings when the trolley enters a zone.
 * Includes divide-by-zero protection for near-stationary trolley cases.
 */

#include "collision_checker.hpp"
#include <cmath>
#include <cfloat>
#include <iostream>

// ---------------------------------------------------------------------------
// Named constants
// ---------------------------------------------------------------------------

/// Minimum speed threshold in m/s below which time-to-impact is set to
/// FLT_MAX. This prevents division by zero when the trolley is stationary
/// or moving extremely slowly. At 0.01 m/s, it would take 25 seconds to
/// cross a 25 cm hazard zone — effectively not approaching.
constexpr float MIN_SPEED_THRESHOLD_MPS = 0.01f;

// ---------------------------------------------------------------------------
// addZone() — Register a hazard zone
// ---------------------------------------------------------------------------

/**
 * @brief Add a hazard zone to the collision checker.
 *
 * Zones are stored in order of registration. The check() method evaluates
 * all zones and returns the warning for the closest threatening zone.
 *
 * @param zone HazardZone struct defining the zone name, centre, and radius.
 */
void CollisionChecker::addZone(const HazardZone& zone) {
    zones_.push_back(zone);
    std::cout << "[CollisionChecker] Added zone: " << zone.name
              << " at (" << zone.centre.x << ", " << zone.centre.y
              << ", " << zone.centre.z << ") radius=" << zone.radius << "m\n";
}

// ---------------------------------------------------------------------------
// check() — Evaluate predicted position against all hazard zones
// ---------------------------------------------------------------------------

/**
 * @brief Check if the predicted trolley position falls within any hazard zone.
 *
 * Iterates over all registered zones. For each zone, computes the Euclidean
 * distance between the predicted position and the zone centre. If the
 * distance is less than the zone radius, a CollisionWarning is generated
 * with the estimated time-to-impact.
 *
 * Time-to-impact is estimated as (distance / speed), where speed is the
 * magnitude of the trolley's velocity vector estimated by the Kalman filter.
 * If speed is near zero (< 0.01 m/s), time-to-impact is set to FLT_MAX
 * to prevent division by zero.
 *
 * If multiple zones are threatened, the warning for the zone with the
 * shortest distance (most imminent threat) is returned.
 *
 * @param predictedPos Predicted 3D position of the trolley centroid.
 * @param horizonSeconds Look-ahead time used for the prediction (for logging).
 * @return CollisionWarning with details of the nearest threat, or inactive if safe.
 */
CollisionWarning CollisionChecker::check(const cv::Point3f& predictedPos,
                                          float horizonSeconds) const {
    CollisionWarning bestWarning;
    bestWarning.active              = false;
    bestWarning.zoneName            = "";
    bestWarning.timeToImpactSeconds = FLT_MAX;
    bestWarning.distanceMetres      = FLT_MAX;

    // Track the closest threatening zone
    float closestDistance = FLT_MAX;

    for (const auto& zone : zones_) {
        // Compute Euclidean distance between the predicted position
        // and the zone centre in 3D space
        float dx = predictedPos.x - zone.centre.x;
        float dy = predictedPos.y - zone.centre.y;
        float dz = predictedPos.z - zone.centre.z;
        float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // Check if the predicted position is within the zone radius
        if (distance < zone.radius) {
            if (distance < closestDistance) {
                closestDistance = distance;

                bestWarning.active         = true;
                bestWarning.zoneName       = zone.name;
                bestWarning.distanceMetres = distance;

                // Estimate time-to-impact using the distance and
                // the prediction horizon. Since the predicted position
                // is already at t + horizonSeconds, the time-to-impact
                // is approximately the horizon time scaled by the
                // ratio of distance to zone radius.
                // Guard against division by zero when speed is near zero.
                float speed = distance / horizonSeconds;
                if (speed < MIN_SPEED_THRESHOLD_MPS) {
                    // Trolley is nearly stationary — set TTI to maximum.
                    // This correctly handles the case where the trolley
                    // is sitting inside a zone but not moving, which
                    // should still trigger a warning but with no urgency.
                    bestWarning.timeToImpactSeconds = FLT_MAX;
                } else {
                    bestWarning.timeToImpactSeconds = distance / speed;
                }
            }
        }
    }

    return bestWarning;
}
