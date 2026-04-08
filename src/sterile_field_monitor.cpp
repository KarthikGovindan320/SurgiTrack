#include "sterile_field_monitor.hpp"
#include <cmath>
#include <cfloat>
#include <iostream>

constexpr float MIN_SPEED_THRESHOLD_MPS = 0.01f;

void SterileFieldMonitor::addSterileZone(const SterileZone& zone) {
    zones_.push_back(zone);
    std::cout << "[SterileFieldMonitor] Added zone: " << zone.name
              << " at (" << zone.centre.x << ", " << zone.centre.y
              << ", " << zone.centre.z << ") radius=" << zone.radius << "m\n";
}

// Returns the smallest t in [0, horizonSeconds] at which the line segment
// currentPos + t*velocity enters sphere(centre, radius), or FLT_MAX if none.
static float segmentSphereIntersect(const cv::Point3f& origin,
                                    const cv::Point3f& vel,
                                    const cv::Point3f& centre,
                                    float radius,
                                    float horizonSeconds) {
    // Parametric ray: P(t) = origin + t*vel, t in [0, horizonSeconds].
    // Solve |P(t) - centre|^2 = radius^2 for t.
    float ox = origin.x - centre.x;
    float oy = origin.y - centre.y;
    float oz = origin.z - centre.z;

    float a = vel.x*vel.x + vel.y*vel.y + vel.z*vel.z;
    float b = 2.0f * (ox*vel.x + oy*vel.y + oz*vel.z);
    float c = ox*ox + oy*oy + oz*oz - radius*radius;

    // Already inside the zone.
    if (c < 0.0f) return 0.0f;

    // Not moving — no future intersection.
    if (a < 1e-12f) return FLT_MAX;

    float disc = b*b - 4.0f*a*c;
    if (disc < 0.0f) return FLT_MAX;   // ray misses sphere entirely

    float sqrtDisc = std::sqrt(disc);
    float t1 = (-b - sqrtDisc) / (2.0f * a);
    float t2 = (-b + sqrtDisc) / (2.0f * a);

    // Pick the smallest positive t within the horizon.
    for (float t : {t1, t2}) {
        if (t >= 0.0f && t <= horizonSeconds) return t;
    }
    return FLT_MAX;
}

FieldBreachAlert SterileFieldMonitor::check(const cv::Point3f& currentPos,
                                            const cv::Point3f& velocity,
                                            const cv::Point3f& predictedPos,
                                            float horizonSeconds) const {
    FieldBreachAlert bestWarning;
    bestWarning.active              = false;
    bestWarning.zoneName            = "";
    bestWarning.timeToImpactSeconds = FLT_MAX;
    bestWarning.distanceMetres      = FLT_MAX;

    float speed = std::sqrt(velocity.x*velocity.x +
                            velocity.y*velocity.y +
                            velocity.z*velocity.z);

    float earliestImpact = FLT_MAX;

    for (const auto& zone : zones_) {
        // Distance from the *predicted* position to the zone centre.
        float dx = predictedPos.x - zone.centre.x;
        float dy = predictedPos.y - zone.centre.y;
        float dz = predictedPos.z - zone.centre.z;
        float distAtHorizon = std::sqrt(dx*dx + dy*dy + dz*dz);

        // Test the full trajectory segment for zone entry.
        float tImpact = segmentSphereIntersect(
            currentPos, velocity, zone.centre, zone.radius, horizonSeconds);

        bool trajectoryBreaches = (tImpact < FLT_MAX);
        bool alreadyInside      = (distAtHorizon < zone.radius);

        if (!trajectoryBreaches && !alreadyInside) continue;

        // Compute time-to-impact from actual speed.
        float tti;
        if (alreadyInside && !trajectoryBreaches) {
            // Currently inside — already breached.
            tti = 0.0f;
        } else if (speed < MIN_SPEED_THRESHOLD_MPS) {
            tti = FLT_MAX;   // essentially stationary
        } else {
            tti = tImpact;   // seconds until boundary crossing
        }

        // Keep the zone whose breach happens soonest.
        if (tti < earliestImpact) {
            earliestImpact = tti;

            bestWarning.active              = true;
            bestWarning.zoneName            = zone.name;
            bestWarning.distanceMetres      = distAtHorizon;
            bestWarning.timeToImpactSeconds = tti;
        }
    }

    return bestWarning;
}