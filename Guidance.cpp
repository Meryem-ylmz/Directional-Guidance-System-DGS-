#include <cmath>
#include "Guidance.h"

double rad2deg(double r) {
    return r * 180.0 / M_PI;
}
double deg2rad(double d) {
    return d * M_PI / 180.0;
}

double computeAzimuth(const Position& drone, const Position& target) {
    double dN = target.N - drone.N;
    double dE = target.E - drone.E;

    double rad = atan2(dE, dN);
    double deg = rad2deg(rad);

    if (deg < 0)
        deg += 360.0;

    return deg;
}

double shortestAngleDiff(double targetDeg, double currentDeg) {
    double diff = targetDeg - currentDeg;

    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    return diff;
}

double computeTurnCommand(double headingErrorDeg) {
    double e = headingErrorDeg;
    double sing = (e >= 0) ? 1.0 : -1.0;
    double mag = fabs(e);
    double step;

    if (mag < 2.0)
        step = 0.0;
    else if (mag < 10.0)
        step = 5.0;
    else if (mag < 25.0)
        step = 10.0;
    else
        step = 20.0;

        return sing * step;
}

double computeDistance(const Position& drone, const Position& target)
{
    double dN = target.N - drone.N;
    double dE = target.E - drone.E;
    return sqrt(dN*dN + dE*dE);
}

double computeDesiredSpeed(double distance, double headingErrorDeg)
{
    double baseSpeed;

    if (distance > 50)
        baseSpeed = 5.0;
    else if (distance > 20)
        baseSpeed = 3.0;
    else if (distance > 5)
        baseSpeed = 1.5;
    else
        baseSpeed = 0.5;

    if (fabs(headingErrorDeg) > 40.0)
        baseSpeed *= 0.5;

    return baseSpeed;
}

MissionCommand updateGuidance(const Position& drone,
                              const Position& target,
                              double currentYawDeg)
{
    MissionCommand cmd;

    double azimuth = computeAzimuth(drone, target);
    double headingError = shortestAngleDiff(azimuth, currentYawDeg);
    double distance = computeDistance(drone, target);

    cmd.deltaYawDeg = computeTurnCommand(headingError);
    cmd.forwardSpeed = computeDesiredSpeed(distance, headingError);

    return cmd;
}