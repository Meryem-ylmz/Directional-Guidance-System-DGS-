#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "Position.h"
#include "MissionCommand.h"

double rad2deg(double r);
double deg2rad(double d);

double computeAzimuth(const Position& drone, const Position& target);
double shortestAngleDiff(double targetDeg, double currentDeg);
double computeTurnCommand(double headingErrorDeg);
double computeDistance(const Position& drone, const Position& target);
double computeDesiredSpeed(double distance, double headingErrorDeg);

MissionCommand updateGuidance(const Position& drone,
                            const Position& target,
                            double currentYawDeg);

#endif