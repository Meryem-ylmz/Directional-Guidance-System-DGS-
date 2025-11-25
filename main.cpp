#include <iostream>
#include "Guidance.h"

using namespace std;

int main() {
    Position drone = {0.0, 0.0};
    Position target = {40.0, 20.0};
    double currentYaw = 90.0;

    MissionCommand cmd = updateGuidance(drone, target, currentYaw);

    cout << "Turn Command (deg): " << cmd.deltaYawDeg << endl;
    cout << "Forward Speed (m/s): " << cmd.forwardSpeed << endl;

    return 0;
}
