#ifndef SONARDATA_H
#define SONARDATA_H

#include <vector>
#include <cstdint>

namespace ORB_SLAM3
{

struct SonarData {
    float angle = -1.0f;                // beam angle in radians
    uint8_t gain = 0;                   // sonar gain
    uint16_t number_of_samples = 0;     // number of bins in this ping
    uint16_t transmit_frequency = 0;    // sonar frequency
    uint16_t speed_of_sound = 0;        // environment parameter
    float range = -1.0f;                // reported range (m)
    std::vector<uint8_t> intensities;   // raw intensity bins
};

} // namespace ORB_SLAM3

#endif // SONARDATA_H
