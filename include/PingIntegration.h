#ifndef PINGINTEGRATION_H
#define PINGINTEGRATION_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include "SonarData.h"

#define SAFE_SONAR_DIST(d) (std::isfinite(d) && (d) > 0.0f)


namespace ORB_SLAM3
{

class Frame;
class KeyFrame;
class MapPoint;

class PingIntegration
{
public:
    PingIntegration(const std::string &strSettingFile);

    void SetPingScan(const float &range,
                     const float &angle,
                     const std::vector<uint8_t> &intensities);

    void RescaleDepth(Frame &F, const SonarData &sonar);
    void ProjectRectangle(Frame &F, const SonarData &sonar);
    cv::Point3f TransformPingPointToWorld(Frame &F, const SonarData &sonar);

    float GetSonarDepthRatio(Frame &F);
    cv::Point3f GetEffectiveSonarPointCamera();

    void RescaleInitialMap(KeyFrame* pKFini, KeyFrame* pKFcur, float invMedianDepth);
    bool ValidateWithIntensity(const SonarData &sonar);

    // --- Extrinsics ---
    cv::Mat R_sc;   // rotation (3x3)
    cv::Mat t_sc;   // translation (3x1)

    // --- Intrinsics ---
    float h_fov, v_fov;     // radians
    float rangeMin, rangeMax;

    // --- State ---
    float pingRange = -1.0f;
    float pingAngle = 0.0f;
    float pingEffectiveDist = -1.0f;
    std::vector<uint8_t> pingIntensities;

    // Peak detection parameters (from YAML)
    int abs_thresh_;
    int peak_prom_;
    float band_lo_;
    float band_hi_;


    SonarData mSonarData;   // unified sonar struct
};

} // namespace ORB_SLAM3

#endif // PINGINTEGRATION_H
