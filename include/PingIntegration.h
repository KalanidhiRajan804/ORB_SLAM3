#ifndef PINGINTEGRATION_H
#define PINGINTEGRATION_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
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
    // --- Lifecycle ---
    PingIntegration(const std::string &strSettingFile);

    // --- Sonar pipeline ---
    void SetPingScan(const float &range,
                     const float &angle,
                     const std::vector<uint8_t> &intensities);

    void RescaleDepth(Frame &F, const SonarData &sonar);
    void ProjectRectangle(Frame &F, const SonarData &sonar);
    cv::Point3f TransformPingPointToWorld(Frame &F, const SonarData &sonar);

    float GetSonarDepthRatio(Frame &F);
    cv::Point3f GetEffectiveSonarPointCamera();

    void RescaleInitialMap(KeyFrame* pKFini,
                           KeyFrame* pKFcur,
                           float invMedianDepth);

    bool ValidateWithIntensity(const SonarData &sonar);

    bool IsPointInsideFrustum(const Eigen::Vector3f &Xc,
                       const Eigen::Vector3f &beam_c,
                       float maxDist) const;


    // --- Geometry helpers ---
    Eigen::Vector3f BeamVector(const SonarData &sonar);
    std::vector<Eigen::Vector3f> BeamRectangle(const Eigen::Vector3f &v, float rho) const;
    // void DebugBeam();

    // --- Extrinsics (loaded from YAML) ---
    cv::Mat R_sc;   // rotation (3x3)
    cv::Mat t_sc;   // translation (3x1)

    // --- Intrinsics / sonar parameters ---
    float h_fov;       // horizontal FOV (rad, half-angle)
    float v_fov;       // vertical FOV (rad, half-angle)
    float rangeMin;
    float rangeMax;

    // --- Step conversion (cached from YAML) ---
    float forward_step;
    float step_to_deg;

    // --- Peak detection parameters (from YAML) ---
    int abs_thresh_;
    int peak_prom_;
    float band_lo_;
    float band_hi_;

    // --- State ---
    float pingRange = -1.0f;          // raw input
    float pingAngle = 0.0f;           // raw input
    float pingEffectiveDist = -1.0f;  // detected peak range
    std::vector<uint8_t> pingIntensities;

    SonarData mSonarData;   // unified sonar struct
};

} // namespace ORB_SLAM3

#endif // PINGINTEGRATION_H
