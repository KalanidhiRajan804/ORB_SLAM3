#include "PingIntegration.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include <iostream>
#include <cmath>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ORB_SLAM3
{

PingIntegration::PingIntegration(const std::string &strSettingFile)
{
    std::cout << "[ENTER] PingIntegration constructor" << std::endl;
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ);

    if (!fSettings.isOpened())
    {
        std::cerr << "[PingIntegration] ERROR: cannot open settings file: "
                  << strSettingFile << std::endl;
        return;
    }

    std::cout << "[PingIntegration] Loaded YAML: " << strSettingFile << std::endl;

    float hfov_deg = (float)fSettings["Sonar.hfov_deg"];
    float vfov_deg = (float)fSettings["Sonar.vfov_deg"];
    h_fov = hfov_deg * static_cast<float>(CV_PI) / 180.0f;
    v_fov = vfov_deg * static_cast<float>(CV_PI) / 180.0f;

    std::cout << "hfov_deg = " << hfov_deg << std::endl;
    std::cout << "hfov_deg = " << vfov_deg << std::endl;

    // --- Load extrinsics separately ---
    fSettings["Sonar.R_sc"] >> R_sc;
    fSettings["Sonar.t_sc"] >> t_sc;
    if (fSettings["Sonar.R_sc"].empty() || fSettings["Sonar.t_sc"].empty())
        std::cerr << "[PingIntegration] ERROR: Missing Sonar extrinsics in YAML!" << std::endl;

    R_sc.convertTo(R_sc, CV_32F);
    t_sc.convertTo(t_sc, CV_32F);

    std::cout << "[PingIntegration] R_sc = " << R_sc << std::endl;
    std::cout << "[PingIntegration] t_sc = " << t_sc.t() << std::endl;
    std::cout << "[PingIntegration] R_sc loaded size: "
              << R_sc.rows << "x" << R_sc.cols << std::endl;
    std::cout << "[PingIntegration] t_sc loaded size: "
              << t_sc.rows << "x" << t_sc.cols << std::endl;

    // --- Load peak detection tuning ---
    abs_thresh_ = (int)fSettings["Sonar.abs_thresh"];
    peak_prom_  = (int)fSettings["Sonar.peak_prom"];
    band_lo_    = (float)fSettings["Sonar.band_lo"];
    band_hi_    = (float)fSettings["Sonar.band_hi"];
    rangeMin    = (float)fSettings["Sonar.RangeMin"];
    rangeMax    = (float)fSettings["Sonar.RangeMax"];

    std::cout << "[PingIntegration] abs_thresh = " << abs_thresh_ << std::endl;
    std::cout << "[PingIntegration] peak_prom = " << peak_prom_ << std::endl;
    std::cout << "[PingIntegration] band_lo = " << band_lo_ << std::endl;
    std::cout << "[PingIntegration] band_hi = " << band_hi_ << std::endl;

    std::cout << "[PingIntegration] R_sc = " << R_sc << std::endl;
    std::cout << "[PingIntegration] t_sc = " << t_sc.t() << std::endl;
    std::cout << "[PingIntegration] h_fov = " << h_fov << " rad" << std::endl;
    std::cout << "[PingIntegration] v_fov = " << v_fov << " rad" << std::endl;
    std::cout << "[PingIntegration] rangeMin = " << rangeMin << std::endl;
    std::cout << "[PingIntegration] rangeMax = " << rangeMax << std::endl;
    std::cout << "[EXIT] PingIntegration constructor" << std::endl;
}

void PingIntegration::SetPingScan(const float &range,
                                  const float &angle,
                                  const std::vector<uint8_t> &intensities)
{
    std::cout << "Hello from SetPingScan" << std::endl;
    this->pingRange = range;
    this->pingAngle = angle;
    this->pingIntensities = intensities;

    // update sonar data struct inside PingIntegration
    mSonarData.range = range;
    mSonarData.angle = angle;
    mSonarData.intensities = intensities;
    mSonarData.number_of_samples = static_cast<uint16_t>(intensities.size());

    if (intensities.empty())
    {
        this->pingEffectiveDist = -1.0f;
        std::cout << "[PingIntegration] Empty intensity array" << std::endl;
        return;
    }

    // Debug: print max intensity
    auto max_val = *std::max_element(intensities.begin(), intensities.end());
    std::cout << " intensities.size=" << intensities.size()
              << " max=" << static_cast<int>(max_val) << std::endl;

    // Step size (meters/bin)
    float step = (rangeMax - rangeMin) / (intensities.size() - 1);

    // Index range for detection band
    int idx_min = (int)((band_lo_ - rangeMin) / step);
    int idx_max = (int)((band_hi_ - rangeMin) / step);
    idx_min = std::max(0, idx_min);
    idx_max = std::min((int)intensities.size() - 1, idx_max);

    std::cout << "[PingIntegration] band indices = "
              << idx_min << " to " << idx_max
              << " (" << band_lo_ << "–" << band_hi_ << " m)" << std::endl;

    // Loop through band
    int best_idx = -1;
    uint8_t best_val = 0;
    for (int i = idx_min; i <= idx_max; i++)
    {
        uint8_t val = intensities[i];
        if (val < abs_thresh_) continue;
        if (val > best_val)
        {
            best_val = val;
            best_idx = i;
        }
    }

    uint8_t raw_band_max =
        *std::max_element(intensities.begin() + idx_min,
                          intensities.begin() + idx_max + 1);
    std::cout << "[Debug] Band raw max intensity="
              << (int)raw_band_max
              << " (before abs_thresh=" << abs_thresh_ << ")" << std::endl;

    if (best_idx >= 0)
    {
        float peak_r = rangeMin + best_idx * step;
        this->pingEffectiveDist = peak_r;
        std::cout << "[PeakDetect] range=" << peak_r
                  << " intensity=" << (int)best_val << std::endl;
    }
    else
    {
        this->pingEffectiveDist = -1.0f;
        std::cout << "[PeakDetect] No valid peak found" << std::endl;
    }

    std::cout << "Bye from SetPingScan" << std::endl;
}

void PingIntegration::RescaleDepth(Frame &F, const SonarData &sonar)
{
    std::cout << "[PingIntegration::RescaleDepth] frame="
              << F.mnId << " sonar range=" << sonar.range << std::endl;

    if (!ValidateWithIntensity(sonar))
    {
        std::cout << "[RescaleDepth] Invalid sonar data" << std::endl;
        return;
    }
    if (sonar.range <= 0.0f || !std::isfinite(sonar.range))
    {
        std::cout << "[RescaleDepth] Sonar range not usable" << std::endl;
        return;
    }

    Sophus::SE3f Tcw = F.GetPose();
    Eigen::Vector3f camPos = Tcw.translation();

    float slamDepth  = camPos.z();
    float sonarDepth = sonar.range;
    float depthRatio = slamDepth / sonarDepth;

    if (std::fabs(depthRatio - 1.0f) > 0.05f)
    {
        Eigen::Vector3f expected(0, 0, sonarDepth);
        Eigen::Vector3f correction = camPos - expected;
        Eigen::Vector3f newCamPos = camPos - (1.0f - depthRatio) * correction;

        if (!std::isfinite(newCamPos[0]) ||
            !std::isfinite(newCamPos[1]) ||
            !std::isfinite(newCamPos[2]))
        {
            std::cout << "[RescaleDepth] Correction produced NaNs/Infs, skipping" << std::endl;
            return;
        }

        Sophus::SE3f newTcw(Tcw.so3(), newCamPos);
        F.SetPose(newTcw);

        std::cout << "[RescaleDepth] Applied sonar correction"
                  << " ratio=" << depthRatio
                  << " oldZ=" << slamDepth
                  << " sonarZ=" << sonarDepth
                  << " newZ=" << newCamPos.z() << std::endl;
    }
    else
    {
        std::cout << "[RescaleDepth] Depth ratio ~1, no correction applied" << std::endl;
    }
}

void PingIntegration::ProjectRectangle(Frame &F, const SonarData &sonar)
{
    std::cout << "[ProjectRectangle] ENTER (frame=" << F.mnId << ")" << std::endl;
    if (pingEffectiveDist <= 0.0f)
    {
        std::cout << "[ProjectRectangle] EXIT (invalid dist)" << std::endl;
        return;
    }

    float th = sonar.angle;
    cv::Mat v_s = (cv::Mat_<float>(3, 1) << std::sin(th), 0.0f, std::cos(th));
    cv::Mat p_s = pingEffectiveDist * v_s;
    cv::Mat p_c = R_sc * p_s + t_sc;

    if (p_c.at<float>(2) <= 0.0f)
    {
        std::cout << "[ProjectRectangle] EXIT (behind camera)" << std::endl;
        return;
    }

    cv::Mat uvw = F.mK * p_c;
    float u = uvw.at<float>(0) / uvw.at<float>(2);
    float v = uvw.at<float>(1) / uvw.at<float>(2);

    float half_w = (h_fov * uvw.at<float>(2)) * F.fx;
    float half_h = (v_fov * uvw.at<float>(2)) * F.fy;
    float umin = u - half_w;
    float umax = u + half_w;
    float vmin = v - half_h;
    float vmax = v + half_h;

    int matchedIdx = -1;
    for (size_t i = 0; i < F.mvKeysUn.size(); i++)
    {
        const cv::KeyPoint &kp = F.mvKeysUn[i];
        if (kp.pt.x >= umin && kp.pt.x <= umax &&
            kp.pt.y >= vmin && kp.pt.y <= vmax)
        {
            matchedIdx = (int)i;
            break;
        }
    }

    if (matchedIdx >= 0)
    {
        F.mPingProj = cv::Point2f(u, v);
        F.mnPingMatchedIdx = matchedIdx;
        std::cout << "[ProjectRectangle] Found match kp="
                  << matchedIdx << " at pixel=(" << u << "," << v << ")" << std::endl;
    }
    else
    {
        std::cout << "[ProjectRectangle] No feature inside sonar frustum" << std::endl;
    }

    std::cout << "[ProjectRectangle] EXIT" << std::endl;
}

cv::Point3f PingIntegration::TransformPingPointToWorld(Frame &F,
                                                       const SonarData &sonar)
{
    std::cout << "[TransformPingPointToWorld] ENTER (frame=" << F.mnId << ")" << std::endl;
    if (pingEffectiveDist <= 0.0f)
    {
        std::cout << "[TransformPingPointToWorld] EXIT (invalid dist)" << std::endl;
        return cv::Point3f(0, 0, 0);
    }

    float th = sonar.angle;
    cv::Mat v_s = (cv::Mat_<float>(3, 1) << std::sin(th), 0.0f, std::cos(th));
    cv::Mat p_s = pingEffectiveDist * v_s;
    cv::Mat p_c = R_sc * p_s + t_sc;

    Sophus::SE3f Tcw = F.GetPose();
    cv::Mat Rcw(3, 3, CV_32F), tcw(3, 1, CV_32F);
    cv::eigen2cv(Tcw.rotationMatrix().cast<float>(), Rcw);
    cv::eigen2cv(Tcw.translation().cast<float>(), tcw);
    cv::Mat p_w = Rcw.t() * (p_c - tcw);

    std::cout << "[TransformPingPointToWorld] EXIT (world="
              << p_w.at<float>(0) << "," << p_w.at<float>(1)
              << "," << p_w.at<float>(2) << ")" << std::endl;

    return cv::Point3f(p_w.at<float>(0), p_w.at<float>(1), p_w.at<float>(2));
}

float PingIntegration::GetSonarDepthRatio(Frame &F)
{
    std::cout << "[PingIntegration::GetSonarDepthRatio] ENTER frame=" << F.mnId << std::endl;
    if (pingEffectiveDist <= 0.0f || F.mnPingMatchedIdx < 0)
    {
        std::cout << "[PingIntegration::GetSonarDepthRatio] No sonar match" << std::endl;
        return 1.0f;
    }

    MapPoint *pMP = F.mvpMapPoints[F.mnPingMatchedIdx];
    if (!pMP)
    {
        std::cout << "[PingIntegration::GetSonarDepthRatio] No MapPoint for kp idx" << std::endl;
        return 1.0f;
    }

    Sophus::SE3f Tcw = F.GetPose();
    Eigen::Vector3f Xw = pMP->GetWorldPos();
    Eigen::Vector3f Xc = Tcw * Xw;
    float depthSLAM = Xc[2];
    float depthSonar = pingEffectiveDist;

    float ratio = (depthSLAM > 1e-3f) ? (depthSonar / depthSLAM) : 1.0f;
    std::cout << "[PingIntegration::GetSonarDepthRatio] sonar=" << depthSonar
              << " slam=" << depthSLAM << " ratio=" << ratio << std::endl;
    std::cout << "[PingIntegration::GetSonarDepthRatio] EXIT" << std::endl;
    return ratio;
}

cv::Point3f PingIntegration::GetEffectiveSonarPointCamera()
{
    if (pingEffectiveDist <= 0.0f)
    {
        std::cout << "[PingIntegration::GetEffectiveSonarPointCamera] invalid" << std::endl;
        return cv::Point3f(0, 0, 0);
    }

    float th = pingAngle;
    cv::Mat v_s = (cv::Mat_<float>(3, 1) << std::sin(th), 0.0f, std::cos(th));
    cv::Mat p_s = pingEffectiveDist * v_s;
    cv::Mat p_c = R_sc * p_s + t_sc;
    return cv::Point3f(p_c.at<float>(0), p_c.at<float>(1), p_c.at<float>(2));
}

void PingIntegration::RescaleInitialMap(KeyFrame *pKFini,
                                        KeyFrame *pKFcur,
                                        float invMedianDepth)
{
    std::cout << "[RescaleInitialMap] ENTER" << std::endl;
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    for (MapPoint *pMP : pKFini->GetMapPointMatches())
    {
        if (pMP)
        {
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }
    std::cout << "[RescaleInitialMap] EXIT" << std::endl;
}

bool PingIntegration::ValidateWithIntensity(const SonarData &sonar)
{
    std::cout << "[ValidateWithIntensity] ENTER" << std::endl;
    bool valid = (sonar.range > 0.0f) && (!sonar.intensities.empty());
    std::cout << "[ValidateWithIntensity] EXIT ("
              << (valid ? "valid" : "invalid") << ")" << std::endl;
    return valid;
}

} // namespace ORB_SLAM3
