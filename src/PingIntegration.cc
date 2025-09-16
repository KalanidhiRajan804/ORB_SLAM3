#include "PingIntegration.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include <iostream>
#include <cmath>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>          
#include <cmath> 


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

    // --- Load sonar FOVs ---
    float hfov_deg = (float)fSettings["Sonar.hfov_deg"];
    float vfov_deg = (float)fSettings["Sonar.vfov_deg"];
    h_fov = hfov_deg * static_cast<float>(CV_PI) / 180.0f;
    v_fov = vfov_deg * static_cast<float>(CV_PI) / 180.0f;

    std::cout << "hfov_deg = " << hfov_deg << std::endl;
    std::cout << "vfov_deg = " << vfov_deg << std::endl;

    // --- Load extrinsics ---
    fSettings["Sonar.R_sc"] >> R_sc;
    fSettings["Sonar.t_sc"] >> t_sc;
    if (fSettings["Sonar.R_sc"].empty() || fSettings["Sonar.t_sc"].empty())
        std::cerr << "[PingIntegration] ERROR: Missing Sonar extrinsics in YAML!" << std::endl;

    R_sc.convertTo(R_sc, CV_32F);
    t_sc.convertTo(t_sc, CV_32F);

    std::cout << "[PingIntegration] R_sc = " << R_sc << std::endl;
    std::cout << "[PingIntegration] t_sc = " << t_sc.t() << std::endl;
    std::cout << "[PingIntegration] R_sc size: " << R_sc.rows << "x" << R_sc.cols << std::endl;
    std::cout << "[PingIntegration] t_sc size: " << t_sc.rows << "x" << t_sc.cols << std::endl;

    // --- Load peak detection tuning ---
    abs_thresh_ = (int)fSettings["Sonar.abs_thresh"];
    peak_prom_  = (int)fSettings["Sonar.peak_prom"];
    band_lo_    = (float)fSettings["Sonar.band_lo"];
    band_hi_    = (float)fSettings["Sonar.band_hi"];
    rangeMin    = (float)fSettings["Sonar.RangeMin"];
    rangeMax    = (float)fSettings["Sonar.RangeMax"];
    forward_step = (float)fSettings["Sonar.ForwardStep"];
    step_to_deg  = (float)fSettings["Sonar.SteptoDeg"];

    std::cout << "[PingIntegration] abs_thresh = " << abs_thresh_ << std::endl;
    std::cout << "[PingIntegration] peak_prom = " << peak_prom_ << std::endl;
    std::cout << "[PingIntegration] band_lo = " << band_lo_ << std::endl;
    std::cout << "[PingIntegration] band_hi = " << band_hi_ << std::endl;
    std::cout << "[PingIntegration] h_fov = " << h_fov << " rad" << std::endl;
    std::cout << "[PingIntegration] v_fov = " << v_fov << " rad" << std::endl;
    std::cout << "[PingIntegration] rangeMin = " << rangeMin << std::endl;
    std::cout << "[PingIntegration] rangeMax = " << rangeMax << std::endl;
    std::cout << "[PingIntegration] forward_step = " << forward_step << std::endl;
    std::cout << "[PingIntegration] step_to_deg = " << step_to_deg << std::endl;
    

    std::cout << "[EXIT] PingIntegration constructor" << std::endl;
}


void PingIntegration::SetPingScan(const float &range,
                                  const float &angle,
                                  const std::vector<uint8_t> &intensities)
{
    std::cout << "Hello from SetPingScan" << std::endl;

    // Store raw values
    this->pingRange = range;
    this->pingAngle = angle;
    this->pingIntensities = intensities;

    // Update sonar data struct
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

    // Debug: max intensity
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

    // Loop through band → find strongest peak above threshold
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

    // Debug: raw max in band
    uint8_t raw_band_max =
        *std::max_element(intensities.begin() + idx_min,
                          intensities.begin() + idx_max + 1);
    std::cout << "[Debug] Band raw max intensity="
              << (int)raw_band_max
              << " (before abs_thresh=" << abs_thresh_ << ")" << std::endl;

    // Result
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


// void PingIntegration::DebugBeam()
// {
//     if (pingEffectiveDist <= 0.0f)
//     {
//         std::cout << "[DebugBeam] No valid sonar distance yet" << std::endl;
//         return;
//     }

//     // 1. Beam vector from current sonar data
//     Eigen::Vector3f v_s = BeamVector(mSonarData);

//     // 2. Rectangle corners at the detected range
//     auto corners = BeamRectangle(v_s, pingEffectiveDist);

//     // 3. Print them
//     std::cout << "[DebugBeam] sonar.angle=" << mSonarData.angle
//               << " pingEffectiveDist=" << pingEffectiveDist << std::endl;

//     for (size_t i = 0; i < corners.size(); i++)
//         std::cout << "[DebugBeam] Corner " << i << ": "
//                   << corners[i].transpose() << std::endl;
// }




Eigen::Vector3f PingIntegration::BeamVector(const SonarData &sonar)
{
    // Step index from sonar packet (Ping360)
    float s = static_cast<float>(sonar.angle);

    // Convert step index -> degrees -> radians
    float ang_deg = (s - forward_step) * step_to_deg;     // degrees
    float th = ang_deg * static_cast<float>(CV_PI) / 180.0f;  // radians

    // Build beam vector (XZ-plane sweep, CCW positive)
    Eigen::Vector3f v_s(std::sin(th), 0.0f, std::cos(th));

    // Debug print
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "[BeamVector] step=" << s
              << " ang_deg=" << ang_deg
              << " th(rad)=" << th
              << " vec=[" << v_s.x() << " "
                          << v_s.y() << " "
                          << v_s.z() << "]" << std::endl;

    return v_s;
}


std::vector<Eigen::Vector3f> PingIntegration::BeamRectangle(const Eigen::Vector3f &v, float rho) const
{
    // Normalize the input beam vector
    Eigen::Vector3f v_norm = v.normalized();

    // Pick any vector not parallel to v
    Eigen::Vector3f tmp = (std::abs(v_norm.y()) < 0.9f)
                            ? Eigen::Vector3f(0, 1, 0)
                            : Eigen::Vector3f(1, 0, 0);

    // Orthogonal basis around v
    Eigen::Vector3f u = v_norm.cross(tmp).normalized();  // horizontal axis
    Eigen::Vector3f w = v_norm.cross(u).normalized();    // vertical axis

    // Corners
    std::vector<Eigen::Vector3f> corners;
    corners.reserve(4);

    for (int sign_h : {-1, 1})
    {
        for (int sign_v : {-1, 1})
        {
            Eigen::Vector3f dir_vec =
                std::cos(h_fov) * std::cos(v_fov) * v_norm +
                std::sin(h_fov * sign_h) * u +
                std::sin(v_fov * sign_v) * w;

            dir_vec.normalize();
            corners.push_back(rho * dir_vec);
        }
    }

    // Debug print
    for (size_t i = 0; i < corners.size(); ++i)
        std::cout << "[BeamRectangle] Corner " << i << ": "
                  << corners[i].transpose() << std::endl;

    return corners;
}



void PingIntegration::ProjectRectangle(Frame &F, const SonarData &sonar)
{
    if (pingEffectiveDist <= 0.0f)
    {
        std::cout << "[ProjectRectangle] EXIT (invalid dist)" << std::endl;
        return;
    }

    // 1. Get beam unit vector
    Eigen::Vector3f v_s = BeamVector(sonar);

    // 2. Compute 4 rectangle corners in sonar frame
    auto corners_sonar = BeamRectangle(v_s, pingEffectiveDist);

    std::cout << "[ProjectRectangle] --- Corners in sonar frame ---" << std::endl;
    for (size_t i = 0; i < corners_sonar.size(); ++i)
        std::cout << " sonar[" << i << "] = " << corners_sonar[i].transpose() << std::endl;

    // 3. Transform to camera frame using R_sc, t_sc
    std::vector<cv::Point3f> corners_cam;
    corners_cam.reserve(corners_sonar.size());
    for (const auto &c : corners_sonar)
    {
        cv::Mat p_s = (cv::Mat_<float>(3,1) << c.x(), c.y(), c.z());
        cv::Mat p_c = R_sc * p_s + t_sc;  // 3x1

        corners_cam.emplace_back(
            p_c.at<float>(0),
            p_c.at<float>(1),
            p_c.at<float>(2));
    }

    std::cout << "[ProjectRectangle] --- Corners in camera frame ---" << std::endl;
    for (size_t i = 0; i < corners_cam.size(); ++i)
        std::cout << " cam[" << i << "] = "
                  << corners_cam[i].x << ", "
                  << corners_cam[i].y << ", "
                  << corners_cam[i].z << std::endl;

    // 4. Project to image plane using camera intrinsics from Frame
    std::vector<cv::Point2f> img_points;
    {
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);

        cv::Mat K = (cv::Mat_<float>(3,3) <<
                     F.fx,    0, F.cx,
                       0,  F.fy, F.cy,
                       0,    0,   1);

        cv::projectPoints(corners_cam, rvec, tvec, K, F.mDistCoef, img_points);
    }

    std::cout << "[ProjectRectangle] --- Projected pixels ---" << std::endl;
    for (size_t i = 0; i < img_points.size(); ++i)
        std::cout << " px[" << i << "] = "
                  << img_points[i].x << ", "
                  << img_points[i].y << std::endl;

    // (Optional) Compute polygon area in pixel²
    // if (img_points.size() >= 3)
    // {
    //     double area = 0.0;
    //     for (size_t i = 0; i < img_points.size(); ++i)
    //     {
    //         const cv::Point2f &p1 = img_points[i];
    //         const cv::Point2f &p2 = img_points[(i+1) % img_points.size()];
    //         area += (p1.x * p2.y - p2.x * p1.y);
    //     }
    //     area = std::fabs(area) * 0.5;
    //     std::cout << "[ProjectRectangle] polygon area = " << area << " pixel^2" << std::endl;
    // }

    std::cout << "[ProjectRectangle] EXIT" << std::endl;
}

void PingIntegration::RescaleDepth(Frame &F, const SonarData &sonar)
{
    if (pingEffectiveDist <= 0.0f) return;

    float ratio = GetSonarDepthRatio(F);
    if (std::fabs(ratio - 1.0f) < 0.05f) {
        std::cout << "[RescaleDepth] Ratio ~1, skip" << std::endl;
        return;
    }

    Sophus::SE3f Tcw = F.GetPose();
    Eigen::Vector3f t = Tcw.translation();

    // Beam vector in camera frame
    Eigen::Vector3f v_s = BeamVector(sonar);
    Eigen::Matrix3f R_eig; cv::cv2eigen(R_sc, R_eig);
    Eigen::Vector3f t_eig; cv::cv2eigen(t_sc, t_eig);
    Eigen::Vector3f beam_c = (R_eig * v_s).normalized();

    // Move camera along beam direction
    Eigen::Vector3f new_t = t + (1.0f - ratio) * beam_c * pingEffectiveDist;

    Sophus::SE3f newTcw(Tcw.so3(), new_t);
    F.SetPose(newTcw);

    std::cout << "[RescaleDepth] ratio=" << ratio
              << " old_t=" << t.transpose()
              << " new_t=" << new_t.transpose() << std::endl;
}


bool PingIntegration::ValidateWithIntensity(const SonarData &sonar)
{
    std::cout << "[ValidateWithIntensity] ENTER" << std::endl;
    bool valid = (sonar.range > 0.0f) && (!sonar.intensities.empty());
    std::cout << "[ValidateWithIntensity] EXIT (" 
              << (valid ? "valid" : "invalid") << ")" << std::endl;
    return valid;
}


cv::Point3f PingIntegration::TransformPingPointToWorld(Frame &F, const SonarData &sonar)
{
    if (pingEffectiveDist <= 0.0f)
    {
        std::cout << "[TransformPingPointToWorld] EXIT (invalid dist)" << std::endl;
        return cv::Point3f(0,0,0);
    }

    Eigen::Vector3f v_s = BeamVector(sonar);
    Eigen::Vector3f p_s = pingEffectiveDist * v_s;

    cv::Mat p_c = R_sc * (cv::Mat_<float>(3,1) << p_s.x(), p_s.y(), p_s.z()) + t_sc;

    Sophus::SE3f Tcw = F.GetPose();
    cv::Mat Rcw(3,3,CV_32F), tcw(3,1,CV_32F);
    cv::eigen2cv(Tcw.rotationMatrix().cast<float>(), Rcw);
    cv::eigen2cv(Tcw.translation().cast<float>(), tcw);

    cv::Mat p_w = Rcw.t() * (p_c - tcw);
    return cv::Point3f(p_w.at<float>(0), p_w.at<float>(1), p_w.at<float>(2));
}

float PingIntegration::GetSonarDepthRatio(Frame &F)
{
    if (pingEffectiveDist <= 0.0f) return 1.0f;

    // Beam vector in camera frame
    Eigen::Vector3f v_s = BeamVector(mSonarData);
    Eigen::Matrix3f R_eig; cv::cv2eigen(R_sc, R_eig);
    Eigen::Vector3f t_eig; cv::cv2eigen(t_sc, t_eig);
    Eigen::Vector3f beam_c = R_eig * v_s + t_eig;

    // Search map points
    for (int i = 0; i < F.N; i++)
    {
        MapPoint *pMP = F.mvpMapPoints[i];
        if (!pMP || F.mvbOutlier[i]) continue;

        // Transform to camera frame
        Sophus::SE3f Tcw = F.GetPose();
        Eigen::Vector3f Xw = pMP->GetWorldPos();
        Eigen::Vector3f Xc = Tcw * Xw;

        if (IsPointInsideFrustum(Xc, beam_c, pingEffectiveDist * 1.1f))
        {
            float slamDepth  = Xc[2];
            float sonarDepth = pingEffectiveDist;
            if (slamDepth > 1e-3f)
            {
                float ratio = sonarDepth / slamDepth;
                std::cout << "[DepthRatio] matched MP depth=" << slamDepth
                          << " sonar=" << sonarDepth
                          << " ratio=" << ratio << std::endl;
                return ratio;
            }
        }
    }

    std::cout << "[DepthRatio] no MapPoint inside " << std::endl;
    return 1.0f;
}


void PingIntegration::RescaleInitialMap(KeyFrame *pKFini, KeyFrame *pKFcur, float invMedianDepth)
{
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
}

bool PingIntegration::IsPointInsideFrustum(const Eigen::Vector3f &Xc,
                                        const Eigen::Vector3f &beamVec,
                                        float maxRange) const
{
    if (Xc.norm() <= 1e-6f) return false;

    Eigen::Vector3f v = Xc.normalized();
    float cosAngle = v.dot(beamVec.normalized());

    // Accept if within beam half-FOV and within sonar range
    bool inside = (cosAngle > std::cos(h_fov)) && (Xc.norm() <= maxRange);
    return inside;
}






// inline std::vector<Eigen::Vector3f> beam_rectangle(
//     const Eigen::Vector3f &v_s,
//     float rho,
//     float h_fov,
//     float v_fov)
// {
//     Eigen::Vector3f v = v_s.normalized();
//     Eigen::Vector3f tmp = (std::abs(v.y()) < 0.9f)
//                             ? Eigen::Vector3f(0, 1, 0)
//                             : Eigen::Vector3f(1, 0, 0);

//     Eigen::Vector3f u = v.cross(tmp).normalized();
//     Eigen::Vector3f w = v.cross(u).normalized();

//     std::vector<Eigen::Vector3f> corners;
//     corners.reserve(4);

//     for (int sign_h : {-1, 1})
//     {
//         for (int sign_v : {-1, 1})
//         {
//             Eigen::Vector3f dir_vec =
//                 std::cos(h_fov) * std::cos(v_fov) * v +
//                 std::sin(h_fov * sign_h) * u +
//                 std::sin(v_fov * sign_v) * w;

//             dir_vec.normalize();
//             corners.push_back(rho * dir_vec);
//         }
//     }
//     return corners;
// }


// inline Eigen::Vector3f sonar_unit_vector(float angle_rad)
// {
//     return Eigen::Vector3f(std::sin(angle_rad), 0.0f, std::cos(angle_rad));
// }




// void PingIntegration::ProjectRectangle(Frame &F, const SonarData &sonar)
// {
//     std::cout << "[ProjectRectangle] ENTER (frame=" << F.mnId << ")" << std::endl;
//     if (pingEffectiveDist <= 0.0f)
//     {
//         std::cout << "[ProjectRectangle] EXIT (invalid dist)" << std::endl;
//         return;
//     }

//     std::cout << "[ProjectRectangle] pingEffectiveDist = "
//               << pingEffectiveDist << std::endl;


//     float s = static_cast<float>(sonar.angle);
//     float forward_step = (float)fSettings["Sonar.forward_step"];
//     float step_to_deg  = (float)fSettings["Sonar.step_to_deg"];


//     float ang_deg = (s - forward_step) * step_to_deg;
//     float th = ang_deg * static_cast<float>(CV_PI) / 180.0f; 
    
//     std::cout << "[ProjectRectangle] angle=" << th
//               << " dist=" << pingEffectiveDist << std::endl;
//     cv::Mat v_s = (cv::Mat_<float>(3, 1) << std::sin(th), 0.0f, std::cos(th));

//     std::vector<Eigen::Vector3f> beam_rectangle (v_s, pingEffectiveDist, h_fov, v_fov);{

//         Eigen::Vector3f v = v_s.normalized();

//     // pick any vector not parallel to v
//     Eigen::Vector3f tmp = (std::abs(v.y()) < 0.9f)
//                             ? Eigen::Vector3f(0, 1, 0)
//                             : Eigen::Vector3f(1, 0, 0);

//     // orthogonal basis around v
//     Eigen::Vector3f u = v.cross(tmp).normalized();  // horizontal axis
//     Eigen::Vector3f w = v.cross(u).normalized();    // vertical axis

//     float dh = h_fov;
//     float dv = v_fov;

//     std::vector<Eigen::Vector3f> corners;
//     corners.reserve(4);

//     for (int sign_h : {-1, 1})
//     {
//         for (int sign_v : {-1, 1})
//         {
//             Eigen::Vector3f dir_vec =
//                 std::cos(dh) * std::cos(dv) * v +
//                 std::sin(dh * sign_h) * u +
//                 std::sin(dv * sign_v) * w;

//             dir_vec.normalize();
//             corners.push_back(rho * dir_vec);
//         }
//     }

//     return corners;

//     for (size_t i = 0; i < corners.size(); ++i)
//     std::cout << "Corner " << i << ": " << corners[i].transpose() << std::endl;
//     }









//     cv::Mat p_s = pingEffectiveDist * v_s;
//     std::cout << "[ProjectRectangle] p_s = " << p_s.t() << std::endl;
//     cv::Mat p_c = R_sc * p_s + t_sc;
//     std::cout << "[ProjectRectangle] p_c = " << p_c.t() << std::endl;

//     if (p_c.at<float>(2) <= 0.0f)
//     {
//         std::cout << "[ProjectRectangle] EXIT (behind camera)" << std::endl;
//         return;
//     }

//     // Sonar point in camera frame
//     std::cout << "[ProjectRectangle] p_c = " << p_c.t() << std::endl;

//     // Project to image plane
//     cv::Mat uvw = F.mK * p_c;
//     std::cout << "[ProjectRectangle] uvw = " << uvw.t() << std::endl;
//     std::cout << "[ProjectRectangle] fx=" << F.fx
//             << " fy=" << F.fy
//             << " cx=" << F.cx
//             << " cy=" << F.cy << std::endl;

//     float u = uvw.at<float>(0) / uvw.at<float>(2);
//     float v = uvw.at<float>(1) / uvw.at<float>(2);
//     std::cout << "[ProjectRectangle] projected center u=" << u
//             << " v=" << v
//             << " depth=" << uvw.at<float>(2) << std::endl;

//     float half_w = (h_fov * uvw.at<float>(2)) * F.fx;
//     float half_h = (v_fov * uvw.at<float>(2)) * F.fy;
//     std::cout << "[ProjectRectangle] half_w=" << half_w
//             << " half_h=" << half_h << std::endl;

//     float umin = u - half_w;
//     float umax = u + half_w;
//     float vmin = v - half_h;
//     float vmax = v + half_h;
//     std::cout << "[ProjectRectangle] bbox: u=[" << umin << "," << umax
//             << "] v=[" << vmin << "," << vmax << "]" << std::endl;


//     int matchedIdx = -1;
//     for (size_t i = 0; i < F.mvKeysUn.size(); i++)
//     {
//         const cv::KeyPoint &kp = F.mvKeysUn[i];
//         if (kp.pt.x >= umin && kp.pt.x <= umax &&
//             kp.pt.y >= vmin && kp.pt.y <= vmax)
//         {
//             matchedIdx = (int)i;
//             break;
//         }
//     }

//     if (matchedIdx >= 0)
//     {
//         F.mPingProj = cv::Point2f(u, v);
//         F.mnPingMatchedIdx = matchedIdx;
//         std::cout << "[ProjectRectangle] Found match kp="
//                   << matchedIdx << " at pixel=(" << u << "," << v << ")" << std::endl;
//     }
//     else
//     {
//         std::cout << "[ProjectRectangle] No feature inside sonar frustum" << std::endl;
//     }

//     std::cout << "[ProjectRectangle] EXIT" << std::endl;
// }







// void PingIntegration::RescaleDepth(Frame &F, const SonarData &sonar)
// {


//     std::cout << "[PingIntegration::RescaleDepth] frame="
//               << F.mnId << " sonar range=" << sonar.range << std::endl;

//     if (!ValidateWithIntensity(sonar))
//     {
//         std::cout << "[RescaleDepth] Invalid sonar data" << std::endl;
//         return;
//     }
//     if (sonar.range <= 0.0f || !std::isfinite(sonar.range))
//     {
//         std::cout << "[RescaleDepth] Sonar range not usable" << std::endl;
//         return;
//     }

//     Sophus::SE3f Tcw = F.GetPose();
//     Eigen::Vector3f camPos = Tcw.translation();

//     float slamDepth  = camPos.z();
//     float sonarDepth = sonar.range;
//     float depthRatio = slamDepth / sonarDepth;

//     if (std::fabs(depthRatio - 1.0f) > 0.05f)
//     {
//         Eigen::Vector3f expected(0, 0, sonarDepth);
//         Eigen::Vector3f correction = camPos - expected;
//         Eigen::Vector3f newCamPos = camPos - (1.0f - depthRatio) * correction;

//         if (!std::isfinite(newCamPos[0]) ||
//             !std::isfinite(newCamPos[1]) ||
//             !std::isfinite(newCamPos[2]))
//         {
//             std::cout << "[RescaleDepth] Correction produced NaNs/Infs, skipping" << std::endl;
//             return;
//         }

//         Sophus::SE3f newTcw(Tcw.so3(), newCamPos);
//         F.SetPose(newTcw);

//         std::cout << "[RescaleDepth] Applied sonar correction"
//                   << " ratio=" << depthRatio
//                   << " oldZ=" << slamDepth
//                   << " sonarZ=" << sonarDepth
//                   << " newZ=" << newCamPos.z() << std::endl;
//     }
//     else
//     {
//         std::cout << "[RescaleDepth] Depth ratio ~1, no correction applied" << std::endl;
//     }
// }



// cv::Point3f PingIntegration::TransformPingPointToWorld(Frame &F,
//                                                        const SonarData &sonar)
// {
//     std::cout << "[TransformPingPointToWorld] ENTER (frame=" << F.mnId << ")" << std::endl;
//     if (pingEffectiveDist <= 0.0f)
//     {
//         std::cout << "[TransformPingPointToWorld] EXIT (invalid dist)" << std::endl;
//         return cv::Point3f(0, 0, 0);
//     }

//     float th = sonar.angle;
//     cv::Mat v_s = (cv::Mat_<float>(3, 1) << std::sin(th), 0.0f, std::cos(th));
//     cv::Mat p_s = pingEffectiveDist * v_s;
//     cv::Mat p_c = R_sc * p_s + t_sc;

//     Sophus::SE3f Tcw = F.GetPose();
//     cv::Mat Rcw(3, 3, CV_32F), tcw(3, 1, CV_32F);
//     cv::eigen2cv(Tcw.rotationMatrix().cast<float>(), Rcw);
//     cv::eigen2cv(Tcw.translation().cast<float>(), tcw);
//     cv::Mat p_w = Rcw.t() * (p_c - tcw);

//     std::cout << "[TransformPingPointToWorld] EXIT (world="
//               << p_w.at<float>(0) << "," << p_w.at<float>(1)
//               << "," << p_w.at<float>(2) << ")" << std::endl;

//     return cv::Point3f(p_w.at<float>(0), p_w.at<float>(1), p_w.at<float>(2));
// }

// float PingIntegration::GetSonarDepthRatio(Frame &F)
// {
//     std::cout << "[PingIntegration::GetSonarDepthRatio] ENTER frame=" << F.mnId << std::endl;
//     if (pingEffectiveDist <= 0.0f || F.mnPingMatchedIdx < 0)
//     {
//         std::cout << "[PingIntegration::GetSonarDepthRatio] No sonar match" << std::endl;
//         return 1.0f;
//     }

//     MapPoint *pMP = F.mvpMapPoints[F.mnPingMatchedIdx];
//     if (!pMP)
//     {
//         std::cout << "[PingIntegration::GetSonarDepthRatio] No MapPoint for kp idx" << std::endl;
//         return 1.0f;
//     }

//     Sophus::SE3f Tcw = F.GetPose();
//     Eigen::Vector3f Xw = pMP->GetWorldPos();
//     Eigen::Vector3f Xc = Tcw * Xw;
//     float depthSLAM = Xc[2];
//     float depthSonar = pingEffectiveDist;

//     float ratio = (depthSLAM > 1e-3f) ? (depthSonar / depthSLAM) : 1.0f;
//     std::cout << "[PingIntegration::GetSonarDepthRatio] sonar=" << depthSonar
//               << " slam=" << depthSLAM << " ratio=" << ratio << std::endl;
//     std::cout << "[PingIntegration::GetSonarDepthRatio] EXIT" << std::endl;
//     return ratio;
// }

// cv::Point3f PingIntegration::GetEffectiveSonarPointCamera()
// {
//     if (pingEffectiveDist <= 0.0f)
//     {
//         std::cout << "[PingIntegration::GetEffectiveSonarPointCamera] invalid" << std::endl;
//         return cv::Point3f(0, 0, 0);
//     }

//     float th = pingAngle;
//     cv::Mat v_s = (cv::Mat_<float>(3, 1) << std::sin(th), 0.0f, std::cos(th));
//     cv::Mat p_s = pingEffectiveDist * v_s;
//     cv::Mat p_c = R_sc * p_s + t_sc;
//     return cv::Point3f(p_c.at<float>(0), p_c.at<float>(1), p_c.at<float>(2));
// }

// void PingIntegration::RescaleInitialMap(KeyFrame *pKFini,
//                                         KeyFrame *pKFcur,
//                                         float invMedianDepth)
// {
//     std::cout << "[RescaleInitialMap] ENTER" << std::endl;
//     Sophus::SE3f Tc2w = pKFcur->GetPose();
//     Tc2w.translation() *= invMedianDepth;
//     pKFcur->SetPose(Tc2w);

//     for (MapPoint *pMP : pKFini->GetMapPointMatches())
//     {
//         if (pMP)
//         {
//             pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
//             pMP->UpdateNormalAndDepth();
//         }
//     }
//     std::cout << "[RescaleInitialMap] EXIT" << std::endl;
// }

// bool PingIntegration::ValidateWithIntensity(const SonarData &sonar)
// {
//     std::cout << "[ValidateWithIntensity] ENTER" << std::endl;
//     bool valid = (sonar.range > 0.0f) && (!sonar.intensities.empty());
//     std::cout << "[ValidateWithIntensity] EXIT ("
//               << (valid ? "valid" : "invalid") << ")" << std::endl;
//     return valid;
// }

} // namespace ORB_SLAM3
