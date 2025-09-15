/**
 * ORB-SLAM3 ROS wrapper (Monocular + Sonar)
 */

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>


#include<opencv2/core/core.hpp>

#include"../../../include/System.h"


using namespace std;

// Global sonar range
float g_sonar_range = -1.0f;

// Sonar callback
void SonarCallback(const std_msgs::Float32ConstPtr& msg)
{
    g_sonar_range = msg->data;
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoSonar");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun orbslam3_ros mono_sonar path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    ros::Subscriber subImg   = nh.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber subSonar = nh.subscribe("/ping360_node/sonar/data", 10, SonarCallback);

    ros::spin();

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveMap("map.bin");


    ros::shutdown();
    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Pass sonar range globally, used inside Tracking::ApplySonarFusion
    mpSLAM->TrackMonocular(cv_ptr->image,
                       msg->header.stamp.toSec(),
                       std::vector<IMU::Point>(),
                       "",
                       g_sonar_range);

}
