/**                                                                               
 * @file tcHwController.cpp                                                       
 * @author Anthony Cianfrocco                                                     
 * @email afc4328@rit.edu                                                         
 *                                                                                
 * @description See header file.                                                  
 */                                                                               
                                                                                  
#include "tcHwController.h"                                                       
                                                                                  
#include <iostream>
#include "std_msgs/Bool.h"

tcHwController::tcHwController(int argc, char* argv[], int anWidthInc,
        const int anNumRows) :
    mcLoopRate(10),
    mbFinishFlag(false),
    mrLocalizedXMtrs(0),
    mrLocalizedYMtrs(0),
    mrLocalizedZ(0),
    mbIsLocalized(false),
    mcNodeHandle(ros::NodeHandle()),
    mcImageTransport(mcNodeHandle),
    mnWidthInc(anWidthInc),
    mbReady(false),
    mnNumRows(anNumRows)
{
 	// Create the ros node
//    ros::init(argc,argv,"localization-3d");
    mcNodeHandle = ros::NodeHandle();        

    ROS_INFO_STREAM("tcHwController CTOR: mnWidthInc = " << mnWidthInc << ", anNumRows = " << anNumRows);

    mcOdomSub = mcNodeHandle.subscribe("/r1/odom", 1, 
            &tcHwController::OdomCallback, this);

    ROS_INFO("Setup OdomSub complete");

    // Subscriber for the kinnect "laser"                                         
    mcKinnectSub = mcNodeHandle.subscribe("/r1/kinect_laser/scan",              
                                            1,                                    
                                            &tcHwController::KinnectCallback,        
                                            this);                                
                                                                                  
    ROS_INFO("Setup KinectSub complete"); 

    image_transport::TransportHints lcHints("raw", ros::TransportHints(), 
            mcNodeHandle);

//    image_transport::ImageTransport lcIt(mcNodeHandle);

    mcImageTransportSub = 
        mcImageTransport.subscribeCamera(
                "/camera/depth/image", 1, &tcHwController::DepthCallback, this, lcHints);

    ROS_INFO("Setup Depth Sub complete");

    mcLocalizedFlagPub = 
        mcNodeHandle.advertise<std_msgs::Bool>("/localizedFlag", 1000, true);

    ROS_INFO("Setup LocalizedFlagPub complete");

    mcLocalizedOdomPub = 
        mcNodeHandle.advertise<nav_msgs::Odometry>(
                "/localizedOdom", 1000, true);

    ROS_INFO("Setup OdomPub complete");

    mcPubThread = std::thread(&tcHwController::PublishCmdVelMsg, this);
}

tcHwController::~tcHwController()
{
    mcPubThread.join();
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::OdomCallback(const nav_msgs::Odometry::ConstPtr& arcOdomMsg)
{
//    ROS_INFO("Reached OdomCallback()");
    std::lock_guard<std::mutex> lcLock(mcPoseMutex);
    mcCurrentOdom.pose.pose = arcOdomMsg->pose.pose;
    mcCurrentOdom.twist.twist = arcOdomMsg->twist.twist;
    mcCurrentOdom.header.stamp = arcOdomMsg->header.stamp;
/*    ROS_INFO("Curr Odom = [%f, %f]",
            mcCurrentOdom.pose.pose.position.x,
            mcCurrentOdom.pose.pose.position.y);
*/
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::PublishCmdVelMsg()
{
    // Continues until ctrl-c is caught
    while(ros::ok() && !CheckSigInt())
    {
        // Lock mutex to get the most recent linear X and angular Z values
        std::unique_lock<std::mutex> lcLock(mcLocMutex);
        nav_msgs::Odometry lcOdomMsg{};
        lcOdomMsg.pose.pose.position.x = mrLocalizedXMtrs;
        lcOdomMsg.pose.pose.position.y = mrLocalizedYMtrs;
        lcOdomMsg.pose.pose.orientation.z = mrLocalizedZ;

        std_msgs::Bool lcBoolMsg;
        lcBoolMsg.data = mbIsLocalized;
        // release lock manually
        lcLock.unlock();

        mcLocalizedOdomPub.publish(lcOdomMsg);
        mcLocalizedFlagPub.publish(lcBoolMsg);

        ros::spinOnce();

        mcLoopRate.sleep();
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
nav_msgs::Odometry
tcHwController::GetMostRecentOdom()
{
    std::lock_guard<std::mutex> lcLock(mcPoseMutex);
    return this->mcCurrentOdom;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::Shutdown()
{
    std::cout << "HwController:Shutdown()\n";
    // Tell the pub thread to stop and wait for it to finish
    {
        std::lock_guard<std::mutex> lcLock(mcSigIntMutex);
        mbFinishFlag = true;
    }
    mcPubThread.join();

    std::cout << "Publisher thread has finished\n";
}

///////////////////////////////////////////////////////////////////////////////   
// See Header File                                                                
///////////////////////////////////////////////////////////////////////////////   
bool                                                                              
tcHwController::CheckSigInt()                                                     
{                                                                                 
    std::lock_guard<std::mutex> lcLock(mcSigIntMutex);                            
    return mbFinishFlag;                                                          
}

///////////////////////////////////////////////////////////////////////////////   
// See Header File                                                                
///////////////////////////////////////////////////////////////////////////////   
void                                                                              
tcHwController::KinnectCallback(const sensor_msgs::LaserScan::ConstPtr& arcLaserScanMsg)
{                                                                                 
//    ROS_INFO("Reached KinnectCallback()");                                      
    std::lock_guard<std::mutex> lcLock(mcLaserScanMutex);                         
                                                                                  
    // start angle of the scan [rad]                                              
    mcCurrentKinnectScan.angle_min = arcLaserScanMsg->angle_min;                  
                                                                                  
    // end angle of the scan [rad]                                                
    mcCurrentKinnectScan.angle_max = arcLaserScanMsg->angle_max;                  
                                                                                  
    // angular distance between measurements [rad]                                
    mcCurrentKinnectScan.angle_increment = arcLaserScanMsg->angle_increment;      
                                                                                  
    // time between measurements [seconds] - if your scanner is moving, this      
    //                                       will be used in interpolating position
    //                                       of 3d points                         
    mcCurrentKinnectScan.time_increment = arcLaserScanMsg->time_increment;        
                                                                                  
    // time between scans [seconds]                                               
    mcCurrentKinnectScan.scan_time = arcLaserScanMsg->scan_time;                  
                                                                                  
    // minimum range value [m]                                                    
    mcCurrentKinnectScan.range_min = arcLaserScanMsg->range_min;                  
    // maximum range value [m]                                                    
    mcCurrentKinnectScan.range_max = arcLaserScanMsg->range_max;                  
                                                                                  
    // range data [m] (Note: values < range_min or > range_max should be discarded)
    // range value of "inf" means the laser did not detect anything within range  
    mcCurrentKinnectScan.ranges = arcLaserScanMsg->ranges;                        
                                                                                  
    // intensity data [device-specific units].  If your                           
    //                     device does not provide intensities, please leave      
    //                     the array empty.                                       
    mcCurrentKinnectScan.intensities = arcLaserScanMsg->intensities;              
                                                                                  
    mcCurrentKinnectScan.header.stamp = arcLaserScanMsg->header.stamp; 

/*
    ROS_INFO("KinnectCallback: Ranges.size() = %li, angle_min = %f, angle_max = %f, angle_increment = %f time_increment = %f, scan_time = %f, range_min = %f, range_max = %f",
            mcCurrentKinnectScan.ranges.size(),
        mcCurrentKinnectScan.angle_min,
            mcCurrentKinnectScan.angle_max,
            mcCurrentKinnectScan.angle_increment,
            mcCurrentKinnectScan.time_increment,
            mcCurrentKinnectScan.scan_time,
            mcCurrentKinnectScan.range_min,
            mcCurrentKinnectScan.range_max);
*/
    std::stringstream lcSs;
    for(auto x : mcCurrentKinnectScan.ranges)
    {
        lcSs << std::to_string(x) << ", ";
    }
//    ROS_INFO("Ranges = %s", lcSs.str().c_str());

    std::stringstream ss;
    for(auto x : mcCurrentKinnectScan.intensities)
    {
        ss << std::to_string(x) << ", ";
    }
//    ROS_INFO("Intensities = %s", ss.str().c_str());
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
sensor_msgs::LaserScan
tcHwController::GetMostRecentKinnectScan()
{
    std::lock_guard<std::mutex> lcLock(mcLaserScanMutex);
    return mcCurrentKinnectScan;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::SetIsLocalized(bool abIsLocalized)
{
    std::lock_guard<std::mutex> lcLock(mcLocMutex);
    mbIsLocalized = abIsLocalized;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::SetLocOdom(const float arX, const float arY,
        const float arAngularZ)
{
    std::lock_guard<std::mutex> lcLock(mcLocMutex);
    mrLocalizedXMtrs = arX;
    mrLocalizedYMtrs = arY;
    mrLocalizedZ = arAngularZ;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::DepthCallback(const sensor_msgs::ImageConstPtr& arpDepthMsg,
        const sensor_msgs::CameraInfoConstPtr& arpInfoMsg)
{
    ROS_INFO("Reached DepthCallback()");

    std::lock_guard<std::mutex> lcLock(mcDepthMutex);
/*    mcCurrentDepthImage.header = arcDepthImageMsg->header;
    mcCurrentDepthImage.height = arcDepthImageMsg->height;
    mcCurrentDepthImage.width = arcDepthImageMsg->width;
    mcCurrentDepthImage.encoding = arcDepthImageMsg->encoding;
    mcCurrentDepthImage.is_bigendian = arcDepthImageMsg->is_bigendian;
    mcCurrentDepthImage.step = arcDepthImageMsg->step;
    mcCurrentDepthImage.data = arcDepthImageMsg->data;
*/
    // Convert msg from pixels to meters
    ConvertDepthMsg(arpDepthMsg, arpInfoMsg);
    
    if(!mbReady)
        mbReady = true;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcHwController::ConvertDepthMsg(const sensor_msgs::ImageConstPtr& arpDepthMsg,
        const sensor_msgs::CameraInfoConstPtr& arpInfoMsg)
{
    ROS_INFO("ConvertDepthMsg");

    // Set camera model
    mcCamModel.fromCameraInfo(arpInfoMsg);

    // Calculate angle_min and angle_max by measuring angles between the left 
    // ray, right ray, and optical center ray

    cv::Point2d raw_pixel_left(0, mcCamModel.cy());
    cv::Point2d rect_pixel_left = mcCamModel.rectifyPoint(raw_pixel_left);
    cv::Point3d left_ray = mcCamModel.projectPixelTo3dRay(rect_pixel_left);

    cv::Point2d raw_pixel_right(arpDepthMsg->width-1, mcCamModel.cy());
    cv::Point2d rect_pixel_right = mcCamModel.rectifyPoint(raw_pixel_right);
    cv::Point3d right_ray = mcCamModel.projectPixelTo3dRay(rect_pixel_right);

    cv::Point2d raw_pixel_center(mcCamModel.cx(), mcCamModel.cy());
    cv::Point2d rect_pixel_center = mcCamModel.rectifyPoint(raw_pixel_center);
    cv::Point3d center_ray = mcCamModel.projectPixelTo3dRay(rect_pixel_center);

    const double angle_max = AngleBetweenRays(left_ray, center_ray);

    // Negative because the laserscan message expects an opposite rotation of 
    // that from the depth image
    const double angle_min = -AngleBetweenRays(center_ray, right_ray); 

    ROS_INFO_STREAM("HwCtrl: minAngle = " << angle_min << ", maxAngle = " << angle_max);

    msDepthImage.mrAngleMin = angle_min;
    msDepthImage.mrAngleMax = angle_max;
    msDepthImage.mrAngleIncrement = 
        (angle_max - angle_min) / (arpDepthMsg->width - 1);
    msDepthImage.mrRangeMin = 0.5;  //mrMinRange;
    msDepthImage.mrRangeMax = 10.0; //mrMaxRange;

    // Only use middle % of the scan
    const int lnScanHeight = (int)arpDepthMsg->height * 0.5;
 
    // Check scan height vs image height
    if(lnScanHeight / 2 > mcCamModel.cy() || 
            lnScanHeight / 2 > arpDepthMsg->height - mcCamModel.cy())
    {
        std::stringstream ss;
        ss << "scan_height ( " << lnScanHeight << " pixels) is too large for the image height.";
        throw std::runtime_error(ss.str());
    }
 
    // Calculate and fill ranges
    const uint32_t lrRangesWidth = arpDepthMsg->width;
    const uint32_t lrRangesHeight = arpDepthMsg->height;
 
/*    // Init 2D vector with Nan value  
    lsDepthImage.mcRanges = std::vector<std::vector<float>>(
            lrRangesHeight,
            std::vector<float>(lrRangesWidth, 
                std::numeric_limits<float>::quiet_NaN())
    ); 
*/
    ROS_INFO_STREAM("Converting DepthMsg for ScanHeight = " << lnScanHeight << 
		    ", WidthInc = " << mnWidthInc << "\n");

    if(arpDepthMsg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        Convert<uint16_t>(arpDepthMsg, mcCamModel, msDepthImage, lnScanHeight, mnWidthInc);
    }
    else if (arpDepthMsg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        Convert<float>(arpDepthMsg, mcCamModel, msDepthImage, lnScanHeight, mnWidthInc);
    }
    else
    {
        std::stringstream ss;
        ss << "Depth image has unsupported encoding: " << arpDepthMsg->encoding;
        throw std::runtime_error(ss.str());
    }

    ROS_INFO_STREAM("Converted depth image: size = " << msDepthImage.mcRanges.size());
}

double 
tcHwController::AngleBetweenRays(const cv::Point3d& ray1,
        const cv::Point3d& ray2) const
{
    const double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
    const double magnitude1 = MagnitudeOfRay(ray1);
    const double magnitude2 = MagnitudeOfRay(ray2);
    return acos(dot_product / (magnitude1 * magnitude2));
}

double 
tcHwController::MagnitudeOfRay(const cv::Point3d& ray) const
{
    return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}

CommonTypes::tsDepthImage 
tcHwController::GetMostRecentConvertedDepthImage() const
{
    return msDepthImage;
}

bool
tcHwController::CheckReady() const
{
    return mbReady;
}
