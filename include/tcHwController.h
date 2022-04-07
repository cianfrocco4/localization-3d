/**                                                                               
 * @file tcHwController.h                                                         
 * @author Anthony Cianfrocco                                                     
 * @email afc4328@rit.edu                                                         
 *                                                                                
 * @description This class handles publishing and subscribing to messages         
 *              related to navigation from the robot.                             
 */                                                                               
                                                                                  
#ifndef INCLUDE_TCHWCONTROLLER_H_                                                 
#define INCLUDE_TCHWCONTROLLER_H_

#include "ros/ros.h"                                                              
#include "geometry_msgs/Twist.h"                                                  
#include "nav_msgs/Odometry.h"                                                    
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include "depth_traits.h"
#include <image_geometry/pinhole_camera_model.h>
#include "CommonTypes.h"

#include <thread>                                                                 
#include <mutex>                                                                  
#include <memory> 

class tcHwController
{
public:
    /**
     * Ctor
     */
    tcHwController(int argc, char* argv[], int anWidthInc);

    /**
     * Dtor
     */
    ~tcHwController();

    /**
     * Call back for the odom msg
     */
    void OdomCallback(const nav_msgs::Odometry::ConstPtr& arcOdomMsg);

    /**                                                                           
     * Get the most recent odom rcv'd from the robot                              
     */                                                                           
    nav_msgs::Odometry GetMostRecentOdom();

    /**                                                                           
     * Shutdown the publisher and subscriber                                      
     */                                                                           
    void Shutdown(); 

    /**                                                                           
     * This is a threaded function that continuously sends /r1/cmd_vel            
     * commands to the robot at a 10Hz frequency. The velocity values it          
     * uses are the member variables mrLinearX and mrAngularZ. Our robot          
     * only can move in these two directions which is why the other velocity      
     * values are left as zero.                                                   
     */                                                                           
    void PublishCmdVelMsg();

    /**                                                                           
     * Call back for the LaserScan msg from the kinnect                           
     */                                                                           
    void KinnectCallback(const sensor_msgs::LaserScan::ConstPtr& arcLaerScanMsg);

    /**                                                                           
     * Get the most recent LaserScan recv'd from the robot's kinnect              
     */                                                                           
    sensor_msgs::LaserScan GetMostRecentKinnectScan();

    /**
     *
     */
    void SetIsLocalized(bool abIsLocalized);

    /**
     *
     */
    void SetLocOdom(const float arX, const float arY, 
            const float arAngularZ);

    /**
     * Synchronized callback for the depth image and info from the Kinect.
     */
    void DepthCallback(const sensor_msgs::ImageConstPtr& depth_msg,
            const sensor_msgs::CameraInfoConstPtr& info_msg);

    /**
     * Get the most current converted depth image
     */
    CommonTypes::tsDepthImage GetMostRecentConvertedDepthImage() const;

    bool CheckReady() const;

private:

    /**
     * Checks if a sigint happened.
     */
    bool CheckSigInt();

    /**
     * Convert the depth image to x y z coordinates in meters that are more easy 
     * to work with.
     */
    void ConvertDepthMsg(const sensor_msgs::ImageConstPtr& arpDepthMsg,
            const sensor_msgs::CameraInfoConstPtr& arpInfoMsg);

    /**
     * Return the magnitude of the ray
     */
    double MagnitudeOfRay(const cv::Point3d& ray) const;

    /**
     * Return the angle between the two provided rays.
     */
    double AngleBetweenRays(const cv::Point3d& ray1,
            const cv::Point3d& ray2) const;

     /**
     * Convert the DepthMsg to local DepthImage struct.
     */
    template<typename T>
    void Convert(const sensor_msgs::ImageConstPtr& arpDepthMsg,
            const image_geometry::PinholeCameraModel& arcCamModel,
            CommonTypes::tsDepthImage &arsDepthImage, 
            const int& arnScanHeight, const int anWidthInc) const
    {
        // Use correct principal point from calibration
        const float center_x = arcCamModel.cx();
        const float center_y = arcCamModel.cy();

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        const double unit_scaling = depthimage_traits::DepthTraits<T>::toMeters( T(1) );
        const float constant_x = unit_scaling / arcCamModel.fx();

        const T* depth_row = reinterpret_cast<const T*>(&arpDepthMsg->data[0]);
        const int row_step = arpDepthMsg->step / sizeof(T);

        arsDepthImage.mnRowIncrement = row_step;

        // Offset to start at the bottom of the desired scan height
        const int offset = (int)(center_y - arnScanHeight/2);
        depth_row += offset*row_step; // Offset to center of image        

        std::vector<std::vector<float>> lcDepthVals{};

	const int lnHeightInc = arnScanHeight / 3.0; // amount to increment v by

	ROS_INFO_STREAM("HeightInc = " << lnHeightInc << ", WidthInc = " << anWidthInc << 
			", ColInc = " << (int)arpDepthMsg->width / anWidthInc << ", ScanHeight = " <<
			arnScanHeight);

        for(int v = offset; v < offset + arnScanHeight; v += lnHeightInc, depth_row += (lnHeightInc * row_step))
        {
            std::vector<float> lcRowDepthVals{};
            for(int u = 0; u < (int)arpDepthMsg->width; u += ((int)arpDepthMsg->width / anWidthInc))
            {
                const T depth = depth_row[u];
                double r = depth; // Assign to pass through NaNs and Infs
                
                // Atan2(x, z), but depth divides out
                const double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); 
                const int index = (th - arsDepthImage.mrAngleMin) / 
                    arsDepthImage.mrAngleIncrement;      

                if (depthimage_traits::DepthTraits<T>::valid(depth)) // Not NaN or Inf
                {
                    // Calculate in XYZ
                    double x = (u - center_x) * depth * constant_x;
                    double z = depthimage_traits::DepthTraits<T>::toMeters(depth);

                    // Calculate actual distance
                    r = hypot(x, z);
 
                    lcRowDepthVals.push_back(r);
                    ROS_INFO_STREAM("Adding depth value: (" << r << 
                            " meters) at x = " << x << ", z = " << z << " --- v = " << v << ", u = " << u); 
                }
                else
                {
                    lcRowDepthVals.push_back(
                            std::numeric_limits<float>::quiet_NaN());
                }
            }
            lcDepthVals.push_back(lcRowDepthVals);
        }

        arsDepthImage.mcRanges = lcDepthVals;

        ROS_INFO_STREAM("DepthImage.mcRanges.size() = " << 
                arsDepthImage.mcRanges.size());
    }

    /**
     * Node handle for this node. Main access point to communications within the 
     * ROS system.
     */
    ros::NodeHandle mcNodeHandle;

    /**
     * Subscriber for the /r1/odom topic
     */
    ros::Subscriber mcOdomSub;

    /**
     * Frequency to publish messages to ROS system
     */
    ros::Rate mcLoopRate;

    /**
     * Most recent odom recv'd from the robot
     */
    nav_msgs::Odometry mcCurrentOdom;

    /**
     * Used on SigInt
     */
    bool mbFinishFlag;

    /**
     * Protects mbFinishFlag
     */
    std::mutex mcSigIntMutex;

    /**                                                                           
     * Thread for publisher                                                       
     */                                                                           
    std::thread mcPubThread;

    /**                                                                           
     * Mutex to protect the most current position value of the robot              
     */                                                                           
    std::mutex mcPoseMutex;

    /**                                                                           
     * Subscriber for the /r1/kinect_laser/scan topic                             
     */                                                                           
    ros::Subscriber mcKinnectSub;

    /**
     * Mutex to protect the most current laser scan data from the robot's kinnect
     */
    std::mutex mcLaserScanMutex;

    /**                                                                           
     * Most recent kinnect data recv'd from the robot                             
     */                                                                           
    sensor_msgs::LaserScan mcCurrentKinnectScan;

    /**
     * Mutex to protect the most recent depth msgs.
     */
    std::mutex mcDepthMutex;

    /**
     * Current converted depth image.
     */
    CommonTypes::tsDepthImage msCurrentDepthImage;

    /**
     * Camera model used for depth image on Kinect.
     * - image_geometry helper class for managing sensor_msgs/CameraInfo messages.
     */
    image_geometry::PinholeCameraModel mcCamModel;

    /**
     * Localized flag publisher
     */
    ros::Publisher mcLocalizedFlagPub;

    /**
     * Localized Odometry publisher
     */
    ros::Publisher mcLocalizedOdomPub;

    /**
     * Mutex for localized data
     */
    std::mutex mcLocMutex;

    /**
     * Subscribes to synchronized Image CameraInfo pairs.
     */
    image_transport::ImageTransport mcImageTransport;

    /**
     * Subscriber for image_transport
     */
    image_transport::CameraSubscriber mcImageTransportSub;

    /**
     * Current converted depth image.
     */
    CommonTypes::tsDepthImage msDepthImage;

    float mrLocalizedXMtrs;
    float mrLocalizedYMtrs;
    float mrLocalizedZ;
    bool mbIsLocalized;

    int mnWidthInc;
    bool mbReady;

    const float mrMinRange = 0.45;
    const float mrMaxRange = 10.0;
};

#endif /* INCLUDE_TCHWCONTROLLER_H_ */
