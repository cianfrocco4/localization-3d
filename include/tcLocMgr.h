/**
 * @file tcLocMgr.h
 * @author Anthony Cianfrocco
 * @email afc4328@rit.edu
 *
 * @description Manages the localization of the robot.
 *
 */

#ifndef INCLUDE_TCLOCMGR_H
#define INCLUDE_TCLOCMGR_H

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

#include "depth_traits.h"
#include "CommonTypes.h"
#include "tcMapMgr.h"

class tcLocMgr
{
public:
    /**
     * CTOR
     */
    tcLocMgr(const std::vector<CommonTypes::tsPose> &arcStartingPoseVec,
             const std::string &arcMapFilename, int argc, char *argv[]);

    /**
     * DTOR
     */
    ~tcLocMgr() = default;

    /**
     * Runs the localization algorithm.
     */
    void Run(int argc, char *argv[]);

private:
    /**
     * Threaded function that will do the localization.
     */
    void Localize();

    /**
     * Initialize the particles to be used for localization.
     */
    void InitializeParticles(
            const std::vector<CommonTypes::tsPose> &arcStartingPoseVec);

    /**
     * The prediction phase of the localization.
     */
    void Predict();

    /**
     * The update phase of the localization.
     */
    void Update();

    /**
     * The resample phase of the localization.
     */
    void Resample();

    /**
     * Normalize the particles so their probabilites sum to 1.
     */
    void NormalizeParticles();

    /**
     * Generate new particles based on current ones with high prob.
     */
    void GenerateNewParticles(const float arXPercent);

    /**
     * Returns a random number.
     */
    float GetRand(float arVal);

    /**
     * Calculate the updated particle probability as a part of the
     * update phase of localization.
     */
    float CalcParticleProb(const CommonTypes::tsParticle &arsPart);

    /**
     * Walk the line of coords and return true if an obstacle was reached.
     */
    bool WalkLine(std::vector<tcMapMgr::tsCoords> acCoordsVec);

    /**
     * Get a vector of distances that an obstacle is at for each angle
     * b/w min and max angle.
     */
    std::vector<float> GetEstimatedDistVec(const float arMinAngle,
                                               const float arMaxAngle,
                                               const float arAngleInc,
                                               const float arMaxRange,
                                               const CommonTypes::tsPose &arsPose); 

    /**
     * Convert the depth image to x y z coordinates that are more easy to 
     * work with.
     */
//    void ConvertDepthImageMsg(const sensor_msgs::ImageConstPtr& arpDepthMsg,
//            const sensor_msgs::CameraInfoConstPtr& arpInfoMsg);

    /**
     * Return the magnitude of the ray
     */
//    double MagnitudeOfRay(const cv::Point3d& ray) const;

    /**
     * Return the angle between the two provided rays.
     */
//    double AngleBetweenRays(const cv::Point3d& ray1, 
//            const cv::Point3d& ray2) const;

    /**
     * Convert the DepthMsg to local DepthImage struct.
     */
/*    template<typename T>
    void Convert(const sensor_msgs::ImageConstPtr& arpDepthMsg,
            const image_geometry::PinholeCameraModel& arcCamModel,
            tsDepthImage &arsDepthImage, const int& arnScanHeight) const
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

        // Want to start at row 0
//        const int offset = (int)(center_y - arnScanHeight/2);
//        depth_row += offset*row_step; // Offset to center of image        

        std::vector<std::vector<float>> lcDepthVals{};

        // TODO - use depth_row to access each indivual row
        for(int v = 0; v < (int)arpDepthMsg->height; ++v, depth_row += row_step)
        {
            std::vector<float> lcRowDepthVals{};
            for(int u = 0; u < (int)arpDepthMsg->width; ++u)
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
                            " meters) at x = " << x << ", z = " << z); 
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
*/
    /** MEMBER DATA */

    /**
     * Handle for MapMgr object.
     */
    tcMapMgr mcMapMgr;    

    /**
     * Thread that will handle localization.
     */
    std::thread mcLocalizeThread;

    /**
     * Vector of all of the particles.
     */
    std::vector<CommonTypes::tsParticle> mcParticleVec;

    /**
     * Current odometry.
     */
    nav_msgs::Odometry mcCurrOdom;

    /**
     * Odometry of the last step.
     */
    nav_msgs::Odometry mcPrevOdom;

    /**
     * Current kinect laser scan data. TODO - change to kinect depth img data
     */
    sensor_msgs::LaserScan mcCurrLaserScan;

    /**
     * Current converted depth image.
     */
    CommonTypes::tsDepthImage msCurrDepthImage;

    /**
     * HwController handle.
     */
    tcHwController mcHwCtrl;

    bool mbOnlyTurning;

    const float mrMinRange = 0.45;
    const float mrMaxRange = 10.0;
};

#endif /* INCLUDE_TCLOCMGR_H */
