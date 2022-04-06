/**                                                                                  
 * @file tcMapMgr.h                                                                  
 * @author Anthony Cianfrocco                                                        
 * @email afc4328@rit.edu                                                            
 *                                                                                   
 * @description Manages the localization map of the robot.                               
 *                                                                                   
 */                                                                                  
                                                                                     
#ifndef INCLUDE_TCMAPMGR_H                                        
#define INCLUDE_TCMAPMGR_H 

#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <thread>

#include "CommonTypes.h"
#include "tcHwController.h"

class tcMapMgr
{
public:
    /**
     * CTOR
     */
    tcMapMgr(const std::vector<CommonTypes::tsPose> &arcStartingPoseVec,
             const std::string &arcMapFilename, int argc, char *argv[]);

    /**
     * DTOR
     */
    ~tcMapMgr() = default;

    /**
     * Begin mapping
     */
     void Run(int argc, char *argv[]);
    
    /**
     * Shutdown the node
     */
    void Shutdown();

    typedef struct tsCoords
    {
        float mrX;
        float mrY;

        tsCoords(float arX, float arY):
            mrX(arX), mrY(arY) {}
    }tsCoords;

    /**
     * Draw the particles on the GUI from a list of particles that are 
     * in meters. This function must convert the particles from meters to 
     * pixels on the map.
     */
    void DrawParticlesFromMeters(
            std::vector<CommonTypes::tsParticle> acParticleVecMtrs);

    /**
     * Undraw the particles on the GUI from a list of particles that are 
     * in meters. This function must convert the particles from meters to 
     * pixels on the map.
     */
    void UndrawParticlesFromMeters(
            std::vector<CommonTypes::tsParticle> acParticleVecMtrs);

    /**
     * Determine if the supplied position is valid or not.
     * Invalid if the pose is in an obstacle on the map.
     */
    bool IsPoseInMtrsValid(const CommonTypes::tsPose &arsPose) const;


    /**
     * Get points in the line formed from pose at dist and angle provided
     * using Brisenham's line algo.
     */
    std::vector<tsCoords> GetPointsInRay(const float arAngle, 
            const float arDistMtrs, const CommonTypes::tsPose &arsPose, 
            const float arDistInc);

    /**
     * Return true if the value at (x, y) in the localmap is 0, false otherwise
     */
    bool CheckLocalMapForObstacle(const int anX, const int anY);

    /**
     * Calcs a point from a point at the given angle and distance
     */
    tsCoords CalcPoint(tsCoords asCurrCoord, float arAngleFromOrigin, 
            float arDist) const;

private:
    /**                                                                              
     * Opens the Map gui                                                             
     */                                                                              
    void LaunchGui(int argc, char *argv[]);     

    /**                                                                              
     * Thread that handles building the map                                          
     */                                                                              
    void Mapper();

    /**
     * Load the map image from a png to the openGL GUI
     */
    void LoadPngToOpenGl(const std::string &arcPngFilename);

    /**
     * Provided the starting possible positions, generate the particles
     * of possible locations the robot could be.
     */
    void InitParticleVec(
            const std::vector<CommonTypes::tsPose> arcStartingPoseVec);

    /**
     * Intialize phase of localization.
     */
    void InitializeLoc();

    /**
     * Draw the particles on the GUI.
     */
    void DrawParticles();

    /**
     * Return all particles locations to orignal map colors
     */
    void UndrawParticles();

    /**
     * Handles the prediction phase of localization.
     */
    void Predict();

    /**
     * Handle the update phase of localization
     */
    void Update();

    /**
     * Distance formula.
     */
    float CalcDistance(const float arX1, const float arX2,
            const float arY1, const float arY2);

    /**
     * Brisenham's line algo
     */
    std::vector<tsCoords> WalkLine(int x1, int y1, int x2, int y2);
    

    /** MEMBER DATA **/

    /**
     * 2D vec representing the local pixel map in openGL
     */
    std::vector<std::vector<float>> mcLocalMap;

    /**
     * Get sign of int. 
     */ 
    int Sign(int dxy);

    /**
     * Resample phase of localization.
     */
    void Resample();

    /**
     * Normalizes the particle probabilities b/w 0 and 1.
     */
    void Normalize();

    /**
     * Generate new particles to add.
     * Duplicate with respect to particles with higher probability.
     */
    void GenerateNewParticles();

    /**
     * Does conversion math from meters to pixels.
     */ 
    tsCoords ConvertMetersToPixels(const float arX, const float arY) const;

    /**
     * Repeatedly calls redisplay for openGL every 1 second
     */
    void RedisplayThread();

    /**
     * Converts from pixels to meters.
     */
    tsCoords ConvertPixelsToMeters(const int anXPix, const int anYPix) const;

    /**
     * Comparison function for particles.
     */
    static bool ParticleCompare(CommonTypes::tsParticle const &lhs,
            CommonTypes::tsParticle const &rhs);

    /**
     * Check if the particles are converged
     */
    bool IsConverged();

    float Mean(std::vector<float> vec, int n) const;

    float Covariance(std::vector<float> vec1, std::vector<float> vec2, int n) const;

    float GetRand(float arVal);

    /**
     * Flag for the gui window existing
     */
    bool mbWinReady;

    /**                                                                           
     * Mapper thread                                                              
     */                                                                           
    std::thread mcMapThread;

    /**
     * The particles used for localization.
     */
    std::vector<CommonTypes::tsParticle> mcParticleVec;

    /**
     * Odom of curr cycle 
     */ 
    nav_msgs::Odometry mcCurrOdom;   

    /**
     * Odom of previous cycle 
     */ 
    nav_msgs::Odometry mcPrevOdom;

    /**
     * Current Kinect Laser Scan data.
     */ 
    sensor_msgs::LaserScan mcCurrLaserScan;

    /**
     * Total number of particles at initialization.
     */
    int mnTotalStartingParticles;

    /**
     *
     */
    bool mbOnlyTurning;
};

/** STATIC FUNCTIONS AND DATA **/

/**
 * Function that displays to openGL
 */
static void Display();

/**
 * Mutex to protect the openGL map
 */
static std::mutex gcMapMutex;

/**
 * Window width
 */
static const int snWinX = 2000;

/**
 * Window height
 */
static const int snWinY = 700;

/**
 * Window scale
 */ 
static const int snWinScale = 1;

/**
 * RGB pixel vec for openGL
 */
static std::vector<std::vector<std::vector<float>>> scDrawMap;

#endif /* INCLUDE_TCMAPMGR_H */
