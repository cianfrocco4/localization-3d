/**
 * @file tcLocMgr.cpp
 * @author Anthony Cianfrocco
 * @email afc4328@rit.edu
 *
 * @description See header file.
 *
 */

#include <tcLocMgr.h>

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
tcLocMgr::tcLocMgr(
        const std::vector<CommonTypes::tsPose> &arcStartingPoseVec,
        const std::string &arcMapFilename, 
        int argc, 
        char *argv[]) :
    mcMapMgr(arcStartingPoseVec, arcMapFilename, argc, argv),
    mnWidthInc(25),
    mcHwCtrl(argc, argv, 25)
{
    ROS_INFO_STREAM("tcLocMgr CTOR: mnWidthInc = " << mnWidthInc);
    // Initalize particles
    InitializeParticles(arcStartingPoseVec);
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Run(int argc, char *argv[])
{

    // Start a thread that will do localization
    mcLocalizeThread = std::thread(&tcLocMgr::Localize, this);

    // Launch the GUI
    // NOTE - this will take control of the main thread
    mcMapMgr.Run(argc, argv);
    
    mcLocalizeThread.join();
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Localize()
{
    // Draw the intial particles
    mcMapMgr.DrawParticlesFromMeters(mcParticleVec);

    // Wait until the HW Controller is ready
    while(!mcHwCtrl.CheckReady()) {}

    ROS_INFO("HW Controller is ready!");

    // init curr robot sensor data
    mcCurrOdom = mcHwCtrl.GetMostRecentOdom();
    //mcCurrLaserScan = mcHwCtrl.GetMostRecentKinnectScan(); // TODO - change to depth img
    msCurrDepthImage = mcHwCtrl.GetMostRecentConvertedDepthImage();
    mcPrevOdom = mcCurrOdom;

    while(ros::ok())
    {
 	// Undraw the particles on the map before doing prediction
        mcMapMgr.UndrawParticlesFromMeters(mcParticleVec);

        // 1) Predict()
        Predict();

        // Redraw the predicted particles
        mcMapMgr.DrawParticlesFromMeters(mcParticleVec);

        // 2) Update();
        Update();

        mcMapMgr.UndrawParticlesFromMeters(mcParticleVec);

        // 3) Resample();
        Resample(); 

	// Redraw the updated and resampled particles
        mcMapMgr.DrawParticlesFromMeters(mcParticleVec);
        // Get updated sensor readings
        mcPrevOdom = mcCurrOdom;
        mcCurrOdom = mcHwCtrl.GetMostRecentOdom();
        msCurrDepthImage = mcHwCtrl.GetMostRecentConvertedDepthImage();
    
	sleep(1);
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::InitializeParticles(
        const std::vector<CommonTypes::tsPose> &arcStartingPoseVec)
{
    ROS_INFO("InitializeParticles()");
    // TODO get x/y bounds from tcMapMgr

    for(auto lsPose : arcStartingPoseVec)
    {
        const int lnParticleWidth = 30;
        const int lnParticleHeight = 30;
        const float lrParticleOffsetMtrs = 0.5; 

        float lrXmtrs = lsPose.mrX - (lnParticleWidth * (lrParticleOffsetMtrs / 2.0));
        while(lrXmtrs < lsPose.mrX + (lnParticleWidth * (lrParticleOffsetMtrs / 2.0)))
        {
            float lrYmtrs = lsPose.mrY - (lnParticleHeight * (lrParticleOffsetMtrs / 2.0));
            while(lrYmtrs < lsPose.mrY + (lnParticleHeight * (lrParticleOffsetMtrs / 2.0)))
            {
                // TODO if y is in bounds

                // Set the new particles pose
                CommonTypes::tsPose lsNewPose(lrXmtrs, lrYmtrs, lsPose.mrTheta);
                
                // Set the new particles probability
                CommonTypes::tsParticle lsNewParticle(lsNewPose, 0.5);

                // Add the new particle to the vector
                mcParticleVec.push_back(lsNewParticle);

/*                ROS_INFO_STREAM("Adding particle " << std::to_string(lrXmtrs) << 
                    ", " << std::to_string(lrYmtrs) << ", " << 
                    std::to_string(lsPose.mrTheta) << ", " << 
                    std::to_string(mcParticleVec.size()) << 
                    std::endl);
*/
                lrYmtrs += lrParticleOffsetMtrs;
            }

            lrXmtrs += lrParticleOffsetMtrs; 
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Predict()
{
    ROS_INFO("Predict()");

    // Prediction Phase
    //  1) get motion since last cycle
    //  2) convert to forward delta and angular delta
    //  3) add deltas to each particle plus noise
    //  4) update the map after prediction phase

    // 1) Get motion since last cycle
    //      - Compare previous odom to current odom
    float lrForwardDelta, lrCurrHeading, lrHeadingDelta;

    const float lrPrevHeading = 2 * atan2(mcPrevOdom.pose.pose.orientation.z,
            mcPrevOdom.pose.pose.orientation.w);


    // Show the particles current position on the map while we wait for the
    // robot to make a movement.
    mcMapMgr.DrawParticlesFromMeters(mcParticleVec);
    ROS_INFO("Waiting for movement...");

    // loop until either the forward or angular delta has changed "enough"
    do
    {
        // Get the current odom
        mcCurrOdom = mcHwCtrl.GetMostRecentOdom();
        
        // Get the current laser scan to be consistent with odom
        //mcCurrLaserScan = mcHwCtrl.GetMostRecentKinnectScan();
        msCurrDepthImage = mcHwCtrl.GetMostRecentConvertedDepthImage();

        lrForwardDelta = std::sqrt(
                std::pow(mcPrevOdom.pose.pose.position.x -
                         mcCurrOdom.pose.pose.position.x, 2)
                +
                std::pow(mcPrevOdom.pose.pose.position.y -
                         mcCurrOdom.pose.pose.position.y, 2)
                );

        lrCurrHeading = 2 * atan2(mcCurrOdom.pose.pose.orientation.z,
                                  mcCurrOdom.pose.pose.orientation.w);

        lrHeadingDelta = std::abs(lrCurrHeading - lrPrevHeading);
    } while(ros::ok() && lrForwardDelta < 0.1 && lrHeadingDelta < 0.05);

    mcMapMgr.UndrawParticlesFromMeters(mcParticleVec);

    ROS_INFO_STREAM("Forward delta = " << lrForwardDelta << " HeadingDelta = " << lrHeadingDelta <<
		    ", PrevOdom = (" << mcPrevOdom.pose.pose.position.x << ", " << 
		    mcPrevOdom.pose.pose.position.y << ") CurrOdom = (" << 
		    mcCurrOdom.pose.pose.position.x << ", " << 
		    mcCurrOdom.pose.pose.position.y << ")");
    
    if(!ros::ok())
    {
        return;
    }
    else if(lrHeadingDelta >= 0.05 && lrForwardDelta < 0.1)
    {
        mbOnlyTurning = true;
    }
    else
    {
        mbOnlyTurning = false;
    }

    for(int lnI = 0; lnI < mcParticleVec.size(); ++lnI)
    {
        mcParticleVec[lnI].msPose.mrTheta += lrCurrHeading - lrPrevHeading;
        
        tcMapMgr::tsCoords lsTemp(mcParticleVec[lnI].msPose.mrX, 
                mcParticleVec[lnI].msPose.mrY);

        tcMapMgr::tsCoords lsNewParticlePointMtrs = mcMapMgr.CalcPoint(
                lsTemp, mcParticleVec[lnI].msPose.mrTheta, lrForwardDelta);

	ROS_INFO_STREAM("Moving point from (" << mcParticleVec[lnI].msPose.mrX << ", " << mcParticleVec[lnI].msPose.mrY << ") to (" <<
			lsNewParticlePointMtrs.mrX << ", " << lsNewParticlePointMtrs.mrY << ")");

        // TODO - add noise
        mcParticleVec[lnI].msPose.mrX = lsNewParticlePointMtrs.mrX;
        mcParticleVec[lnI].msPose.mrY = lsNewParticlePointMtrs.mrY;
    }
}   


///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
bool
tcLocMgr::WalkLine(std::vector<tcMapMgr::tsCoords> acCoordsVec)
{
    bool lbFound = false;
    for(auto lsCoord : acCoordsVec)
    {
        int lnXIdx = static_cast<int>(lsCoord.mrX); // in pixels
        int lnYIdx = static_cast<int>(lsCoord.mrY); // in pixels

        if(mcMapMgr.CheckLocalMapForObstacle(lnXIdx, lnYIdx))
        {
            lbFound = true;
            break;
        }
    }

    return lbFound;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
std::vector<float>
tcLocMgr::GetEstimatedDistVec(const float arMinAngle,
                     const float arMaxAngle,
                     const float arAngleInc,
                     const float arMaxRange,
                     const CommonTypes::tsPose &arsPose)
{
	
    ROS_INFO_STREAM("GetEstimtedDistVec: MinAngle = " << arMinAngle << ", MaxAngle = " << arMaxAngle <<
		    ", AngleInc = " << arAngleInc << ", MaxRange = " << arMaxRange << ", Pose = (" <<
		    arsPose.mrX << ", " << arsPose.mrY << ", " << arsPose.mrTheta << ")");

    // init to -0.4 radians, about the widest the kinect can sense
    float lrAngleOffset = arMinAngle;
    float lrAngleInc = std::abs((arMaxAngle - arMinAngle)) / mnWidthInc;

    ROS_INFO_STREAM("AngleInc = " << lrAngleInc);    

    std::vector<float> lcDistVec{};

    // Iterate from left to right over the scan
    while(lrAngleOffset <= arMaxAngle)
    {
        bool lbFoundObstacle = false;
        float lrDistAwayFromRobot = 0.1; // meters
        const float lrDistInc = 0.1;  // meters

        // Iterate from close to far away from the robot at the current
        // angle until an obstacle is reached
        while(!lbFoundObstacle && lrDistAwayFromRobot < arMaxRange)
        {
            auto lcCoordsPixVec = mcMapMgr.GetPointsInRay(lrAngleOffset, 
                                                       lrDistAwayFromRobot,
                                                       arsPose,
                                                       lrDistInc);
            lbFoundObstacle = WalkLine(lcCoordsPixVec);

            if(!lbFoundObstacle)
            {
                lrDistAwayFromRobot += lrDistInc;
            }
        }

        lcDistVec.push_back(lrDistAwayFromRobot);

        std::cout << "Found obstacle at distance = " << 
            lrDistAwayFromRobot << " meters, heading = " <<
            lrAngleOffset + arsPose.mrTheta << " degrees for pixel: (" <<
            arsPose.mrX << ", " << arsPose.mrY << ")" << ", AngleOffset = " << lrAngleOffset << " deg\n";

        lrAngleOffset += lrAngleInc; // Increment to the next angle
    }

    return lcDistVec;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
float
tcLocMgr::CalcParticleProb(const CommonTypes::tsParticle &arsPart)
{
    const float lrMaxAngle = msCurrDepthImage.mrAngleMax; // radians
    const float lrMinAngle = msCurrDepthImage.mrAngleMin; // radians
    const float lrAngleInc = msCurrDepthImage.mrAngleIncrement; // radians

    const float lrMaxRange = msCurrDepthImage.mrRangeMax; // meters
    const float lrMinRange = msCurrDepthImage.mrRangeMin; // meters
    const float lrVerticalInc = msCurrDepthImage.mnRowIncrement; // meters

    const int lnNumRows = msCurrDepthImage.mcRanges.size();

    // only consider middle 50% rows
    const int lnStartRow = lnNumRows * 0.25; 
    const int lnEndRow = lnNumRows * 0.75;

    float lrProb = arsPart.mrProb;

    auto lsPose = arsPart.msPose;

    // check if particle is in a obstacle
    if(mcMapMgr.IsPoseInMtrsValid(arsPart.msPose) == false)
    {
        ROS_INFO_STREAM("Particle at (" << arsPart.msPose.mrX << 
                ", " << arsPart.msPose.mrY << ") is NOT valid, prob = 0");

        lrProb = 0;
    }
    else
    {
        // Vector of distances to obstacles at angles b/w min and max angle
        auto lcEstimatedDistVec = 
            GetEstimatedDistVec(lrMinAngle, lrMaxAngle, lrMaxRange,
                                lrAngleInc, lsPose);

        int lnRow = 0;
        // Compare estimated ranges to actual in CurrDepthImage
        for (auto lcRow : msCurrDepthImage.mcRanges)
        {
            if(lnRow >= lnStartRow && lnRow <= lnEndRow)
            {
                int lnCol = 0;
                int lnEstIdx = 0;
                for(auto lrRange : lcRow)
                {
                    const float lrImageAngle = lrMinAngle + 
                        (lnCol * msCurrDepthImage.mrAngleIncrement);

                    const float lrEstimatedAngle = lrMinAngle + 
                        (lnEstIdx * lrAngleInc);

                    if(std::abs(lrImageAngle - lrEstimatedAngle) < 0.0001)
                    {
                        const float lrCurrHeight = lnRow * lrVerticalInc;

                        const float lrRangeDelta = 
                            std::abs(lrRange - lcEstimatedDistVec[lnEstIdx]);

                        if(lrRangeDelta > 2)
                        {
                            lrProb *= 0.5;
                        }
                        else if(lrRangeDelta > 1)
                        {
                            lrProb *= 0.9;
                        }
                        else if(lrRangeDelta > 0.5)
                        {
                            lrProb *= 0.95;
                        }
                        else
                        {
                            lrProb *= 0.99;
                        }

                        lnEstIdx++;
                    }

                    lnCol++;
                }
            }
            lnRow++;
        }
    }

    return lrProb;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Update()
{
    ROS_INFO("Update()");

    // Update Phase
    //  1) For each particle
    //      a) compute expected sensor readings and pose
    //      b) particles in walls are not possible
    //      c) compute prob of particle based on actual and expected sensor 
    //         values
    //      d) Multiply previous particle prob by new one

    for(int lnI = 0; lnI < mcParticleVec.size(); lnI++)
    {
        CommonTypes::tsParticle lsCurrPart = mcParticleVec[lnI];

        mcParticleVec[lnI].mrProb = CalcParticleProb(lsCurrPart);

	ROS_INFO_STREAM("Updating particle at (" << lsCurrPart.msPose.mrX << ", " << 
			lsCurrPart.msPose.mrY << "( to prob = " << mcParticleVec[lnI].mrProb <<
			"\n");
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::NormalizeParticles()
{
    float lrSum = 0;
    for(auto lsP : mcParticleVec)
    {
        lrSum += lsP.mrProb;
    }

    ROS_INFO_STREAM("NormalizeParticles() sum = " << lrSum);

    for(int lnI = 0; lnI < mcParticleVec.size(); ++lnI)
    {
        mcParticleVec[lnI].mrProb = mcParticleVec[lnI].mrProb / lrSum; 

        ROS_INFO_STREAM("Normalized prob for part (" << 
                mcParticleVec[lnI].msPose.mrX << ", " << 
                mcParticleVec[lnI].msPose.mrY << ") = " <<
                mcParticleVec[lnI].mrProb);
    }
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
float
tcLocMgr::GetRand(float arVal)
{
    float lrRand = ((float) std::rand() / RAND_MAX) * arVal * 2;
    if(lrRand < arVal)
        lrRand *= -1;
    else
        lrRand -= arVal;

    return lrRand;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::GenerateNewParticles(const int anNumToGenerate)
{
//    float lrN = 1 - arXPercent;
//    int lnNewSize = lrN / mcParticleVec.size();
//    int lnNewSize = lrN * mcParticleVec.size();
//    int lnNumberToGenerate = lnNewSize - mcParticleVec.size();

    int lnNumberToGenerate = anNumToGenerate;

    ROS_INFO_STREAM("Number to Generate = " << lnNumberToGenerate); 

    while(lnNumberToGenerate > 0)
    {
        int lnSize = mcParticleVec.size();
        const float lrForwardNoiseScalar = 0.05; // in meters
        const float lrAngularNoiseScalar = 0;
        for(int lnI = 0; lnI < (lnSize / 4) && lnI < lnNumberToGenerate; lnI++)
        {
            auto lsP = mcParticleVec[lnI];
            CommonTypes::tsParticle lsNew(CommonTypes::tsPose(0, 0, 0), 0);
            float lrRand = GetRand(lrForwardNoiseScalar); // get rand number
            lsNew.msPose.mrX = lsP.msPose.mrX + lrRand; // adding noise
            lrRand = GetRand(lrForwardNoiseScalar);
            lsNew.msPose.mrY = lsP.msPose.mrY + lrRand;
            lrRand = GetRand(lrAngularNoiseScalar);
            lsNew.msPose.mrTheta = lsP.msPose.mrTheta + lrRand;
            lsNew.mrProb = lsP.mrProb;

            mcParticleVec.push_back(lsNew);
            lnNumberToGenerate -= 1;
        }
    }
}

bool 
tcLocMgr::ProbIsZero(const CommonTypes::tsParticle &arsPart)
{
    ROS_INFO_STREAM("ProbIsZero for particle: (" << 
            arsPart.msPose.mrX << ", " << arsPart.msPose.mrY << ") = " <<
            arsPart.mrProb);

    return arsPart.mrProb == 0.0;
}

///////////////////////////////////////////////////////////////////////////////
// See Header File
///////////////////////////////////////////////////////////////////////////////
void
tcLocMgr::Resample()
{
    ROS_INFO("Resample()");

    // Resample
    //  a) Normalize all prob values
    //  b) check if particles are < theshold value and get rid of it if yes
    //  c) generate new particles if removed over 1/2 of original
    //      - duplicate the ones with higher probability

    NormalizeParticles();

    const int lnPrevSize = mcParticleVec.size();

    int temp = mcParticleVec.size();
    
    // Remove if prob = 0
    mcParticleVec.erase(
            std::remove_if(
                mcParticleVec.begin(), 
                mcParticleVec.end(), 
                tcLocMgr::ProbIsZero), 
            mcParticleVec.end());

    ROS_INFO_STREAM((temp - mcParticleVec.size())  << " particles had prob = 0");


    // Sort in order of Higher Prob -> Lower Prob
    std::sort(mcParticleVec.begin(), mcParticleVec.end(),
            [this](CommonTypes::tsParticle const &lhs,
                CommonTypes::tsParticle const &rhs)
            {
                return lhs.mrProb > rhs.mrProb;
            }
            );

    // Remove worst X percent
    float lrXPercent = 0.2; 

    int lnBeginBadPart = 
        mcParticleVec.size() - (mcParticleVec.size() * lrXPercent);
    
    mcParticleVec.erase(mcParticleVec.begin() + lnBeginBadPart,
            mcParticleVec.end());

    const int lnNewSize = mcParticleVec.size();

    const int lnNumToGen = lnPrevSize - lnNewSize;
    GenerateNewParticles(lnNumToGen);
}

