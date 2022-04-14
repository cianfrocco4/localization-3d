/**
 * @file tcMapMgr.cpp
 * @author Anthony Cianfrocco
 * @email afc4328@rit.edu
 *
 * @description See header file.
 *
 */

#include <GL/glut.h>
#include <png.h>
#include <math.h>

#include <random>
#include <cstdio>
#include <iostream>
#include <chrono>

#include "tcMapMgr.h"

tcMapMgr::tcMapMgr(const std::vector<CommonTypes::tsPose> &arcStartingPoseVec,
                   const std::string &arcMapFilename, int argc, char* argv[]) :
    mcLocalMap(snWinY),
    mbOnlyTurning(false),
    mbWinReady(false)
{
    std::srand(std::time(nullptr));
    scDrawMap = std::vector<std::vector<std::vector<float>>>(snWinY);
    scDrawMap = std::vector<std::vector<std::vector<float>>>(snWinY);
    
    // initialize the local and draw maps
    for(int lnI = 0; lnI < snWinY; lnI++)
    {
        mcLocalMap[lnI] = std::vector<float>(snWinX);
        scDrawMap[lnI] = std::vector<std::vector<float>>(snWinX);
        
        for(int lnQ = 0; lnQ < snWinX; lnQ++)
        {
            mcLocalMap[lnI][lnQ] = 0.5; // init to 0.5
            scDrawMap[lnI][lnQ] = std::vector<float>(3);
            scDrawMap[lnI][lnQ][0] = 0.5;
            scDrawMap[lnI][lnQ][1] = 0.5;
            scDrawMap[lnI][lnQ][2] = 0.5;
        }
    }

    LoadPngToOpenGl(arcMapFilename);

//    InitParticleVec(arcStartingPoseVec);

    mnTotalStartingParticles = mcParticleVec.size();    
}

void                                                                                 
tcMapMgr::Shutdown()                                                                 
{                                                                                    
/*    
    {                                                                                
        std::unique_lock<std::mutex> lcLock(mcSigIntMutex);                          
        mbFinishedFlag = true;                                                       
    }                                                                                
                                                                                     
    mcHwCtrl.Shutdown();
*/
}                                                                                    
                                                                                     
void                                                                                 
tcMapMgr::Run(int argc, char *argv[])                                                
{
//    mcMapThread = std::thread(&tcMapMgr::Mapper, this);
//    std::thread(&tcMapMgr::RedisplayThread, this).detach();

    LaunchGui(argc, argv);                                                           
                                                                                     
//    mcMapThread.join();
    //lcThread.join();                                            
}  

void
tcMapMgr::LaunchGui(int argc, char *argv[])
{
    // initialize the map (could be done in a more sensible spot)                 
                                                                                  
    glutInit( &argc, argv );                                                      
    glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE );                                
    glutInitWindowPosition( 50, 50 );                                             
    glutInitWindowSize( snWinX, snWinY );                                         
    glutCreateWindow( "Map" );                                                    
                                                                                  
    // OpenGL Callback                                                            
    glutDisplayFunc( Display );                                                   
                                                                                  
    mbWinReady = true;                                                            
                                                                                  
    // this blocks, makes the window do its thing                                 
    glutMainLoop();
}

void                                                                                 
tcMapMgr::Mapper()                                                                   
{
    sleep(2);
    // Do all localization tasks and update the map accordingly

    // 1) Initalization
    //    - how many particles?
    //        - 30x30 around each possible start point
    //        - (30*30) = 900 points per possible start
    //        - 900 * 9 = 8100 total points
    //

    // wait for window to be ready
    while(!mbWinReady){}
/*
    int x = 2;
    int y = 0;

    std::cout << "Enter x: ";
    std::cin >> x;
    std::cout << "Enter y: ";
    std::cin >> y;

    while(x < snWinX && y < snWinY)
    {
        mcParticleVec.clear();
       
        for(int i = x; i < x + 1; i++)
        {
            for(int j = y; j < y + 1; j++)
            {
                tsParticle lsP;
                CommonTypes::tsPose lsPose{i, j, 0};
                lsP.msPose = lsPose;
                lsP.mrProb = 0.5;

                mcParticleVec.push_back(lsP);
            }
        }
        DrawParticles();
        glutPostRedisplay();


        std::cout << "Enter x: ";
        std::cin >> x;
        std::cout << "Enter y: ";
        std::cin >> y;

    }
*/
/*
    DrawParticles();
    glutPostRedisplay();
    
    UndrawParticles();
    DrawParticles();
    glutPostRedisplay();
    
    UndrawParticles();
    for(int i = 0; i < mcParticleVec.size(); i++)
    {
        mcParticleVec[i].msPose.mrY += 20; // pixels not meters
    }

    DrawParticles();
    glutPostRedisplay();
*/

    // TODO while not converged
    while(ros::ok())
    {
        UndrawParticles();
        auto start = std::chrono::high_resolution_clock::now();

        // 2) Prediction Phase
        //Predict();
        //DrawParticles();

//        std::cout << "Finished prediction******************************\n";

        // DONT update or resample if the robot only turned and didn't move
        // forward at all
/*        if(!mbOnlyTurning)
        {
            // 3) Update Phase
            Update();
    //        std::cout << "Finished update*****************************\n";

            UndrawParticles();
            // 4) Resampling Phase
            Resample();
            DrawParticles();
        }
        glutPostRedisplay();   
*/

//        std::cout << "Finished resample********************************\n";

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//        std::cout << "Time for step: " << duration.count() << " microseconds\n";

/*        
        if(IsConverged())
        {
            // publish is_converge = true
            // publish localized_odom
           // mcHwCtrl.SetIsLocalized(true);

            // compute averge x, y, z coords
            float lrXAvg = 0;
            float lrYAvg = 0;
            float lrZAvg = 0;
            for(auto lsP : mcParticleVec)
            {
                // sum all values
                lrXAvg += lsP.msPose.mrX;
                lrYAvg += lsP.msPose.mrY;
                lrZAvg += lsP.msPose.mrTheta;
            }

            lrXAvg /= mcParticleVec.size();
            lrYAvg /= mcParticleVec.size();
            lrZAvg /= mcParticleVec.size();
          
            tsCoords lsAvg = ConvertPixelsToMeters(
                   lrXAvg, lrYAvg);

            std::cout << "Localized Odom: ( " << lrXAvg << ", " <<
                lrYAvg << ", " << lrZAvg << " )" << 
                "\nMeters: ( " << lsAvg.mrX << ", " << lsAvg.mrY << " ) " <<
                std::endl;

            //mcHwCtrl.SetLocOdom(lsAvg.mrX, lsAvg.mrY, lrZAvg);
        }
*/
    }
} 

//function to find mean
float 
tcMapMgr::Mean(std::vector<float> vec, int n) const
{
   float sum = 0;
   for(int i = 0; i < n; i++)
   sum = sum + vec[i];
   return sum / n;
}
//finding covariance
float 
tcMapMgr::Covariance(
        std::vector<float> vec1, std::vector<float> vec2, int n) const
{
   float sum = 0;
   for(int i = 0; i < n; i++)
      sum = sum + (vec1[i] - Mean(vec1, n)) * (vec2[i] - Mean(vec2, n));
   return sum / (n - 1);
}

bool
tcMapMgr::IsConverged()
{
    bool lbConverged = false;

    std::vector<float> lcXs, lcYs;
    for(auto lsP : mcParticleVec)
    {
        lcXs.push_back(lsP.msPose.mrX);
        lcYs.push_back(lsP.msPose.mrY);
    }

    float lrCov = Covariance(lcXs, lcYs, lcXs.size());

    std::cout << "Covariance b/w Xs and Ys = " << lrCov << std::endl;

    if(std::abs(lrCov) < 3) // in pixels not meters 
    {
        lbConverged = true;
    }

    return lbConverged;
}

void
tcMapMgr::Resample()
{
    const float lrThreshold = 0.00001; // this good enough???
    
    // Resample
    //  a) Normalize all prob values
    //  b) check if particles are < theshold value and get rid of it if yes
    //  c) generate new particles if removed over 1/2 of original 
    //      - duplicate the ones with higher probability
    Normalize();

    std::sort(mcParticleVec.begin(), mcParticleVec.end(), 
            [this](CommonTypes::tsParticle const &lhs, 
                CommonTypes::tsParticle const &rhs)
            {
                return lhs.mrProb > rhs.mrProb;
            }
            );


    // Remove worst 5 percent
    int lnBeginBadPart = mcParticleVec.size() - (mcParticleVec.size() / 20);
    mcParticleVec.erase(mcParticleVec.begin() + lnBeginBadPart, 
            mcParticleVec.end());

    auto lcIter = mcParticleVec.begin();
    // Remove really low prob particles
    while(lcIter != mcParticleVec.end())
    {
        if((*lcIter).mrProb < lrThreshold)
        {
            lcIter = mcParticleVec.erase(lcIter);
        }
        else
        {
            lcIter++;
        }
    }

    if(mcParticleVec.size() < (mnTotalStartingParticles / 2))
    {
        GenerateNewParticles();
    }


    // Normalize again after adding new particles
    //Normalize();
}

bool
tcMapMgr::ParticleCompare(CommonTypes::tsParticle const &lhs,
        CommonTypes::tsParticle const &rhs)
{
    return lhs.mrProb <= rhs.mrProb;
}

float
tcMapMgr::GetRand(float arVal)
{
    float lrRand = ((float) std::rand() / RAND_MAX) * arVal * 2;
    if(lrRand < arVal)
        lrRand *= -1;
    else
        lrRand -= arVal;

    return lrRand;
}

void
tcMapMgr::GenerateNewParticles()
{
    /*
    int lnNumberToGenerate = mnTotalStartingParticles - mcParticleVec.size();
    while(lnNumberToGenerate > 0)
    {
        int lnSize = mcParticleVec.size();
        const float lrForwardNoiseScalar = 1; // in pixels
        const float lrAngularNoiseScalar = 0;
        for(int lnI = 0; lnI < (lnSize / 4); lnI++)
        {
            auto lsP = mcParticleVec[lnI];
            tsParticle lsNew;
            float lrRand = GetRand(lrForwardNoiseScalar); // get rand number b/w -10 and 10
            lsNew.msPose.mrX = lsP.msPose.mrX + lrRand; // adding noise
            lrRand = GetRand(lrForwardNoiseScalar);
            lsNew.msPose.mrY = lsP.msPose.mrY + lrRand;
            lrRand = GetRand(lrAngularNoiseScalar);
            lsNew.msPose.mrTheta = lsP.msPose.mrTheta + lrRand;
            lsNew.mrProb = lsP.mrProb;

            if(mcLocalMap[round(lsNew.msPose.mrY)][round(lsNew.msPose.mrX)] != 0)
                mcParticleVec.push_back(lsNew);
                lnNumberToGenerate -= 1;
        } 
    }
    */
}

void
tcMapMgr::Normalize()
{
    int lnMinIdx = 0, lnMaxIdx = 0;
    for(int lnI = 1; lnI < mcParticleVec.size(); lnI++)
    {
        float lrProb = mcParticleVec[lnI].mrProb;
        float lrMin = mcParticleVec[lnMinIdx].mrProb;
        float lrMax = mcParticleVec[lnMaxIdx].mrProb;
        if(lrProb < lrMin)
        {
            lnMinIdx = lnI;
        } 

        if(lrProb > lrMax)
        {
            lnMaxIdx = lnI;
        }
    }

    for(int lnI = 0; lnI < mcParticleVec.size();lnI++)
    {
        mcParticleVec[lnI].mrProb = 
            (mcParticleVec[lnI].mrProb - mcParticleVec[lnMinIdx].mrProb) /
            (mcParticleVec[lnMaxIdx].mrProb - mcParticleVec[lnMinIdx].mrProb);
    }
}

void
tcMapMgr::Update()
{
    // Update Phase
    //  1) For each particle
    //      a) compute expected sensor readings and pose
    //      b) particles in walls are not possible
    //      c) compute prob of particle based on actual and expected sensor
    //      values
    //      d) Multiply previous particle prob by new one

    // a) compute expected sensor readings and pose
    //      - Walk line of pixels for defined num of angles of kinect until a 
    //      black pixel is found
    //          - once found, compute expected range
    //
    for(int lnI = 0; lnI < mcParticleVec.size(); lnI++)
    {
        CommonTypes::tsParticle lsP = mcParticleVec[lnI];
        auto lsPose = lsP.msPose;
        tsCoords lsParticlePointMtrs = 
            ConvertPixelsToMeters(lsPose.mrX, lsPose.mrY);
       
        // check if particle is in a obstacle
        if(mcLocalMap[(int) lsPose.mrY][(int) lsPose.mrX] == 0)
        {
            mcParticleVec[lnI].mrProb = 0;
/*            std::cout << "Particle in wall: (" << 
                mcParticleVec[lnI].msPose.mrX << ", " << 
                mcParticleVec[lnI].msPose.mrY << ")\n";*/
            continue;
        }

        // Vector of distances away from particle at angles 0.4, 0.2, 0, -0.2
        // and -0.4 respectively
        std::vector<float> lcDistVec;

        // Increment for angle to check in scan
        const float lrInc = 0.025; // about 1 degree

        float lrAngleOffset = mcCurrLaserScan.angle_min;//-0.4; // radians
        while(lrAngleOffset <= mcCurrLaserScan.angle_max)
        {
            bool lbFound = false;
            float lrDistAway = 0.1; // start at 0.1 meters away and go up
            while(!lbFound && lrDistAway < mcCurrLaserScan.range_max)
            {
                tsCoords lsStartPointMtrs = 
                    CalcPoint(lsParticlePointMtrs,
                            (lsPose.mrTheta + lrAngleOffset),
                            lrDistAway - 0.1);

                tsCoords lsStartPointPix =
                    ConvertMetersToPixels(lsStartPointMtrs.mrX, 
                            lsStartPointMtrs.mrY);

                tsCoords lsEndPointMtrs = 
                    CalcPoint(lsParticlePointMtrs, 
                            (lsPose.mrTheta + lrAngleOffset),
                            lrDistAway);
                
                tsCoords lsEndPointPix = 
                    ConvertMetersToPixels(lsEndPointMtrs.mrX, 
                            lsEndPointMtrs.mrY); // 5 meters away
                
                auto lcLineVec = WalkLine(
                        lsStartPointPix.mrX, lsStartPointPix.mrY,
                        lsEndPointPix.mrX, lsEndPointPix.mrY);

                // walk line until a pixel containing obstacle is found
                for(auto lsCoord : lcLineVec)
                {
                    int lnXIdx = static_cast<int>(lsCoord.mrX);
                    int lnYIdx = static_cast<int>(lsCoord.mrY);
                    if(mcLocalMap[lnYIdx][lnXIdx] == 0) 
                    {
/*                        tsCoords lsObsPointMtrs = ConvertPixelsToMeters(
                                lsCoord.mrX, lsCoord.mrY);
                        // found obstacle, so calc distance away
                        lrDistAway = CalcDistance(lsObsPointMtrs.mrX, 
                                                  lsParticlePointMtrs.mrX, 
                                                  lsObsPointMtrs.mrY, 
                                                  lsParticlePointMtrs.mrY); 
*/
                        lbFound = true;
                        break;
                    }
                }

                if(!lbFound)
                    lrDistAway += 0.1; // increment by 0.1 meters each step
            }
            lcDistVec.push_back(lrDistAway);

            std::cout << "Found obstacle at distance, angle: " <<
                lrDistAway << ", " << 
                lrAngleOffset + lsPose.mrTheta << " from pixel: (" <<
                lsPose.mrX << ", " << lsPose.mrY << ")\n";

            lrAngleOffset += lrInc;; // defaulting to every 0.2 radians 
        }

        // TODO iterate over laser scan and compare against dist values at
        // angles
        float lrLaserAngle = mcCurrLaserScan.angle_min;
        float lrAngleThreshold = 0.0001;
        for(float lrRange : mcCurrLaserScan.ranges)
        {
            if(isnan(lrRange) || isinf(lrRange) || 
                    lrRange > mcCurrLaserScan.range_max)
            {}
            else
            {
                float lrA = -0.4;
                int lnQ = 0;
                while(lrA <= 0.4)
                {
                    if(std::abs(lrLaserAngle - lrA) < lrAngleThreshold)
                    {
                        float lrDistDelta = std::abs(lcDistVec[lnQ] - lrRange);

//                        if(lrDistDelta < mcCurrLaserScan.range_max)
//                        {
//                            mcParticleVec[lnI].mrProb *= 
//                                (1 - lrDistDelta / mcCurrLaserScan.range_max);
//                        }
                        if(lrDistDelta > 2)
                        {
                            mcParticleVec[lnI].mrProb *= 0.1;
                        }
                        else if(lrDistDelta > 1)
                        {
                            mcParticleVec[lnI].mrProb *= 0.3;
                        }
                        else if(lrDistDelta > 0.5)
                        {
                            mcParticleVec[lnI].mrProb *= 0.6;
                        }
                        else
                        {
                            mcParticleVec[lnI].mrProb *= 0.9;
                        }
                    }

                    lrA += lrInc;
                    lnQ++;
                }
            }

            lrLaserAngle += mcCurrLaserScan.angle_increment;
        }
    }
   

}

float
tcMapMgr::CalcDistance(const float arX1, const float arX2,
        const float arY1, const float arY2)
{
    return std::sqrt(
                std::pow( (arX1 - arX2), 2 ) +
                std::pow( (arY1 - arY2), 2 )
            );
}

std::vector<tcMapMgr::tsCoords>
tcMapMgr::WalkLine(int x1, int y1, int x2, int y2)
{
    std::vector<tsCoords> lcPoints;                                               
                                                                                  
    int Dx = x2 - x1;                                                             
    int Dy = y2 - y1;                                                             
                                                                                  
    //# Increments                                                                
    int Sx = Sign(Dx);                                                            
    int Sy = Sign(Dy);                                                            
                                                                                  
    //# Segment length                                                            
    Dx = abs(Dx);                                                                 
    Dy = abs(Dy);                                                                 
    int D = std::max(Dx, Dy);                                                     
                                                                                  
    //# Initial remainder                                                         
    double R = D / 2;                                                             
                                                                                  
    int X = x1;                                                                   
    int Y = y1;                                                                   
    if(Dx > Dy)                                                                   
    {                                                                             
        //# Main loop                                                             
        for(int I=0; I<D; I++)                                                    
        {                                                                         
            lcPoints.push_back(tsCoords{(float) X, (float) Y});                   
            //# Update (X, Y) and R                                               
            X+= Sx; R+= Dy; //# Lateral move                                      
            if (R >= Dx)                                                          
            {                                                                     
                Y+= Sy;                                                           
                R-= Dx; //# Diagonal move                                         
            }                                                                     
        }                                                                         
    }
    else                                                                          
    {                                                                             
        //# Main loop                                                             
        for(int I=0; I<D; I++)                                                    
        {                                                                         
            lcPoints.push_back(tsCoords{(float) X, (float) Y});                   
            //# Update (X, Y) and R                                               
            Y+= Sy;                                                               
            R+= Dx; //# Lateral move                                              
            if(R >= Dy)                                                           
            {                                                                     
                X+= Sx;                                                           
                R-= Dy; //# Diagonal move                                         
            }                                                                     
        }                                                                         
    }                                                                             
                                                                                  
    return lcPoints;
}

int                                                                               
tcMapMgr::Sign(int dxy)                                                           
{                                                                                 
    if(dxy<0) return -1;                                                          
    else if(dxy>0) return 1;                                                      
    else return 0;                                                                
}

void
tcMapMgr::Predict()
{
//    UndrawParticles();
    // Prediction Phase
    //  1) get motion since last cycle
    //  2) convert to Forward Delta and Angular Delta
    //  3) Add deltas to each particle plus noise
    //  4) Update map after prediction phase

    // 1) get motion since last cycle
    //      - get odom of last cycle
    //      - get current odom
    //      - calculate deltas
    // 2) convert to Forward Delta and Angular Delta
    float lrForwardDelta;
    float lrCurrHeading, lrPrevHeading, lrHeadingDelta;
    do
    {
        DrawParticles();
        //mcCurrOdom = mcHwCtrl.GetMostRecentOdom();
        // Get kinect here too so they are consistent with eachother
//        mcCurrLaserScan = mcHwCtrl.GetMostRecentKinnectScan();

        lrForwardDelta = std::sqrt(
                    std::pow(mcPrevOdom.pose.pose.position.x - 
                             mcCurrOdom.pose.pose.position.x, 2)
                    +
                    std::pow(mcPrevOdom.pose.pose.position.y - 
                             mcCurrOdom.pose.pose.position.y, 2)
                    );

        lrPrevHeading = 2 * atan2(mcPrevOdom.pose.pose.orientation.z,
                mcPrevOdom.pose.pose.orientation.w);

        lrCurrHeading = 2 * atan2(mcCurrOdom.pose.pose.orientation.z,
                mcCurrOdom.pose.pose.orientation.w);

        lrHeadingDelta = std::abs(lrCurrHeading - lrPrevHeading);

        UndrawParticles();

    }while(ros::ok() && lrForwardDelta < 0.1 && lrHeadingDelta < 0.05);

    if(!ros::ok())
    {
        return;
    }
    else if(lrHeadingDelta > 0.05 && lrForwardDelta < 0.1)
    {
        mbOnlyTurning = true;
    }
    else
    {
        mbOnlyTurning = false;
    }

/*    std::cout << "Curr_x_y = (" << mcCurrOdom.pose.pose.position.x <<
        ", " << mcCurrOdom.pose.pose.position.y << ")" <<
        " CurrHeading = " << lrCurrHeading << " " << 
        "Prev_x_y = (" << mcPrevOdom.pose.pose.position.x <<
        ", " << mcPrevOdom.pose.pose.position.y << ")" << 
        " PrevHeading = " << lrPrevHeading << std::endl << 
        "ForwardDelta = " << lrForwardDelta << " meters" << std::endl;
*/
    // 3) Add deltas to each particle plus noise
    float lrNoiseScalar = 0; // 3 pixels = .2 meters
    for(int lnI = 0; lnI < mcParticleVec.size(); ++lnI)
    {
//        std::cout << "Old Particle Point = (" << mcParticleVec[lnI].msPose.mrX <<
//            ", " << mcParticleVec[lnI].msPose.mrY << ")" << std::endl;

        mcParticleVec[lnI].msPose.mrTheta += lrCurrHeading - lrPrevHeading;

        tsCoords lsTemp = ConvertPixelsToMeters(
                round(mcParticleVec[lnI].msPose.mrX), 
                round(mcParticleVec[lnI].msPose.mrY)
                );

//        std::cout << "Pixel in meters = (" << lsTemp.mrX << ", " <<
//            lsTemp.mrY << ")\n";

        tsCoords lsNewParticlePointMtrs = CalcPoint(
                lsTemp, mcParticleVec[lnI].msPose.mrTheta, lrForwardDelta);
       
        tsCoords lsNewParticlePointPix = ConvertMetersToPixels(
                lsNewParticlePointMtrs.mrX, lsNewParticlePointMtrs.mrY);

        // Add noise to the values
        mcParticleVec[lnI].msPose.mrX = lsNewParticlePointPix.mrX + 
                                        ((float) (std::rand()/RAND_MAX) * lrNoiseScalar);
        mcParticleVec[lnI].msPose.mrY = lsNewParticlePointPix.mrY + 
                                        ((float) (std::rand()/RAND_MAX) * lrNoiseScalar);

//        std::cout << "New Particle Point = (" << mcParticleVec[lnI].msPose.mrX <<
//            ", " << mcParticleVec[lnI].msPose.mrY << ")"  << " In meters : ("
//            << lsNewParticlePointMtrs.mrX << ", " << 
//            lsNewParticlePointMtrs.mrY << ")" << std::endl;
    }

    // 4) Redraw updated particles
    //      - Draw new ones
    DrawParticles();

//    if(mbWinReady)
//        glutPostRedisplay();

    mcPrevOdom = mcCurrOdom;
}

tcMapMgr::tsCoords
tcMapMgr::CalcPoint(tsCoords asCurrCoord, float arAngleFromOrigin, float arDist) const
{
    return tsCoords
    {
        asCurrCoord.mrX + (arDist * cos(arAngleFromOrigin)),
        asCurrCoord.mrY + (arDist * sin(arAngleFromOrigin))
    };
}

void
tcMapMgr::LoadPngToOpenGl(const std::string &arcPngFilename)
{
    // header for testing if it is a png
    png_byte header[8];

    //open file as binary
    FILE *fp = fopen(arcPngFilename.c_str(), "rb");
    if (!fp) 
    {
        std::cout << "TEXTURE_LOAD_ERROR\n";
        return;
    }

    //read the header
    fread(header, 1, 8, fp);

    //test if png
    int is_png = !png_sig_cmp(header, 0, 8);
    if (!is_png) 
    {
        fclose(fp);
        std::cout << "ERROR - file is NOT a png!\n";
        return;
    }

    //create png struct
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL,
       NULL, NULL);
    if (!png_ptr) 
    {
        fclose(fp);
        std::cout << "ERROR - could not create png pointer!\n";
        return;
    }

    //create png info struct
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) 
    {
        png_destroy_read_struct(&png_ptr, (png_infopp) NULL, (png_infopp) NULL);
        fclose(fp);
        std::cout << "ERROR - could not create png info struct!\n";
        return;
    }

    //create png info struct
    png_infop end_info = png_create_info_struct(png_ptr);
    if (!end_info) 
    {
        png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp) NULL);
        fclose(fp);
        std::cout << "ERROR - could not create png end info struct!\n";
        return;
    }

    //png error stuff, not sure libpng man suggests this.
    if (setjmp(png_jmpbuf(png_ptr))) 
    {
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        fclose(fp);
        std::cout << "ERROR - png error!\n";
        return;
    }

    //init png reading
    png_init_io(png_ptr, fp);

    //let libpng know you already read the first 8 bytes
    png_set_sig_bytes(png_ptr, 8);

    // read all the info up to the image data
    png_read_info(png_ptr, info_ptr);

    //variables to pass to get info
    int bit_depth, color_type;
    png_uint_32 twidth, theight;

    // get info about png
    png_get_IHDR(png_ptr, info_ptr, &twidth, &theight, &bit_depth, &color_type,
       NULL, NULL, NULL);

    //update width and height based on png info
    int width = twidth;
    int height = theight;

    // Update the png info struct.
    png_read_update_info(png_ptr, info_ptr);

    // Row size in bytes.
    int rowbytes = png_get_rowbytes(png_ptr, info_ptr);

    // Allocate the image_data as a big block, to be given to opengl
    png_byte *image_data = new png_byte[rowbytes * height];
    if (!image_data) 
    {
        //clean up memory and close stuff
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        fclose(fp);
        std::cout << "ERROR - could not allocate image data!\n";
        return;
    }

    //row_pointers is for pointing to image_data for reading the png with libpng
    png_bytep *row_pointers = new png_bytep[height];
    if (!row_pointers) 
    {
        //clean up memory and close stuff
        png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
        delete[] image_data;
        fclose(fp);
        std::cout << "ERROR - could not allocate row pointers!\n";
        return;
    }

    // set the individual row_pointers to point at the correct offsets of image_data
    for (int i = 0; i < height; ++i)
        row_pointers[height - 1 - i] = image_data + i * rowbytes;

    //read the png into image_data through row_pointers
    png_read_image(png_ptr, row_pointers);


    // Display map in openGL
    std::unique_lock<std::mutex> lcLock(gcMapMutex);
    for(int lnR = 0; lnR < height; lnR++)
    {
        for(int lnC = 0; lnC < width; lnC++)
        {
            const int lnIndex = lnR * width + lnC;
            mcLocalMap[lnR][lnC] = image_data[lnIndex];
            scDrawMap[lnR][lnC][0] = mcLocalMap[lnR][lnC];
            scDrawMap[lnR][lnC][1] = mcLocalMap[lnR][lnC];
            scDrawMap[lnR][lnC][2] = mcLocalMap[lnR][lnC];
        }
    }
    lcLock.unlock();

    //clean up memory and close stuff
    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
    delete[] image_data;
    delete[] row_pointers;
    fclose(fp);
}

void
tcMapMgr::InitializeLoc()
{
    // Initalization
    //    - how many particles?
    //        - 30x30 around each possible start point
    //        - (30*30) = 900 points per possible start
    //        - 900 * 9 = 8100 total points
    //
    
   // TODO 


}

tcMapMgr::tsCoords
tcMapMgr::ConvertMetersToPixels(const float arX, const float arY) const
{
    // Image is 132x46.2 meters
    const float lrXScale = 132;
    const float lrYScale = 46.2;
    const int lnXPix = round(snWinX * (arX / lrXScale) + (snWinX / 2));
    const int lnYPix = round(snWinY * (arY / lrYScale) + (snWinY / 2));
  
    return tsCoords(lnXPix, lnYPix);
}

tcMapMgr::tsCoords
tcMapMgr::ConvertPixelsToMeters(const int anXPix, const int anYPix) const
{
    const float lrXScale = 132;
    const float lrYScale = 46.2;
    const float lrX = lrXScale * ((anXPix - (snWinX / 2.0)) / snWinX);
    const float lrY = lrYScale * ((anYPix - (snWinY / 2.0)) / snWinY);

    return tsCoords(lrX, lrY);
}

void
tcMapMgr::InitParticleVec(
        const std::vector<CommonTypes::tsPose> arcStartingPoseVec)
{
/*
    for(auto lsPose : arcStartingPoseVec)
    {
        // convert to pixel from meters
        tsCoords lsPoseInPixels = ConvertMetersToPixels(lsPose.mrX, lsPose.mrY);
        int lnXPix = lsPoseInPixels.mrX;
        int lnYPix = lsPoseInPixels.mrY;

//        std::cout << "Converted starting particle from meters to pixel:" <<
//            "\n\t( " << lsPose.mrX << ", " << lsPose.mrY << " ) meters" <<
//            "\n\t( " << lnXPix << ", " << lnYPix << " ) pixels\n";
        float mrTheta = lsPose.mrTheta;
       
        const int lnParticleWidth = 30;
        const int lnParticleHeight = 30;
        const int lnPixOffset = 2; // 0.528 meters

        for(int lnParticleX = lnXPix - (lnParticleWidth * lnPixOffset / 2); 
                lnParticleX < lnXPix + (lnParticleWidth * lnPixOffset / 2); 
                lnParticleX += lnPixOffset)
        {
            if(lnParticleX >= 0 && lnParticleX < snWinX)
            {
                for(int lnParticleY = lnYPix - (lnParticleHeight * lnPixOffset / 2);
                        lnParticleY < lnYPix + (lnParticleHeight * lnPixOffset / 2); 
                        lnParticleY += lnPixOffset)
                {
                    if(lnParticleY >= 0 && lnParticleY < snWinY)
                    {
                        CommonTypes::tsParticle lsPart;

                        CommonTypes::tsPose lsParticlePose;
                        lsParticlePose.mrX = lnParticleX;
                        lsParticlePose.mrY = lnParticleY;
                        lsParticlePose.mrTheta = mrTheta;

                        lsPart.msPose = lsParticlePose;
                        lsPart.mrProb = 0.5;

                        mcParticleVec.push_back(lsPart);
                    }
                }
            } 
        } 
    }
*/
//    std::cout << "Init particle vec:\n";
    int i = 0;
/*    for(auto p : mcParticleVec)
    {
        std::cout << "Particle[ " << i << " ] = (" << p.msPose.mrX << ", " 
        << p.msPose.mrY << ")\n";
    }
*/
    // draw the initial particles on the GUI
/*    DrawParticles();*/
}

void
tcMapMgr::DrawParticles()
{
    // Display map in openGL
    std::unique_lock<std::mutex> lcLock(gcMapMutex);
    for(auto lsP : mcParticleVec)
    {
        // Particles are stored in PIXELS, no need to do conversion
        auto lsPose = lsP.msPose;

        const int lnXPix = round(lsPose.mrX);
        const int lnYPix = round(lsPose.mrY);

//        std::cout << "XVal = " << lsPose.mrX << " YVal = " << lsPose.mrY <<
//            " XPix = " << lnXPix << " YPix = " << lnYPix << std::endl;

        if(lnXPix >= 0 && lnXPix < snWinX && lnYPix >= 0 && lnYPix < snWinY)
        {
            // Set pixel to RED = (255, 0, 0)
            scDrawMap[lnYPix][lnXPix][0] = 1; 
            scDrawMap[lnYPix][lnXPix][1] = 0;
            scDrawMap[lnYPix][lnXPix][2] = 0;
        }
    }
}

void
tcMapMgr::UndrawParticles()
{
    // Display map in openGL
    std::unique_lock<std::mutex> lcLock(gcMapMutex);
    for(auto lsP : mcParticleVec)
    {
        auto lsPose = lsP.msPose;

        const int lnXPix = round(lsPose.mrX);
        const int lnYPix = round(lsPose.mrY);
        
//        std::cout << "XVal = " << lsPose.mrX << " YVal = " << lsPose.mrY <<
//            " XPix = " << lnXPix << " YPix = " << lnYPix << std::endl;

        if(lnXPix >= 0 && lnXPix < snWinX && lnYPix >= 0 && lnYPix < snWinY)
        {
            // Return pixel to original color
            scDrawMap[lnYPix][lnXPix][0] = mcLocalMap[lnYPix][lnXPix]; 
            scDrawMap[lnYPix][lnXPix][1] = mcLocalMap[lnYPix][lnXPix];
            scDrawMap[lnYPix][lnXPix][2] = mcLocalMap[lnYPix][lnXPix];
        }
    }
}

void
tcMapMgr::RedisplayThread()
{
   while(true)
   {
        //std::this_thread::sleep_for(std::chrono::seconds(1));
        if(mbWinReady)
            glutPostRedisplay();
   }
}

void
tcMapMgr::DrawParticlesFromMeters(
        std::vector<CommonTypes::tsParticle> acParticleVecMtrs)
{
    ROS_INFO("DrawParticlesFromMeters()");

    // wait until the window is ready
    while(!mbWinReady) {}

    // for each particle
    //      convert from meters to pixels
    //      draw as red pixel on map
    for(auto lsParticleMtrs : acParticleVecMtrs)
    {
        tcMapMgr::tsCoords lsCoordsPix = 
            ConvertMetersToPixels(
                    lsParticleMtrs.msPose.mrX,
                    lsParticleMtrs.msPose.mrY);

        const int lnXPix = std::round(lsCoordsPix.mrX);
        const int lnYPix = std::round(lsCoordsPix.mrY);

        std::unique_lock<std::mutex> lcLock(gcMapMutex);

        if(lnXPix >= 0 && lnXPix < snWinX && lnYPix >= 0 && lnYPix < snWinY)
        {
            // Set pixel to RED = (255, 0, 0)
            scDrawMap[lnYPix][lnXPix][0] = 1; 
            scDrawMap[lnYPix][lnXPix][1] = 0;
            scDrawMap[lnYPix][lnXPix][2] = 0;
        }

    }

    glutPostRedisplay();
}

void
tcMapMgr::UndrawParticlesFromMeters(
        std::vector<CommonTypes::tsParticle> acParticleVecMtrs)
{
    ROS_INFO("UndrawParticlesFromMeters()");

    // for each particle
    //      convert from meters to pixels
    //      draw as red pixel on map
    for(auto lsParticleMtrs : acParticleVecMtrs)
    {
        tcMapMgr::tsCoords lsCoordsPix = 
            ConvertMetersToPixels(
                    lsParticleMtrs.msPose.mrX,
                    lsParticleMtrs.msPose.mrY);

        const int lnXPix = std::round(lsCoordsPix.mrX);
        const int lnYPix = std::round(lsCoordsPix.mrY);

        std::unique_lock<std::mutex> lcLock(gcMapMutex);

        if(lnXPix >= 0 && lnXPix < snWinX && lnYPix >= 0 && lnYPix < snWinY)
        {
            // Set pixel back to what it was at the start
            scDrawMap[lnYPix][lnXPix][0] = mcLocalMap[lnYPix][lnXPix]; 
            scDrawMap[lnYPix][lnXPix][1] = mcLocalMap[lnYPix][lnXPix];
            scDrawMap[lnYPix][lnXPix][2] = mcLocalMap[lnYPix][lnXPix];
        }

    }

    //glutPostRedisplay();
}

bool
tcMapMgr::IsPoseInMtrsValid(const CommonTypes::tsPose &arsPose) const
{
    bool lbValid = true;

    tcMapMgr::tsCoords lsCoordsPix =
        ConvertMetersToPixels(
                arsPose.mrX, arsPose.mrY
                );
    const int lnXPix = std::round(lsCoordsPix.mrX);
    const int lnYPix = std::round(lsCoordsPix.mrY);

    if(mcLocalMap[lnYPix][lnXPix] == 0)
    {
        lbValid = false;
    }

    return lbValid;
}

std::vector<tcMapMgr::tsCoords>
tcMapMgr::GetPointsInRay(const float arAngle, const float arDistMtrs, 
        const CommonTypes::tsPose &arsPose, const float arDistInc)
{
    // Calc start coordinates
    tsCoords lsStartCoordsMtrs = 
        CalcPoint(tsCoords(arsPose.mrX, arsPose.mrY),
                  arsPose.mrTheta + arAngle,
                  arDistMtrs - arDistInc);

    // Convert start coords to pixels
    tsCoords lsStartCoordsPix = 
        ConvertMetersToPixels(lsStartCoordsMtrs.mrX,
                lsStartCoordsMtrs.mrY);

    // Calc end coordinates
    tsCoords lsEndCoordsMtrs = 
        CalcPoint(tsCoords(arsPose.mrX, arsPose.mrY),
                  arsPose.mrTheta + arAngle,
                  arDistMtrs);

    // Convert end coords to pixels
    tsCoords lsEndCoordsPix = 
        ConvertMetersToPixels(lsEndCoordsMtrs.mrX,
                lsEndCoordsMtrs.mrY);
    
    return WalkLine(
            lsStartCoordsPix.mrX, lsStartCoordsPix.mrY,
            lsEndCoordsPix.mrX, lsEndCoordsPix.mrY);
    
}

bool
tcMapMgr::CheckLocalMapForObstacle(const int anX, const int anY)
{
    return mcLocalMap[anY][anX] == 0;
}

/** STATIC FUNCTIONS **/

float * ConvertVecToArray()                                                       
{                                                                                 
    float *lpArr = new float [snWinX * snWinY * 3];                               
    int lnI = 0;                                                                  
    for(auto rows : scDrawMap)                                                      
    {                                                                             
        for(auto rgb_arr : rows)                                                         
        {                                                                         
            for(float val : rgb_arr)                                                   
            {                                                                     
                lpArr[lnI] = val;                                                 
                lnI++;                                                            
            }                                                                     
        }                                                                         
    }                                                                             
                                                                                  
    return lpArr;                                                                 
}  

void
Display()
{
    float *lpMapArr = ConvertVecToArray();                                        
    {                                                                             
        std::lock_guard<std::mutex> lcLock(gcMapMutex);                           
        glDrawPixels(snWinX, snWinY, GL_RGB, GL_FLOAT, lpMapArr);                 
        glFlush();                                                                
    }                                                                             
                                                                                  
    glutSwapBuffers(); 
}

