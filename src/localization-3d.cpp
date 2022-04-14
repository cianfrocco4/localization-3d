/**
 * @file localization-3d.cpp
 * @author Anthony Cianfrocco
 * @email afc4328@rit.edu         
 *
 * @description Main file for localization for Anthony Cianfrocco's capstone
 *              project.
 *              Performs Particle-Filter Based Localization.
 *
 */

#include "CommonTypes.h"
#include "tcLocMgr.h"

#include  "ros/ros.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <memory>

void
PrintUsage()
{
    std::cout << "Usage: ./localization-3d <start_locations_filename> <map_filename> <num_rows> <num_cols>\n";
}

std::vector<CommonTypes::tsPose>
ReadStartingPoseList(const std::string acFilename)
{
    std::vector<CommonTypes::tsPose> lcPoseVec;
    std::ifstream lcFile(acFilename);
    if(lcFile.is_open())
    {
        std::cout << "Opened file: " << acFilename << std::endl;

        std::string lcLine;
        while(std::getline(lcFile, lcLine))
        {
            std::istringstream lcIss(lcLine);
            std::vector<std::string> lcTempVec;
            std::string lcTemp;
            while(std::getline(lcIss, lcTemp, ' '))
            {
                lcTempVec.push_back(lcTemp);
            }

            if(lcTempVec.size() == 3)
            {
                lcPoseVec.push_back(CommonTypes::tsPose(
                            std::stof(lcTempVec[0]),
                            std::stof(lcTempVec[1]),
                            std::stof(lcTempVec[2])));
            }
        }
    }

    return lcPoseVec;
}

int
main(int argc, char *argv[])
{
    if(argc < 5)
    {
        PrintUsage();
    }
    else
    {
        std::vector<CommonTypes::tsPose> lcStartingPoseVec = 
            ReadStartingPoseList(argv[1]);

        const int lnNumRows = atoi(argv[3]);
        const int lnNumCols = atoi(argv[4]);

        std::cout << "NumRows = " << lnNumRows << 
            " NumCols = " << lnNumCols << "\n";

        ros::init(argc, argv, "localization-3d");

        tcLocMgr lcLocMgr(lcStartingPoseVec, argv[2], lnNumRows, lnNumCols, 
                          argc, argv);
        lcLocMgr.Run(argc, argv);
    }
}
