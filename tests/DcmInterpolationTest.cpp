/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi, Stefano Dafarra
 * CopyPolicy: Released under the terms of the BSD-3-Clause license, see LICENSE
 *
 */



// std
#include <memory>
#include <fstream>
#include <ctime>

// iDynTree
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/TestUtils.h>

#include "utils.h"
#include <UnicycleGenerator.h>

/**
 * Struct containing the necessary quantities for the trajectory planner
 */
struct Configuration
{
    // timing quantities
    double plannerHorizon = 10.0, dT = 0.010;

    // Unicycle quantities
    double unicycleGain = 10.0, referencePointDistanceX = 0.1, referencePointDistanceY = 0;
    double timeWeight = 2.5, positionWeight = 1;

    // Bounds
    // Step length
    double maxStepLength = 0.175, minStepLength = 0.05;
    // Step Width
    double minWidth = 0.14;
    // Angle Variations in DEGREES
    double maxAngleVariation = 10.0, minAngleVariation = 5.0;
    // Timings
    double maxStepDuration = 5.0, minStepDuration = 2.2;

    // Nominal Values
    // Width
    double nominalWidth = 0.16;
    // Height
    double stepHeight = 0.02, stepLandingVelocity = -0.1, footApexTime = 0.8;
    double comHeight = 0.53, comHeightDelta = 0.01;

    // Timings
    double nominalDuration = 2.3, lastStepSwitchTime = 0.8,  switchOverSwingRatio = 1.0;

    // Local ZMP via points
    // Right
    double rStancePositionX = 0.0, rStancePositionY = 0.0;
    double rSwitchInitPositionX = 0.0, rSwitchInitPositionY = 0.0;
    // Left
    double lStancePositionX = 0.0, lStancePositionY = 0.0;
    double lSwitchInitPositionX = 0.0, lSwitchInitPositionY = 0.0;

    // DCM offset percentage for the last step
    double lastStepDCMOffsetPercentage = 0.2;

    bool swingLeft = true;
};


/**
 * Configure the unicyclce planner
 * return true in case of success, false otherwise
 */
bool configurePlanner(std::shared_ptr<UnicyclePlanner> planner, const Configuration &conf)
{
    bool ok = true;

    ok = ok && planner->setDesiredPersonDistance(conf.referencePointDistanceX, conf.referencePointDistanceY);
    ok = ok && planner->setPersonFollowingControllerGain(conf.unicycleGain);
    ok = ok && planner->setMaximumIntegratorStepSize(conf.dT);
    ok = ok && planner->setMaxStepLength(conf.maxStepLength);
    ok = ok && planner->setMaxAngleVariation(conf.maxAngleVariation);
    ok = ok && planner->setCostWeights(conf.positionWeight, conf.timeWeight);
    ok = ok && planner->setStepTimings(conf.minStepDuration, conf.maxStepDuration, conf.nominalDuration);
    ok = ok && planner->setPlannerPeriod(conf.dT);
    ok = ok && planner->setMinimumAngleForNewSteps(conf.minAngleVariation);
    ok = ok && planner->setMinimumStepLength(conf.minStepLength);
    ok = ok && planner->setWidthSetting(conf.minWidth, conf.nominalWidth);
    planner->addTerminalStep(true);
    planner->startWithLeft(conf.swingLeft);

    return ok;
}

///**
// * Configure the interpolator
// * return true in case of success, false otherwise
// */
bool configureGenerator(UnicycleGenerator& generator, const Configuration &conf)
{
    bool ok = true;

    ok = ok && generator.setSwitchOverSwingRatio(conf.switchOverSwingRatio);
    ok = ok && generator.setTerminalHalfSwitchTime(conf.lastStepSwitchTime);
    ok = ok && generator.setPauseConditions(conf.maxStepDuration, conf.nominalDuration);

    auto dcmGenerator = generator.addDCMTrajectoryGenerator();

    // Setup the dcm planner
    iDynTree::assertTrue(dcmGenerator->setOmega(std::sqrt(9.81/conf.comHeight)));
    iDynTree::assertTrue(dcmGenerator->setAlpha(0.5));
    iDynTree::Vector2 leftOffset, rightOffset;
    leftOffset(0) = conf.lStancePositionX;
    leftOffset(1) = conf.lStancePositionY;
    rightOffset(0) = conf.rStancePositionX;
    rightOffset(1) = conf.rStancePositionY;
    iDynTree::assertTrue(dcmGenerator->setFootOriginOffset(leftOffset, rightOffset));
    iDynTree::assertTrue(dcmGenerator->setLastStepDCMOffsetPercentage(conf.lastStepDCMOffsetPercentage));

    return ok;
}

/**
 * Save all the trajectories in files
 */
void printTrajectories(UnicycleGenerator& generator, size_t& newMergePoint, size_t mergePoint, DCMInitialState& boundaryConditionAtMergePoint,
                       const std::string& DCMPosFileName, const std::string& DCMVelFileName, const std::string& ZMPPosFileName,
                       const std::string& mergePointFileName,
                       const std::string& lFootWeightFileName, const std::string& rFootWeightFileName)
{
    auto dcmGenerator = generator.addDCMTrajectoryGenerator();

    // instantiate ofstream
    std::ofstream DCMPosStream, DCMVelStream, ZMPPosStream;
    std::ofstream mergePointsStream;
    std::ofstream lFootWeightStream, rFootWeightStream;


    DCMPosStream.open(DCMPosFileName.c_str());
    ZMPPosStream.open(ZMPPosFileName.c_str());
    DCMVelStream.open(DCMVelFileName.c_str());

    mergePointsStream.open(mergePointFileName.c_str());

    lFootWeightStream.open(lFootWeightFileName.c_str());
    rFootWeightStream.open(rFootWeightFileName.c_str());


    // print the vector containing the instant when the feet are in contact

    std::vector<size_t> mergePoints;
    generator.getMergePoints(mergePoints);
    newMergePoint = mergePoints[1] + mergePoint;

    Color::Modifier green(Color::FG_GREEN);
    Color::Modifier def(Color::FG_DEFAULT);
    std::cerr << green << "Merge Points: " << def;
    printVector(mergePoints);
    printVector(mergePoints, mergePointsStream);

    // print the position of the DCM
    static std::vector<iDynTree::Vector2> DCMPosVector;
    std::vector<iDynTree::Vector2> DCMPosInput;
    DCMPosInput = dcmGenerator->getDCMPosition();
    DCMPosVector.insert(DCMPosVector.begin() + mergePoint, DCMPosInput.begin(), DCMPosInput.end());
    DCMPosVector.resize(mergePoint + DCMPosInput.size());
    DCMPosStream << "DCM_x DCM_y" <<std::endl;
    print_iDynTree(DCMPosVector, DCMPosStream);

    // print the velocity of the DCM
    static std::vector<iDynTree::Vector2> DCMVelVector;
    std::vector<iDynTree::Vector2> DCMVelInput;
    DCMVelInput = dcmGenerator->getDCMVelocity();
    DCMVelVector.insert(DCMVelVector.begin() + mergePoint, DCMVelInput.begin(), DCMVelInput.end());
    DCMVelVector.resize(mergePoint + DCMVelInput.size());
    DCMVelStream << "DCM_vx DCM_vy" <<std::endl;
    print_iDynTree(DCMVelVector, DCMVelStream);


    // print the position of the ZMP
    static std::vector<iDynTree::Vector2> ZMPPosVector;
    std::vector<iDynTree::Vector2> ZMPPosInput;
    ZMPPosInput = dcmGenerator->getZMPPosition();
    ZMPPosVector.insert(ZMPPosVector.begin() + mergePoint, ZMPPosInput.begin(), ZMPPosInput.end());
    ZMPPosVector.resize(mergePoint + ZMPPosInput.size());
    ZMPPosStream << "ZMP_x ZMP_y" <<std::endl;
    print_iDynTree(ZMPPosVector, ZMPPosStream);

    static std::vector < double > lFootWeight;
    std::vector < double > lFootWeightIn;
    static std::vector < double > rFootWeight;
    std::vector < double > rFootWeightIn;

    dcmGenerator->getWeightPercentage(lFootWeightIn, rFootWeightIn);

    lFootWeight.insert(lFootWeight.begin() + mergePoint, lFootWeightIn.begin(), lFootWeightIn.end());
    rFootWeight.insert(rFootWeight.begin() + mergePoint, rFootWeightIn.begin(), rFootWeightIn.end());
    lFootWeight.resize(mergePoint + lFootWeightIn.size());
    rFootWeight.resize(mergePoint + rFootWeightIn.size());

    printVector(lFootWeight, lFootWeightStream);
    printVector(rFootWeight, rFootWeightStream);

    // evaluate the the new DCM boundary conditions
    boundaryConditionAtMergePoint.initialPosition = DCMPosInput[mergePoints[1]];
    boundaryConditionAtMergePoint.initialVelocity = DCMVelInput[mergePoints[1]];

    std::cerr << "************************************" << std::endl;

    DCMPosStream.close();
    DCMVelStream.close();
    ZMPPosStream.close();

    mergePointsStream.close();

    lFootWeightStream.close();
    rFootWeightStream.close();

}


bool interpolationTest()
{
    // colors
    Color::Modifier red(Color::FG_RED);
    Color::Modifier blue(Color::FG_BLUE);
    Color::Modifier def(Color::FG_DEFAULT);

    // instantiate the configuration struct
    Configuration conf;

    // instantiate the trajectory generator
    UnicycleGenerator unicycleGenerator;
    auto unicyclePlanner = unicycleGenerator.unicyclePlanner();

    // configure the planner
    iDynTree::assertTrue(configurePlanner(unicyclePlanner, conf));
    iDynTree::assertTrue(configureGenerator(unicycleGenerator, conf));

    auto dcmGenerator = unicycleGenerator.addDCMTrajectoryGenerator();

    // set the initial and final desired position and velocity
    iDynTree::Vector2 initPosition, initVelocity, finalPosition, finalVelocity;
    initPosition(0) = conf.referencePointDistanceX;
    initPosition(1) = conf.referencePointDistanceY;
    initVelocity.zero();
    finalPosition(0) = initPosition(0) + 0.5;
    finalPosition(1) = initPosition(1) + 0;
    finalVelocity.zero();

    // set the init time of the trajectory
    double initTime = 0;

    // add desired initial and final position dor the unicycle
    iDynTree::assertTrue(unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(initTime, initPosition, initVelocity));
    iDynTree::assertTrue(unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(initTime + conf.plannerHorizon,
                                                            finalPosition, finalVelocity));

    // generate the reference footprints and the trajectory for the DCM
    clock_t startTime, endTime;
    startTime = clock();
    std::cerr << red << "First run" << def << ": start time " << initTime << " seconds" << std::endl;


    iDynTree::assertTrue(unicycleGenerator.generate(initTime,
                                                    conf.dT, initTime + conf.plannerHorizon));
    endTime = clock();

    std::cerr << blue << "Total time " << (static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC)
              << " seconds." << def <<std::endl;

    size_t newMergePoint;
    std::string pLFileName("pL1.txt");
    std::string pRFileName("pR1.txt");
    std::string footstepsLFileName("footstepsL1.txt");
    std::string footstepsRFileName("footstepsR1.txt");
    std::string heightFileName("height1.txt");
    std::string heightAccFileName("heightAcc1.txt");
    std::string DCMPosFileName("DCMPos1.txt");
    std::string DCMVelFileName("DCMVel1.txt");
    std::string ZMPPosFileName("ZMPPos1.txt");
    std::string mergePointsFileName("mergePoints1.txt");
    std::string lFootWeightFileName("leftFootWeight1.txt");
    std::string rFootWeightFileName("rightFootWeight1.txt");

    // print footsteep in the files
    printSteps(unicycleGenerator.getLeftFootPrint()->getSteps(), unicycleGenerator.getRightFootPrint()->getSteps(),
               footstepsLFileName, footstepsRFileName);

    // print the trajectory in the files
    DCMInitialState boundaryConditionAtMergePoint;
    printTrajectories(unicycleGenerator, newMergePoint, 0, boundaryConditionAtMergePoint,
                      DCMPosFileName, DCMVelFileName, ZMPPosFileName,
                      mergePointsFileName,
                      lFootWeightFileName, rFootWeightFileName);

    // test merge points
    initTime = (newMergePoint)*conf.dT;
    std::cerr << red << "New run" << def << ": start time " << initTime << " seconds" << std::endl;
    iDynTree::assertTrue(unicyclePlanner->getPersonPosition(initTime, initPosition));

    // remove all desired trajectory point
    unicyclePlanner->clearPersonFollowingDesiredTrajectory();

    // set new desired positin
    finalPosition(0) = initPosition(0) + 0.3;
    finalPosition(1) = initPosition(1) + 0.5;
    iDynTree::assertTrue(unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(initTime + conf.plannerHorizon, finalPosition));

    iDynTree::assertTrue(dcmGenerator->setDCMInitialState(boundaryConditionAtMergePoint));

    // evaluate the new trajectory
    startTime = clock();
    iDynTree::assertTrue(unicycleGenerator.reGenerate(initTime, conf.dT, initTime + conf.plannerHorizon));
    endTime = clock();

    std::cerr << blue << "Total time " << (static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC)
              << " seconds." << def <<std::endl;

    // save data
    DCMPosFileName = "DCMPos2.txt";
    ZMPPosFileName = "ZMPPos2.txt";
    DCMVelFileName = "DCMVel2.txt";
    mergePointsFileName = "mergePoints2.txt";
    lFootWeightFileName = "leftFootWeight2.txt";
    rFootWeightFileName = "rightFootWeight2.txt";

    printSteps(unicycleGenerator.getLeftFootPrint()->getSteps(), unicycleGenerator.getRightFootPrint()->getSteps(),
               footstepsLFileName, footstepsRFileName);

    printTrajectories(unicycleGenerator, newMergePoint, 0, boundaryConditionAtMergePoint,
                      DCMPosFileName, DCMVelFileName, ZMPPosFileName,
                      mergePointsFileName,
                      lFootWeightFileName, rFootWeightFileName);

    // test merge points
    initTime = newMergePoint*conf.dT;
    std::cerr << red << "New run" << def << ": start time " << initTime << " seconds" << std::endl;
    iDynTree::assertTrue(unicyclePlanner->getPersonPosition(initTime, initPosition));

    // remove all desired trajectory point
    unicyclePlanner->clearPersonFollowingDesiredTrajectory();

    // set new desired positin
    finalPosition(0) = initPosition(0) + 0.0;
    finalPosition(1) = initPosition(1) + 0.2;
    iDynTree::assertTrue(unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(initTime + conf.plannerHorizon, finalPosition));
    iDynTree::assertTrue(dcmGenerator->setDCMInitialState(boundaryConditionAtMergePoint));

    // evaluate the new trajectory
    startTime = clock();
    iDynTree::assertTrue(unicycleGenerator.reGenerate(initTime, conf.dT, initTime + conf.plannerHorizon));
    endTime = clock();

    std::cerr << blue << "Total time " << (static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC)
              << " seconds." << def <<std::endl;

    // save data
    pLFileName = "pL3.txt";
    pRFileName = "pR3.txt";
    footstepsLFileName = "footstepsL3.txt";
    footstepsRFileName = "footstepsR3.txt";
    heightFileName = "height3.txt";
    heightAccFileName = "heightAcc3.txt";
    DCMPosFileName = "DCMPos3.txt";
    ZMPPosFileName = "ZMPPos3.txt";
    DCMVelFileName = "DCMVel3.txt";
    mergePointsFileName = "mergePoints3.txt";
    lFootWeightFileName = "leftFootWeight3.txt";
    rFootWeightFileName = "rightFootWeight3.txt";

    printSteps(unicycleGenerator.getLeftFootPrint()->getSteps(), unicycleGenerator.getRightFootPrint()->getSteps(),
               footstepsLFileName, footstepsRFileName);

    printTrajectories(unicycleGenerator, newMergePoint, newMergePoint, boundaryConditionAtMergePoint,
                      DCMPosFileName, DCMVelFileName, ZMPPosFileName,
                      mergePointsFileName,
                      lFootWeightFileName, rFootWeightFileName);

    return true;
}

int main(){
    iDynTree::assertTrue(interpolationTest());
    return EXIT_SUCCESS;
}
