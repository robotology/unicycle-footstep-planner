/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "utils.h"

#include "UnicycleTrajectoryGenerator.h"
#include "iDynTree/Core/TestUtils.h"
#include <memory>
#include "iDynTree/Core/VectorDynSize.h"
#include "iDynTree/Core/VectorFixSize.h"
#include <fstream>
#include <ctime>

/*
 * Struct containing the necessary quantities for the trajectory planner
 */
typedef struct
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
  double nominalDuration = 2.5, lastStepSwitchTime = 0.8,  switchOverSwingRatio = 1.0;

  // Local ZMP via points
  // Right
  double rStancePositionX = 0.0, rStancePositionY = 0.0; 
  double rSwitchInitPositionX = 0.0, rSwitchInitPositionY = 0.0;
  // Left
  double lStancePositionX = 0.0, lStancePositionY = 0.0; 
  double lSwitchInitPositionX = 0.0, lSwitchInitPositionY = 0.0;
  
  bool swingLeft = true;
} Configuration;


/**
 * Configure the unicyclce planner
 * return true in case of success, false otherwise
 */
bool configurePlanner(UnicyclePlanner& planner, const Configuration &conf)
{
  bool ok = true;
    
  ok = ok && planner.setDesiredPersonDistance(conf.referencePointDistanceX, conf.referencePointDistanceY);
  ok = ok && planner.setControllerGain(conf.unicycleGain);
  ok = ok && planner.setMaximumIntegratorStepSize(conf.dT);
  ok = ok && planner.setMaxStepLength(conf.maxStepLength);
  ok = ok && planner.setMinStepWidth(conf.minWidth);
  ok = ok && planner.setMaxAngleVariation(conf.maxAngleVariation);
  ok = ok && planner.setCostWeights(conf.positionWeight, conf.timeWeight);
  ok = ok && planner.setStepTimings(conf.minStepDuration, conf.maxStepDuration, conf.nominalDuration);
  ok = ok && planner.setPlannerPeriod(conf.dT);
  ok = ok && planner.setMinimumAngleForNewSteps(conf.minAngleVariation);
  ok = ok && planner.setMinimumStepLength(conf.minStepLength);
  ok = ok && planner.setNominalWidth(conf.nominalWidth);
  planner.addTerminalStep(true);
  planner.startWithLeft(conf.swingLeft);

  return ok;
}

/**
 * Configure the interpolator
 * return true in case of success, false otherwise
 */
bool configureInterpolator(FeetInterpolator& interpolator, const Configuration &conf)
{
  bool ok = true;
  
  ok = ok && interpolator.setSwitchOverSwingRatio(conf.switchOverSwingRatio);
  ok = ok && interpolator.setTerminalHalfSwitchTime(conf.lastStepSwitchTime);
  ok = ok && interpolator.setStepHeight(conf.stepHeight);
  ok = ok && interpolator.setPauseConditions(conf.maxStepDuration, conf.nominalDuration);
  ok = ok && interpolator.setCoMHeightSettings(conf.comHeight, conf.comHeightDelta);

  // REMOVE ME
  iDynTree::Vector2 leftZMPstance, rightZMPstance;
  leftZMPstance(0) = conf.lStancePositionX;
  leftZMPstance(1) = conf.lStancePositionY;
  rightZMPstance(0) = conf.rStancePositionX;
  rightZMPstance(1) = conf.rStancePositionY;
  ok = ok && interpolator.setStanceZMPDelta(leftZMPstance, rightZMPstance);

  // REMOVE ME
  iDynTree::Vector2 leftZMPswitch, rightZMPswitch;
  leftZMPswitch(0) = conf.lSwitchInitPositionX;
  leftZMPswitch(1) = conf.lSwitchInitPositionY;
  rightZMPswitch(0) = conf.rSwitchInitPositionX;
  rightZMPswitch(1) = conf.rSwitchInitPositionY;
  ok = ok && interpolator.setInitialSwitchZMPDelta(leftZMPswitch, rightZMPswitch);

  ok = ok && interpolator.setFootLandingVelocity(conf.stepLandingVelocity);
  ok = ok && interpolator.setFootApexTime(conf.footApexTime);
  
  return ok;
}

/**
 * Save all the trajectories in files
 */
void printTrajectories(const FeetInterpolator& interpolator, size_t& newMergePoint, size_t mergePoint, InitialState& newAlpha,
		       const std::string& pL, const std::string& pR,
		       const std::string& height, const std::string& heightAcc,
		       const std::string& dcmPos, const std::string& dcmVel)
{
  // instantiate ofstream
  std::ofstream posLeftStream, posRightStream;
  std::ofstream heightStream, heightAccelerationStream;
  std::ofstream dcmPosStream, dcmVelStream;

  // open files
  posLeftStream.open(pL.c_str());
  posRightStream.open(pR.c_str());
    
  heightStream.open(height.c_str());
  heightAccelerationStream.open(heightAcc.c_str());

  dcmPosStream.open(dcmPos.c_str());
  dcmVelStream.open(dcmVel.c_str());

  // print feet trajectories
  static std::vector<iDynTree::Transform> lFootTrajectory, rFootTrajectory;  
  std::vector<iDynTree::Transform> lFootTrajectoryInput, rFootTrajectoryInput;
  interpolator.getFeetTrajectories(lFootTrajectoryInput, rFootTrajectoryInput);

  lFootTrajectory.insert(lFootTrajectory.begin() + mergePoint, lFootTrajectoryInput.begin(), lFootTrajectoryInput.end());
  lFootTrajectory.resize(mergePoint + lFootTrajectoryInput.size());
  rFootTrajectory.insert(rFootTrajectory.begin() + mergePoint, rFootTrajectoryInput.begin(), rFootTrajectoryInput.end());
  rFootTrajectory.resize(mergePoint + rFootTrajectoryInput.size());
  
  std::cerr << "--------------------------------------------->Left Trajectory." << std::endl;
  for (auto pose : lFootTrajectory){
    posLeftStream << pose.getPosition()(0) << "    " << pose.getPosition()(1) << "    " <<
      "    " << pose.getPosition()(2)<<"    "<< std::endl;
  }

  std::cerr << "--------------------------------------------->Right Trajectory." << std::endl;
  for (auto pose : rFootTrajectory){
    posRightStream << pose.getPosition()(0) << " " << pose.getPosition()(1) << " " <<
      " " << pose.getPosition()(2)<<"    "<< std::endl;
  }


  // print CoM height trajectory
  static std::vector< double > CoMHeightTrajectory;
  std::vector< double > CoMHeightTrajectoryIn;
  interpolator.getCoMHeightTrajectory(CoMHeightTrajectoryIn);
  CoMHeightTrajectory.insert(CoMHeightTrajectory.begin()+ mergePoint, CoMHeightTrajectoryIn.begin(), CoMHeightTrajectoryIn.end());
  CoMHeightTrajectory.resize(mergePoint + CoMHeightTrajectoryIn.size());

  std::cerr << "--------------------------------------------->CoM Height Trajectory." << std::endl;
  printVector(CoMHeightTrajectory, heightStream);

  // print CoM height acceleration
  static std::vector< double > CoMHeightAccelerationProfile;
  std::vector< double > CoMHeightAccelerationProfileIn;
  interpolator.getCoMHeightAccelerationProfile(CoMHeightAccelerationProfileIn);
  CoMHeightAccelerationProfile.insert(CoMHeightAccelerationProfile.begin()+ mergePoint, CoMHeightAccelerationProfileIn.begin(), CoMHeightAccelerationProfileIn.end());
  CoMHeightAccelerationProfile.resize(mergePoint + CoMHeightAccelerationProfileIn.size());

  std::cerr << "--------------------------------------------->CoM Height Acceleration." << std::endl;
  printVector(CoMHeightAccelerationProfile, heightAccelerationStream);

  
  // print the vector containing the instant when the feet are in contact
  static std::vector < bool > lFootContacts, rFootContacts;
  std::vector < bool > lFootContactsIn, rFootContactsIn;
  interpolator.getFeetStandingPeriods(lFootContactsIn, rFootContactsIn);
  lFootContacts.insert(lFootContacts.begin()+ mergePoint, lFootContactsIn.begin(), lFootContactsIn.end());
  lFootContacts.resize(mergePoint + lFootContactsIn.size());
  rFootContacts.insert(rFootContacts.begin()+ mergePoint, rFootContactsIn.begin(), rFootContactsIn.end());
  rFootContacts.resize(mergePoint + rFootContactsIn.size());


  static std::vector < bool > lFootFixed;
  std::vector < bool > lFootFixedIn;
  interpolator.getWhenUseLeftAsFixed(lFootFixedIn);
  lFootFixed.insert(lFootFixed.begin()+ mergePoint, lFootFixedIn.begin(), lFootFixedIn.end());
  lFootFixed.resize(mergePoint + lFootFixedIn.size());
  std::cerr << "--------------------------------------------->Left fixed." << std::endl;
  //    printVector(lFootFixed);

  std::vector<InitialState> initPoints;
  interpolator.getInitialStatesAtMergePoints(initPoints);

  std::vector<size_t> mergePoints;
  interpolator.getMergePoints(mergePoints);
  newMergePoint = mergePoints[1] + mergePoint;

  // REMOVE ME necessary only for ZMP
  newAlpha = initPoints[1];
  
  std::cerr << "--------------------------------------------->Merge Points." << std::endl;
  printVector(mergePoints);

  std::cerr << "--------------------------------------------->DCM trajectory." << std::endl;

  // print the position of the DCM
  static std::vector<iDynTree::Vector2> dcmPosVector;
  std::vector<iDynTree::Vector2> dcmPosInput;
  dcmPosInput = interpolator.getDcmPosition();
  dcmPosVector.insert(dcmPosVector.begin() + mergePoint, dcmPosInput.begin(), dcmPosInput.end());
  dcmPosVector.resize(mergePoint + dcmPosInput.size());
  print_iDynTree(dcmPosVector, dcmPosStream);

  // print the velocity of the DCM
  static std::vector<iDynTree::Vector2> dcmVelVector;
  std::vector<iDynTree::Vector2> dcmVelInput;
  dcmVelInput = interpolator.getDcmVelocity();
  dcmVelVector.insert(dcmVelVector.begin() + mergePoint, dcmVelInput.begin(), dcmVelInput.end());
  dcmVelVector.resize(mergePoint + dcmVelInput.size());
  print_iDynTree(dcmVelVector, dcmVelStream);

  // close stream
  posLeftStream.close();
  posRightStream.close();
  
  heightStream.close();
  heightAccelerationStream.close();

  dcmPosStream.close();
  dcmVelStream.close();
}


bool interpolationTest()
{
  Configuration conf;
  UnicycleTrajectoryGenerator unicycle;

  // configure the planner
  iDynTree::assertTrue(configurePlanner(unicycle, conf));
  iDynTree::assertTrue(configureInterpolator(unicycle, conf));

  // instantiate Footprints pointer  
  std::shared_ptr<FootPrint> leftFoot, rightFoot;
  leftFoot = std::make_shared<FootPrint>();
  rightFoot = std::make_shared<FootPrint>();

  // set the initial and final desired position and velocity  
  iDynTree::Vector2 initPosition, initVelocity, finalPosition, finalVelocity;
  initPosition(0) = conf.referencePointDistanceX;
  initPosition(1) = conf.referencePointDistanceY;
  initVelocity.zero();
  finalPosition(0) = initPosition(0) + 0.5;
  finalPosition(1) = initPosition(1) + 0;
  finalVelocity.zero();

  double initTime = 0;

  // add desired initial and final position dor the unicycle
  iDynTree::assertTrue(unicycle.addDesiredTrajectoryPoint(initTime, initPosition, initVelocity));
  iDynTree::assertTrue(unicycle.addDesiredTrajectoryPoint(initTime + conf.plannerHorizon, finalPosition, finalVelocity));

  // generate the reference footprints and the trajectory for the DCM
  clock_t startTime, endTime;
  startTime = clock();
  iDynTree::assertTrue(unicycle.generateAndInterpolate(leftFoot, rightFoot,
						       initTime, conf.dT, initTime + conf.plannerHorizon));
  endTime = clock();
  
  std::cerr << "Total time " << (static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC) << " seconds."<<std::endl;
  
  std::cerr << "Output from unicycle." << std::endl;
  
  size_t newMergePoint;
  std::string pL("pL.txt");
  std::string pR("pR.txt");
  std::string footstepsL("footstepsL.txt");
  std::string footstepsR("footstepsR.txt");
  std::string height("height.txt");
  std::string heightAcc("heightAcc.txt");
  std::string dcmPos("dcmPos.txt");
  std::string dcmVel ("dcmVel.txt");

  // print footsteep in the files
  printSteps(leftFoot->getSteps(), rightFoot->getSteps(),
	     footstepsL, footstepsR);
  
  // print the trajectory in the files
  InitialState newAlpha;
  printTrajectories(unicycle, newMergePoint, 0, newAlpha,
		    pL,  pR,
		    height,  heightAcc,
		    dcmPos, dcmVel);
    

  // test merge points
  initTime = newMergePoint*conf.dT + initTime;
  std::cerr << "New run at " << initTime << std::endl;
  iDynTree::assertTrue(unicycle.setEndTime(initTime + conf.plannerHorizon));
  iDynTree::assertTrue(unicycle.getPersonPosition(initTime, initPosition));
  
  // remove all desired trajectory point
  unicycle.clearDesiredTrajectory();

  // set new desired positin
  finalPosition(0) = initPosition(0) + 0.3;
  finalPosition(1) = initPosition(1) + 0.5;
  iDynTree::assertTrue(unicycle.addDesiredTrajectoryPoint(initTime + conf.plannerHorizon, finalPosition));

  // evaluate the new trajectory
  startTime = clock();
  iDynTree::assertTrue(unicycle.reGenerate(initTime, conf.dT, initTime + conf.plannerHorizon, newAlpha));
  endTime = clock();
  
  std::cerr << "Total time " << (static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

  // save data 
  pL = "pL1.txt";
  pR = "pR1.txt";
  footstepsL = "footstepsL1.txt";
  footstepsR = "footstepsR1.txt";
  height = "height1.txt";
  heightAcc = "heightAcc1.txt";
  dcmPos = "dcmPos1.txt";
  dcmVel = "dcmVel1.txt";

  printSteps(leftFoot->getSteps(), rightFoot->getSteps(),
	     footstepsL, footstepsR);

  printTrajectories(unicycle, newMergePoint, newMergePoint, newAlpha,
		    pL,  pR,
		    height,  heightAcc,
		    dcmPos, dcmVel);
  
  return true;
}

int main(){
  iDynTree::assertTrue(interpolationTest());
  return EXIT_SUCCESS;
}
