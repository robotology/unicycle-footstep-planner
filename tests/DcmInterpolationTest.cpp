/**
 * @file DcmInterpolationTest.cpp
 * @author Giulio Romualdi
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <memory>
#include <fstream>
#include <ctime>

// iDynTree
#include "iDynTree/Core/VectorDynSize.h"
#include "iDynTree/Core/VectorFixSize.h"
#include "iDynTree/Core/TestUtils.h"

#include "utils.h"
#include "UnicycleTrajectoryGenerator.h"

/**
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
  double nominalDuration = 2.3, lastStepSwitchTime = 0.8,  switchOverSwingRatio = 1.0;

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
void printTrajectories(const FeetInterpolator& interpolator, size_t& newMergePoint, size_t mergePoint, DCMInitialState& boundaryConditionAtMergePoint,
		       const std::string& pLFileName, const std::string& pRFileName,
		       const std::string& heightFileName, const std::string& heightAccFileName,
		       const std::string& DCMPosFileName, const std::string& DCMVelFileName,
		       const std::string& mergePointFileName)
{
  // instantiate ofstream
  std::ofstream posLeftStream, posRightStream;
  std::ofstream heightStream, heightAccelerationStream;
  std::ofstream DCMPosStream, DCMVelStream;
  std::ofstream mergePointsStream;

  // open files
  posLeftStream.open(pLFileName.c_str());
  posRightStream.open(pRFileName.c_str());

  heightStream.open(heightFileName.c_str());
  heightAccelerationStream.open(heightAccFileName.c_str());

  DCMPosStream.open(DCMPosFileName.c_str());
  DCMVelStream.open(DCMVelFileName.c_str());

  mergePointsStream.open(mergePointFileName.c_str());

  // print feet trajectories
  static std::vector<iDynTree::Transform> lFootTrajectory, rFootTrajectory;
  std::vector<iDynTree::Transform> lFootTrajectoryInput, rFootTrajectoryInput;
  interpolator.getFeetTrajectories(lFootTrajectoryInput, rFootTrajectoryInput);

  lFootTrajectory.insert(lFootTrajectory.begin() + mergePoint, lFootTrajectoryInput.begin(), lFootTrajectoryInput.end());
  lFootTrajectory.resize(mergePoint + lFootTrajectoryInput.size());
  rFootTrajectory.insert(rFootTrajectory.begin() + mergePoint, rFootTrajectoryInput.begin(), rFootTrajectoryInput.end());
  rFootTrajectory.resize(mergePoint + rFootTrajectoryInput.size());

  for (auto pose : lFootTrajectory){
    posLeftStream << pose.getPosition()(0) << " " << pose.getPosition()(1) << " " <<
      " " << pose.getPosition()(2)<< " " << std::endl;
  }

  for (auto pose : rFootTrajectory){
    posRightStream << pose.getPosition()(0) << " " << pose.getPosition()(1) << " " <<
      " " << pose.getPosition()(2)<< " " << std::endl;
  }

  // print CoM height trajectory
  static std::vector< double > CoMHeightTrajectory;
  std::vector< double > CoMHeightTrajectoryIn;
  interpolator.getCoMHeightTrajectory(CoMHeightTrajectoryIn);
  CoMHeightTrajectory.insert(CoMHeightTrajectory.begin()+ mergePoint, CoMHeightTrajectoryIn.begin(), CoMHeightTrajectoryIn.end());
  CoMHeightTrajectory.resize(mergePoint + CoMHeightTrajectoryIn.size());
  printVector(CoMHeightTrajectory, heightStream);

  // print CoM height acceleration
  static std::vector< double > CoMHeightAccelerationProfile;
  std::vector< double > CoMHeightAccelerationProfileIn;
  interpolator.getCoMHeightAccelerationProfile(CoMHeightAccelerationProfileIn);
  CoMHeightAccelerationProfile.insert(CoMHeightAccelerationProfile.begin()+ mergePoint, CoMHeightAccelerationProfileIn.begin(), CoMHeightAccelerationProfileIn.end());
  CoMHeightAccelerationProfile.resize(mergePoint + CoMHeightAccelerationProfileIn.size());
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

  std::vector<InitialState> initPoints;
  interpolator.getInitialStatesAtMergePoints(initPoints);

  std::vector<size_t> mergePoints;
  interpolator.getMergePoints(mergePoints);
  newMergePoint = mergePoints[1] + mergePoint;

  Color::Modifier green(Color::FG_GREEN);
  Color::Modifier def(Color::FG_DEFAULT);
  std::cerr << green << "Merge Points: " << def;
  printVector(mergePoints);
  printVector(mergePoints, mergePointsStream);

  // print the position of the DCM
  static std::vector<iDynTree::Vector2> DCMPosVector;
  std::vector<iDynTree::Vector2> DCMPosInput;
  DCMPosInput = interpolator.getDCMPosition();
  DCMPosVector.insert(DCMPosVector.begin() + mergePoint, DCMPosInput.begin(), DCMPosInput.end());
  DCMPosVector.resize(mergePoint + DCMPosInput.size());
  DCMPosStream << "DCM_x DCM_y" <<std::endl;
  print_iDynTree(DCMPosVector, DCMPosStream);

  // print the velocity of the DCM
  static std::vector<iDynTree::Vector2> DCMVelVector;
  std::vector<iDynTree::Vector2> DCMVelInput;
  DCMVelInput = interpolator.getDCMVelocity();
  DCMVelVector.insert(DCMVelVector.begin() + mergePoint, DCMVelInput.begin(), DCMVelInput.end());
  DCMVelVector.resize(mergePoint + DCMVelInput.size());
  DCMVelStream << "DCM_vx DCM_vy" <<std::endl;
  print_iDynTree(DCMVelVector, DCMVelStream);

  // evaluate the the new DCM boundary conditions
  boundaryConditionAtMergePoint.initialPosition = DCMPosInput[mergePoints[1]];
  boundaryConditionAtMergePoint.initialVelocity = DCMVelInput[mergePoints[1]];

  std::cerr << "************************************" << std::endl;

  // close stream
  posLeftStream.close();
  posRightStream.close();

  heightStream.close();
  heightAccelerationStream.close();

  DCMPosStream.close();
  DCMVelStream.close();

  mergePointsStream.close();
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

  // set the init time of the trajectory
  double initTime = 0;

  // add desired initial and final position dor the unicycle
  iDynTree::assertTrue(unicycle.addDesiredTrajectoryPoint(initTime, initPosition, initVelocity));
  iDynTree::assertTrue(unicycle.addDesiredTrajectoryPoint(initTime + conf.plannerHorizon,
							  finalPosition, finalVelocity));

  // generate the reference footprints and the trajectory for the DCM
  clock_t startTime, endTime;
  startTime = clock();
  std::cerr << red << "First run" << def << ": start time " << initTime << " seconds" << std::endl;

  iDynTree::assertTrue(unicycle.generateAndInterpolateDCM(leftFoot, rightFoot,
							  initTime,
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
  std::string mergePointsFileName("mergePoints1.txt");

  // print footsteep in the files
  printSteps(leftFoot->getSteps(), rightFoot->getSteps(),
	     footstepsLFileName, footstepsRFileName);

  // print the trajectory in the files
  DCMInitialState boundaryConditionAtMergePoint;
  printTrajectories(unicycle, newMergePoint, 0, boundaryConditionAtMergePoint,
		    pLFileName,  pRFileName,
		    heightFileName,  heightAccFileName,
		    DCMPosFileName, DCMVelFileName,
		    mergePointsFileName);


  // test merge points
  initTime = (newMergePoint)*conf.dT;
  std::cerr << red << "New run" << def << ": start time " << initTime << " seconds" << std::endl;
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
  iDynTree::assertTrue(unicycle.reGenerateDCM(initTime, conf.dT, initTime + conf.plannerHorizon, boundaryConditionAtMergePoint));
  endTime = clock();

  std::cerr << blue << "Total time " << (static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC)
	    << " seconds." << def <<std::endl;

  // save data
  pLFileName = "pL2.txt";
  pRFileName = "pR2.txt";
  footstepsLFileName = "footstepsL2.txt";
  footstepsRFileName = "footstepsR2.txt";
  heightFileName = "height2.txt";
  heightAccFileName = "heightAcc2.txt";
  DCMPosFileName = "DCMPos2.txt";
  DCMVelFileName = "DCMVel2.txt";
  mergePointsFileName = "mergePoints2.txt";

  printSteps(leftFoot->getSteps(), rightFoot->getSteps(),
	     footstepsLFileName, footstepsRFileName);

  printTrajectories(unicycle, newMergePoint, newMergePoint, boundaryConditionAtMergePoint,
		    pLFileName,  pRFileName,
		    heightFileName,  heightAccFileName,
		    DCMPosFileName, DCMVelFileName,
		    mergePointsFileName);

  // test merge points
  initTime = newMergePoint*conf.dT;
  std::cerr << red << "New run" << def << ": start time " << initTime << " seconds" << std::endl;
  iDynTree::assertTrue(unicycle.setEndTime(initTime + conf.plannerHorizon));
  iDynTree::assertTrue(unicycle.getPersonPosition(initTime, initPosition));

  // remove all desired trajectory point
  unicycle.clearDesiredTrajectory();

  // set new desired positin
  finalPosition(0) = initPosition(0) + 0.0;
  finalPosition(1) = initPosition(1) + 0.2;
  iDynTree::assertTrue(unicycle.addDesiredTrajectoryPoint(initTime + conf.plannerHorizon, finalPosition));

  // evaluate the new trajectory
  startTime = clock();
  iDynTree::assertTrue(unicycle.reGenerateDCM(initTime, conf.dT, initTime + conf.plannerHorizon, boundaryConditionAtMergePoint));
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
  DCMVelFileName = "DCMVel3.txt";
  mergePointsFileName = "mergePoints3.txt";

  printSteps(leftFoot->getSteps(), rightFoot->getSteps(),
	     footstepsLFileName, footstepsRFileName);

  printTrajectories(unicycle, newMergePoint, newMergePoint, boundaryConditionAtMergePoint,
		    pLFileName,  pRFileName,
		    heightFileName,  heightAccFileName,
		    DCMPosFileName, DCMVelFileName,
		    mergePointsFileName);

  return true;
}

int main(){
  iDynTree::assertTrue(interpolationTest());
  return EXIT_SUCCESS;
}
