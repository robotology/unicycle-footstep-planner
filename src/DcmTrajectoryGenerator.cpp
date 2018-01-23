/**
 * @file DcmTrajectoryGenerator.cpp
 * @author Giulio Romualdi
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <math.h>
#include <vector>
#include <iostream>

// eigen
#include <Eigen/Dense>

// iDynTree
#include "iDynTree/Core/VectorFixSize.h"
#include "iDynTree/Core/EigenHelpers.h"

#include "DcmTrajectoryGenerator.h"

GeneralSupportTrajectory::GeneralSupportTrajectory(const double &startTime, const double &endTime)
{
  // set the general support trajectory domain
  m_trajectoryDomain = std::make_pair(startTime, endTime);
}

bool GeneralSupportTrajectory::timeBelongsToDomain(const double &t)
{
  // check if m_startTime <= t <=  m_endTime
  double startTime = std::get<0>(m_trajectoryDomain);
  double endTime = std::get<1>(m_trajectoryDomain);
  
  if ((t >= startTime) && (t <= endTime))
    return true;

  std::cerr << "[GENERAL SUPPORT TRAJECTORY] the time t: " << t << " does not belong to the trajectory domain" << std::endl; 
  return false;
}

const std::pair<double, double> &GeneralSupportTrajectory::getTrajectoryDomain() const
{
  return m_trajectoryDomain;
}

SingleSupportTrajectory::SingleSupportTrajectory(const double &startTime,
						 const double &endTime,
						 const double &t0,
						 const double &stepDuration,
						 const double &omega,
						 const iDynTree::Vector2 &zmp,
						 const iDynTree::Vector2 &nextDcmIos):
  GeneralSupportTrajectory(startTime, endTime),
  m_t0(t0),
  m_omega(omega),
  m_zmp(zmp)
{
  setDcmIos(nextDcmIos, stepDuration);
}

void SingleSupportTrajectory::setDcmIos(const iDynTree::Vector2 &nextDcmIos,
					const double &stepDuration)
{
  // The position of the DCM at the beginning of the i-th step
  // depends on the position of the DCM at the beginning of the steps i+1
  // dcm_ios[i] = dcm_ios[i+1]
  //              + exp(-omega * Ts[i]) * (1 - exp(-omega * Ts[i])) * zmp[i]
  
  iDynTree::toEigen(m_dcmIos) = iDynTree::toEigen(m_zmp) + exp(-m_omega * stepDuration) * (iDynTree::toEigen(nextDcmIos) - iDynTree::toEigen(m_zmp));
}

const iDynTree::Vector2& SingleSupportTrajectory::getDcmIos() const
{
  return m_dcmIos;
}

const iDynTree::Vector2& SingleSupportTrajectory::getZmp() const
{
  return m_zmp;
}

bool SingleSupportTrajectory::getDcmPos(const double &t, iDynTree::Vector2 &dcmPos)
{
  // Evaluate the position of the DCM at time t
  if (timeBelongsToDomain(t)){
    iDynTree::toEigen(dcmPos) = iDynTree::toEigen(m_zmp)
      + exp(m_omega * (t - m_t0)) * (iDynTree::toEigen(m_dcmIos) - iDynTree::toEigen(m_zmp));
    return true;
  }
  std::cerr << "[SINGLE SUPPORT TRAJECTORY] the time t: " << t << " does not belong to the trajectory domain." <<
    std::endl;
  return false;  
}

bool SingleSupportTrajectory::getDcmVel(const double &t, iDynTree::Vector2 &dcmVel)
{
  // Evaluate the velocity of the DCM at time t
  if (timeBelongsToDomain(t)){
    iDynTree::toEigen(dcmVel) = m_omega * exp(m_omega * (t - m_t0)) * (iDynTree::toEigen(m_dcmIos) - iDynTree::toEigen(m_zmp));
    return true;
  }
  std::cerr << "[SINGLE SUPPORT TRAJECTORY] the time t: " << t << " does not belong to the trajectory domain." <<
    std::endl;
  return false;  
}

DoubleSupportTrajectory::DoubleSupportTrajectory(const iDynTree::Vector2 &initPosition, 
						 const iDynTree::Vector2 &initVelocity,
						 const iDynTree::Vector2 &endPosition, 
						 const iDynTree::Vector2 &endVelocity,
						 const double &startTime,
						 const double &endTime):
  GeneralSupportTrajectory(startTime, endTime)
{

  double dsDuration = endTime - startTime;
  
  // get the boundary conditions
  iDynTree::Vector2 positionBoundaryCondsX;
  iDynTree::Vector2 positionBoundaryCondsY;
  iDynTree::Vector2 velocityBoundaryCondsX;
  iDynTree::Vector2 velocityBoundaryCondsY;

  positionBoundaryCondsX.setVal(0, initPosition(0));
  positionBoundaryCondsX.setVal(1, endPosition(0));
  positionBoundaryCondsY.setVal(0, initPosition(1));
  positionBoundaryCondsY.setVal(1, endPosition(1));

  velocityBoundaryCondsX.setVal(0, initVelocity(0));
  velocityBoundaryCondsX.setVal(1, endVelocity(0));
  velocityBoundaryCondsY.setVal(0, initVelocity(1));
  velocityBoundaryCondsY.setVal(1, endVelocity(1));

  // evaluate the coefficents of the X and Y polinomials
  m_coefficentsX = polinominalInterpolation(positionBoundaryCondsX, velocityBoundaryCondsX,
					    dsDuration);

  m_coefficentsY = polinominalInterpolation(positionBoundaryCondsY, velocityBoundaryCondsY,
					    dsDuration);
}

Eigen::Vector4d DoubleSupportTrajectory::polinominalInterpolation(const iDynTree::Vector2 &positionBoundaryConds, 
								  const iDynTree::Vector2 &velocityBoundaryConds,
								  const double &dsDuration)
{

  // evaluate the coefficent of a 3-th order polinomial
  // p(t) = a3 * t^3 + a2 * t^2 + a1 * t + a0
  Eigen::Vector4d coefficents;
  Eigen::Vector4d boundaryCond;
  Eigen::Matrix4d estimationMatrix;

  // set boundary conditions
  boundaryCond << positionBoundaryConds(0),
    velocityBoundaryConds(0),
    positionBoundaryConds(1),
    velocityBoundaryConds(1);

  // set the estimation matrix
  estimationMatrix << 2/(pow(dsDuration,3)), 1/(pow(dsDuration,2)), -2/(pow(dsDuration,3)), 1/(pow(dsDuration,2)),
    -3/(pow(dsDuration,2)), -2/dsDuration, 3/(pow(dsDuration,2)), -1/dsDuration,
    0, 1, 0, 0,
    1, 0, 0, 0;
  
  // evaluate the trajectory parameters
  coefficents = estimationMatrix * boundaryCond;

  return coefficents;
}

bool DoubleSupportTrajectory::getDcmPos(const double &t, iDynTree::Vector2 &dcmPos)
{
  if (timeBelongsToDomain(t)){
    // Evaluate the position of the desired DCM at time t
    double startTime = std::get<0>(m_trajectoryDomain);
    double time = t - startTime;
    Eigen::Vector4d tVector;
    tVector<< pow(time,3),
      pow(time,2),
      time,
      1;
    
    // evaluate booth x and y coordinates
    double dcmPosX = tVector.dot(m_coefficentsX);
    double dcmPosY = tVector.dot(m_coefficentsY);

    // store the DCM position into the iDyntree vector
    dcmPos.setVal(0, dcmPosX);
    dcmPos.setVal(1, dcmPosY);
    
    return true;
  }
  
  std::cerr << "[DOUBLE SUPPORT TRAJECTORY] the time t: " << t << " does not belong to the trajectory domain." <<
    std::endl;
  return false;
}

bool DoubleSupportTrajectory::getDcmVel(const double &t, iDynTree::Vector2 &dcmVel)
{
  if (timeBelongsToDomain(t)){
    // Evaluate the position of the desired DCM at time t
    double startTime = std::get<0>(m_trajectoryDomain);
    double time = t - startTime;
    Eigen::Vector4d tVector;
    tVector<< 3 * pow(time,2),
      2 * time,
      1,
      0;

    // evaluate booth x and y coordinates
    double dcmVelX = tVector.dot(m_coefficentsX);
    double dcmVelY = tVector.dot(m_coefficentsY);

    // store the DCM velocity into the iDyntree vector
    dcmVel.setVal(0, dcmVelX);
    dcmVel.setVal(1, dcmVelY);
    
    return true;
  }
  
  std::cerr << "[DOUBLE SUPPORT TRAJECTORY] the time t: " << t << "does not belong to the trajectory domain." <<
    std::endl;
  return false;  
}

DcmTrajectoryGenerator::DcmTrajectoryGenerator():
  m_pauseActive(false)
{}

DcmTrajectoryGenerator::DcmTrajectoryGenerator(const double &dT, const double &omega):
  m_omega(omega),
  m_dT(dT),
  m_pauseActive(false)
{}

void DcmTrajectoryGenerator::setOmega(const double &omega)
{
  m_omega = omega;
}

void DcmTrajectoryGenerator::setdT(const double &dT)
{
  m_dT = dT;
}

bool DcmTrajectoryGenerator::addLastStep(const double &singleSupportStartTime,
					 const double &singleSupportEndTime,
					 const double &singleSupportDuration,
					 const double &doubleSupportEndTime,
					 const double &t0,
					 const iDynTree::Vector2 &comPosition,
					 const iDynTree::Vector2 &zmp)
{
  // evaluate the Single Support trajectory parameters
  std::shared_ptr<GeneralSupportTrajectory> newSingleSupport = nullptr;
  newSingleSupport.reset(new SingleSupportTrajectory(singleSupportStartTime, singleSupportEndTime,
						     t0, singleSupportDuration, m_omega,
						     zmp, comPosition));
  
  // instantiate position and velocity boundary conditions vectors
  iDynTree::Vector2 initPosition, initVelocity;
  iDynTree::Vector2 endPosition, endVelocity;

  // set boundary conditions
  if(!newSingleSupport->getDcmPos(singleSupportEndTime, initPosition)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM in the next SS phase is evaluated." <<std::endl;
    return false;
  }

  if(!newSingleSupport->getDcmVel(singleSupportEndTime, initVelocity)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM in the next SS phase is evaluated." <<std::endl;
    return false;
  }

  endPosition = comPosition;
  endVelocity.zero();
  double doubleSupportStartTime = singleSupportEndTime;
  std::shared_ptr<GeneralSupportTrajectory> newDoubleSupport = nullptr;
  newDoubleSupport.reset(new DoubleSupportTrajectory(initPosition, initVelocity,
						     endPosition, endVelocity,
						     doubleSupportStartTime,
						     doubleSupportEndTime));
  // add the new Double Support phase
  m_trajectory.push_back(newDoubleSupport);

  // add the new Single Support phase
  m_trajectory.push_back(newSingleSupport);
  
  return true;
}

bool DcmTrajectoryGenerator::addNewStep(const double &singleSupportStartTime,
					const double &singleSupportEndTime,
					const double &singleSupportDuration,
					const double &t0,
					const iDynTree::Vector2 &zmp)
{
  // get the next Single Support trajectory
  std::shared_ptr<GeneralSupportTrajectory> nextSingleSupport = m_trajectory.back();

  // get the domain of the next SingleSupport trajectory
  const std::pair<double, double> nextSubTrajectoryDomain = nextSingleSupport->getTrajectoryDomain();

  // get the DCM init of step parameter of the next SingleSupport trajectory
  std::shared_ptr<SingleSupportTrajectory> nextSingleSupport_cast = std::static_pointer_cast<SingleSupportTrajectory>(nextSingleSupport);
  iDynTree::Vector2 nextDcmIos = nextSingleSupport_cast->getDcmIos();
  
  // evaluate the new Single Support trajectory parameters
  std::shared_ptr<GeneralSupportTrajectory> currentSingleSupport(new SingleSupportTrajectory(singleSupportStartTime, singleSupportEndTime,
											     t0, singleSupportDuration, m_omega,
											     zmp, nextDcmIos));
  
  // instantiate position and velocity boundary conditions vectors
  iDynTree::Vector2 initPosition, initVelocity;
  iDynTree::Vector2 endPosition, endVelocity;

  // the begining time of the Double Support phase coinceds with the beginning of the next Single Support phase
  double doubleSupportEndTime = std::get<0>(nextSubTrajectoryDomain);

  // set the boundary conditions
  if(!currentSingleSupport->getDcmPos(singleSupportEndTime, initPosition)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM in the next SS phase is evaluated." <<std::endl;
    return false;
  }
  
  if(!currentSingleSupport->getDcmVel(singleSupportEndTime, initVelocity)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM in the next SS phase is evaluated." <<std::endl;
    return false;
  }
  
  if(!nextSingleSupport->getDcmPos(doubleSupportEndTime, endPosition)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM in the previous SS phase is evaluated." <<std::endl;
    return false;
  }

  if(!nextSingleSupport->getDcmVel(doubleSupportEndTime, endVelocity)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM in the previous SS phase is evaluated." <<std::endl;
    return false;
  }

  // evaluate the new Double Support phase
  double doubleSupportStartTime = singleSupportEndTime;
  std::shared_ptr<GeneralSupportTrajectory> currentDoubleSupport = nullptr;

  // check if pause features is active and if it is needed
  if (m_pauseActive && (doubleSupportEndTime - doubleSupportStartTime > m_maxDoubleSupportDuration)){
    // in this case the DS phase is splitted in three DS phase
    // In the first one the DCM of the robot has to reach the center of the feet convex hull
    // In the second one the robot has to stop (stance phase)
    // In the third one the DCM has to reach the "endPosition" with the "endVelocity"

    // evaluate the stance position constrain
    iDynTree::Vector2 stancePosition, stanceVelocity;
    stanceVelocity.zero();
    iDynTree::toEigen(stancePosition) = (iDynTree::toEigen(zmp) + iDynTree::toEigen(nextSingleSupport_cast->getZmp())) / 2;

    // evaluate the start and the end time of the Stance phase
    double doubleSupportStanceStartTime = doubleSupportStartTime + m_nominalDoubleSupportDuration / 2;
    double doubleSupportStanceEndTime = doubleSupportEndTime - m_nominalDoubleSupportDuration / 2;

    if(fabs(doubleSupportStanceStartTime - doubleSupportStanceEndTime) < MIN_DURATION_DOUBLE_SUPPORT_STANCE_PHASE){
      std::cerr << "[DCM TRAJECTORY GENERATOR] The duration of the DS stance phase has to be greater than " <<
	MIN_DURATION_DOUBLE_SUPPORT_STANCE_PHASE << "." << std::endl;
      return false;
    }    
    // add the 3-th part of the Double Support phase
    currentDoubleSupport.reset(new DoubleSupportTrajectory(stancePosition, stanceVelocity,
							   endPosition, endVelocity,
							   doubleSupportStanceEndTime,
							   doubleSupportEndTime));
    m_trajectory.push_back(currentDoubleSupport);

    // add 2-th part of the Double Support phase (stance phase)
    currentDoubleSupport.reset(new DoubleSupportTrajectory(stancePosition, stanceVelocity,
							   stancePosition, stanceVelocity,
							   doubleSupportStanceStartTime,
							   doubleSupportStanceEndTime));
    m_trajectory.push_back(currentDoubleSupport);

    // add 1-th part of the Double Support phase
    currentDoubleSupport.reset(new DoubleSupportTrajectory(initPosition, initVelocity,
							   stancePosition, stanceVelocity,
							   doubleSupportStartTime,
							   doubleSupportStanceStartTime));
    m_trajectory.push_back(currentDoubleSupport);
  }else{
    // add the new Double Support phase
    currentDoubleSupport.reset(new DoubleSupportTrajectory(initPosition, initVelocity,
							   endPosition, endVelocity,
							   doubleSupportStartTime,
							   doubleSupportEndTime));
    m_trajectory.push_back(currentDoubleSupport);
  }
  // add the new Single Support phase
  m_trajectory.push_back(currentSingleSupport);

  return true; 
}

bool DcmTrajectoryGenerator::addFirstDoubleSupportPhase(const double &doubleSupportStartTime,
							const iDynTree::Vector2 &initPosition,
							const iDynTree::Vector2 &initVelocity)
{
  // get the next Single Support trajectory
  std::shared_ptr<GeneralSupportTrajectory> nextSingleSupport = m_trajectory.back();

  // get the domain of the next SingleSupport trajectory
  const std::pair<double, double> nextSubTrajectoryDomain = nextSingleSupport->getTrajectoryDomain();

  // instantiate position and velocity boundary conditions vectors
  iDynTree::Vector2 endPosition, endVelocity;

  // the begining time of the Double Support phase coinceds with the beginning of the next Single Support phase
  double doubleSupportEndTime = std::get<0>(nextSubTrajectoryDomain);

  // set the boundary conditions
  if(!nextSingleSupport->getDcmPos(doubleSupportEndTime, endPosition)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM in the next SS phase is evaluated." <<std::endl;
    return false;
  }

  if(!nextSingleSupport->getDcmVel(doubleSupportEndTime, endVelocity)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM in the next SS phase is evaluated." <<std::endl;
    return false;
  }

  // evaluate the new Double Support phase
  std::shared_ptr<GeneralSupportTrajectory> currentDoubleSupport(new DoubleSupportTrajectory(initPosition, initVelocity,
											     endPosition, endVelocity,
											     doubleSupportStartTime,
											     doubleSupportEndTime));
  // add the new Double Support phase
  m_trajectory.push_back(currentDoubleSupport);
  return true;
}

void DcmTrajectoryGenerator::getLastStepsTiming(double &singleSupportStartTime,
						double &singleSupportEndTime,
						double &singleSupportDuration,
						double &doubleSupportEndTime,
						double &singleSupportT0)
{
  doubleSupportEndTime = m_phaseShift.back() * m_dT;
  m_phaseShift.pop_back();

  singleSupportEndTime = m_phaseShift.back() * m_dT;
  m_phaseShift.pop_back();
  
  singleSupportStartTime = m_phaseShift.back() * m_dT;
  m_phaseShift.pop_back();
 
  singleSupportT0  =  (singleSupportStartTime + m_phaseShift.back() * m_dT)/2;
  singleSupportDuration = singleSupportEndTime  - singleSupportT0;
}


void DcmTrajectoryGenerator::getStepsTiming(double &singleSupportStartTime,
					    double &singleSupportEndTime,
					    double &singleSupportDuration,
					    double &singleSupportT0)
{
  singleSupportEndTime = m_phaseShift.back() * m_dT;
  m_phaseShift.pop_back();
  
  singleSupportStartTime = m_phaseShift.back() * m_dT;
  m_phaseShift.pop_back();
  
  singleSupportT0  =  (singleSupportStartTime + m_phaseShift.back() * m_dT) / 2;

  std::shared_ptr<GeneralSupportTrajectory> lastDoubleSupportPhase = m_trajectory.back();
  std::pair<double, double> subtrajectoryDomain = lastDoubleSupportPhase->getTrajectoryDomain();
  double doubleSupportEndTime = std::get<1>(subtrajectoryDomain);
  singleSupportDuration = (singleSupportEndTime + doubleSupportEndTime) / 2 - singleSupportT0;
}



void DcmTrajectoryGenerator::getFirstDoubleSupportTiming(double &doubleSupportStartTime)
{
  doubleSupportStartTime = m_phaseShift.back() * m_dT;
  m_phaseShift.pop_back();
}


bool DcmTrajectoryGenerator::generateDcmTrajectory(const std::vector<StepList::const_iterator> &orderedSteps,
						   const StepList::const_iterator &firstStanceFoot,
						   const iDynTree::Vector2 &initPosition,
						   const iDynTree::Vector2 &initVelocity,
						   const std::vector<size_t> &phaseShift)
{

  m_trajectoryDomain = std::make_pair(phaseShift.front(), phaseShift.back());
  
  m_orderedSteps = orderedSteps;

  // reset the trajectory
  m_trajectory.clear();
  
  // add the first stance foot at the beginning of the orderedStep vector
  m_orderedSteps.insert(m_orderedSteps.begin(), firstStanceFoot);
  
  m_phaseShift = phaseShift;

  double singleSupportStartTime;
  double singleSupportEndTime;
  double singleSupportDuration;
  double doubleSupportEndTime;
  double singleSupportT0;

  // evaluate times for the last step
  getLastStepsTiming(singleSupportStartTime, singleSupportEndTime,
		     singleSupportDuration, doubleSupportEndTime,
		     singleSupportT0);


  iDynTree::Vector2 lastZmp = m_orderedSteps.back()->position;
  m_orderedSteps.pop_back();

  // evaluate the position of the Center of mass at the end of the trajectory 
  iDynTree::Vector2 comPosition;
  iDynTree::toEigen(comPosition)  = (iDynTree::toEigen(lastZmp) + iDynTree::toEigen(m_orderedSteps.back()->position)) / 2;

  lastZmp = m_orderedSteps.back()->position;
  m_orderedSteps.pop_back();

  // evaluate the last step
  if(!addLastStep(singleSupportStartTime, singleSupportEndTime,
		  singleSupportDuration, doubleSupportEndTime,
		  singleSupportT0,
		  comPosition, lastZmp)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the DCM trajectory of the last step is generated." << std::endl;
    return false;
  }
  
  while (m_orderedSteps.size() > 0){
    
    getStepsTiming(singleSupportStartTime, singleSupportEndTime,
  		   singleSupportDuration, singleSupportT0);

    
    lastZmp = m_orderedSteps.back()->position;
    m_orderedSteps.pop_back();
    
    if(!addNewStep(singleSupportStartTime, singleSupportEndTime,
  		   singleSupportDuration, singleSupportT0, 
  		   lastZmp)){
      std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the DCM trajectory of a new step is generated." << std::endl;
      return false;
    }
  }

  // evaluate the first Double support phase
  double doubleSupportStartTime;
  getFirstDoubleSupportTiming(doubleSupportStartTime);

  if(!addFirstDoubleSupportPhase(doubleSupportStartTime,
  				 initPosition,
  				 initVelocity)){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the DCM trajectory of the first DS phase is generated." << std::endl;
    return false;
  }

  // evaluate the DCM trajectory
  if(!evaluateDcmTrajectory()){
    std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the whole DCM trajectory is evaluated." << std::endl;
    return false;
  }

  return true;
}

bool DcmTrajectoryGenerator::evaluateDcmTrajectory()
{
  size_t timeVectorLength = std::get<1>(m_trajectoryDomain) - std::get<0>(m_trajectoryDomain);
  std::vector<size_t> timeVector(timeVectorLength);
  
  // populate the time vector
  std::iota(std::begin(timeVector), std::end(timeVector), std::get<0>(m_trajectoryDomain));

  // clear all the previous DCM position 
  m_dcmPos.clear();
  m_dcmPos.reserve(timeVectorLength);

  // clear all the previous DCM velocity 
  m_dcmVel.clear();
  m_dcmVel.reserve(timeVectorLength);

  iDynTree::Vector2 dcmPos, dcmVel;
  double time;
  std::vector<std::shared_ptr<GeneralSupportTrajectory>>::reverse_iterator subTrajectory = m_trajectory.rbegin();
  
  for (size_t t : timeVector){
    time = t * m_dT;
    double subTrajectoryEndTime  = std::get<1>((*subTrajectory)->getTrajectoryDomain());
    if (time > subTrajectoryEndTime){
      subTrajectory++;
    }
    
    if(!(*subTrajectory)->getDcmPos(time, dcmPos)){
      std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM is evaluated." << std::endl;
      return false;
    }

    if(!(*subTrajectory)->getDcmVel(time, dcmVel)){
      std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM is evaluated." << std::endl;
      return false;
    }
    m_dcmPos.push_back(dcmPos);
    m_dcmVel.push_back(dcmVel);
  }
  return true;
}

const std::vector<iDynTree::Vector2>& DcmTrajectoryGenerator::getDcmPosition() const
{
  return m_dcmPos;
}


const std::vector<iDynTree::Vector2>& DcmTrajectoryGenerator::getDcmVelocity() const
{
  return m_dcmVel;
}

bool DcmTrajectoryGenerator::setPauseConditions(const double &maxDoubleSupportDuration, const double &nominalDoubleSupportDuration)
{
  if (maxDoubleSupportDuration < 0){
    std::cerr << "[DCM TRAJECTORY GENERATOR] If the maxDoubleSupportDuration  is negative, the robot won't pause in middle stance." << std::endl;
    m_pauseActive = false;
  }
  
  m_pauseActive = true;
  m_maxDoubleSupportDuration = maxDoubleSupportDuration;
  
  if (m_pauseActive){
    if (nominalDoubleSupportDuration <= 0){
      std::cerr << "[DCM TRAJECTORY GENERATOR] The nominalDoubleSupportDuration is supposed to be positive." << std::endl;
      m_pauseActive = false;
      return false;
    }
    
    if (nominalDoubleSupportDuration > maxDoubleSupportDuration / 2){
      std::cerr << "[DCM TRAJECTORY GENERATOR] The nominalDoubleSupportDuration cannot be greater than maxDoubleSupportDuration." << std::endl;
      m_pauseActive = false;
      return false;
    }
  }
  m_nominalDoubleSupportDuration = nominalDoubleSupportDuration;

  return true;
}
