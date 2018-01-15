/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later
 */

#include <math.h>
#include <vector>

#include <Eigen/Dense>

#include "iDynTree/Core/VectorFixSize.h"
#include "iDynTree/Core/EigenHelpers.h"

#include "DcmTrajectoryGenerator.h"


//dedbug
#include <iostream>

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

  return false;
}

SingleSupportTrajectory::SingleSupportTrajectory(const double &startTime,
						 const double &endTime,
						 const double &t0,
						 const double &omega,
						 const iDynTree::Vector2 &zmp):
  GeneralSupportTrajectory(startTime, endTime),
  m_t0(t0),
  m_omega(omega),
  m_zmp(zmp)
{}

void SingleSupportTrajectory::evalDcmIos(const iDynTree::Vector2 &nextDcmIos,
					 const double &stepDuration)
{
  // The position of the DCM at the beginning of the i-th step
  // depends on the position of the DCM at the beginning of the steps i+1
  // dcm_ios[i] = dcm_ios[i+1]
  //              + exp(-omega * Ts[i]) * (1 - exp(-omega * Ts[i])) * zmp[i]
  
  iDynTree::toEigen(m_dcmIos) = iDynTree::toEigen(m_zmp)
    + exp(-m_omega * stepDuration) * (iDynTree::toEigen(nextDcmIos) - iDynTree::toEigen(m_zmp));
}


// SingleSupportTrajectory::SingleSupportTrajectory(const std::vector<double> &footsteps_position, 
// 						 const std::vector<double> &impact_times,
// 						 const double &start_time,
// 						 const double &end_time,
// 						 const double &omega):
//   GeneralSupportTrajectory(start_time, end_time),
//   m_omega(omega)
// {

//   // Absolute init time of the Single Support phase
//   // it is important to notice that t0 is equal to the impact_time of the current
//   // Support foot and it is NOT equal to the start_time (aka the time of the beginning
//   // of the SS trajectory. Because the SS trajectory is obtained under the hypotesis of
//   // no DS phase. However in order to take into account the DS phase the duration of the SS
//   // phase is less than duration of the step (impact_times[i + 1] - impact_times[i])
//   m_t0 = impact_times[0];
  
//   // The Zero Momentum Point is assumed constant and it coincides with
//   // center of the footstep
//   m_zmp = footsteps_position[0];

//   // Evaluate the desired position of the DCM at the beginning of the step.
//   // It is evaluated recursively
//   // The position of the last dcm_ios is assumed to be coincident with the
//   // position of the last footstep
//   dcm_ios = footsteps_position.back();
//   for(int i = impact_times.size() - 2; i >= 0; i --)
//     {
//       double step_duration = impact_times[i + 1] - impact_times[i];
//       dcm_ios = evalDcmIos(dcm_ios, step_duration, footsteps_position[i]);
//     }
//   return;      
// }

bool SingleSupportTrajectory::getDcmPos(const double &t, iDynTree::Vector2 &dcmPos)
{
  // Evaluate the position of the DCM at time t
  if (timeBelongsToDomain(t)){
    iDynTree::toEigen(dcmPos) = iDynTree::toEigen(m_zmp)
      + exp(m_omega * (t - m_t0)) * (iDynTree::toEigen(m_dcmIos) - iDynTree::toEigen(m_zmp));
    return true;
  }
  std::cerr << "[Single support trajectory] the time t: " << t << "does not belong to the trajectory domain" <<
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
  std::cerr << "[Single support trajectory] the time t: " << t << "does not belong to the trajectory domain" <<
    std::endl;
  return false;  
}

DoubleSupportTrajectory::DoubleSupportTrajectory(const std::vector<double> &positionBoundaryCondsX, 
						 const std::vector<double> &velocityBoundaryCondsX,
						 const std::vector<double> &positionBoundaryCondsY, 
						 const std::vector<double> &velocityBoundaryCondsY,
						 const double &startTime,
						 const double &endTime):
  GeneralSupportTrajectory(startTime, endTime)
{

  double dsDuration = endTime - startTime;
  
  // evaluate the trajectories parameters
  m_coefficentsX = polinominalInterpolation(positionBoundaryCondsX, velocityBoundaryCondsX,
					    dsDuration);

  m_coefficentsY = polinominalInterpolation(positionBoundaryCondsY, velocityBoundaryCondsY,
					    dsDuration);

  return;      
}

Eigen::Vector4d DoubleSupportTrajectory::polinominalInterpolation(const std::vector<double> &positionBoundaryConds, 
								  const std::vector<double> &velocityBoundaryConds,
								  const double &dsDuration)
{

  // evaluate the coefficent of a 3-th order polinomial
  // p(t) = a3 * t^3 + a2 * t^2 + a1 * t + a0
  Eigen::Vector4d coefficents;
  Eigen::Vector4d boundaryCond;
  Eigen::Matrix4d estimationMatrix;

  // set boundary condition
  boundaryCond << positionBoundaryConds[0],
    velocityBoundaryConds[0],
    positionBoundaryConds[1],
    velocityBoundaryConds[1];

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
  
  std::cerr << "[Double support trajectory] the time t: " << t << "does not belong to the trajectory domain" <<
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
  
  std::cerr << "[Double support trajectory] the time t: " << t << "does not belong to the trajectory domain" <<
    std::endl;
  return false;  
}




/**
 * This class represents the sequences of the phases of a coordiante (x or y) 
 * of the Divergent Component of Motion trajectory during the whole walking patern
 */
class DCMTrajectoryGeneratorCoordinate
{
private:

  /**
   * vector containing pointer of every subtrajectory trajectory phase
   */
  std::vector<std::shared_ptr<GeneralSupportTrajectory>> m_trajectory;

  double m_endSwitch; /**< duration of the last double support phase */
  double m_doubleSupportPercentage; /**< percentage of the double support phase */

  
  /**
   * Return the subtrajectory such that the time t belongs to the domain
   * @param t is the trajectory evaluation time 
   * @return shared_ptr to the subtrajectory
   */  
  std::shared_ptr<GeneralSupportTrajectory> findSubTrajectory(const double &t);

  
public:

  void addNewFootprints(const std::vector<double> &footstepsPosition,
			const std::vector<double> &impactTimes);

  /**
   * Evaluate the position of the DCM at time t for the general
   * trajectory.
   * @param t is the trajectory evaluation time
   * @return the position of the DCM @t
   */
  double getDcmPos(const double &t);

  
  
  /**
   * Add a new Double and Single support phases
   * @param footsteps_position is vector containing the position of the footsteps 
   * @param impact_times is vector containing the impact time of each footstep
   * @param double_support_init_time is the absolute time of the beginning of the Double
   *       Support phase
   * @param double_support_end_time is the absolute time of the end of the Double
   *       Support phase (note that this time is also the beginning of the Single
   *       Support phase)
   * @param single_support_end_time is the absolute time of the end of the Single
   *       Support phase
   * @param omega is the time constant of the 3D-LIPM
   */
  void addNewStep(const std::vector<double> &footsteps_position,
		  const std::vector<double> &impact_times,
		  const double &double_support_init_time,
		  const double &double_support_end_time,
		  const double &single_support_end_time,
		  const double &omega);

  /**
   * Add the firsts Double and Single support phases
   * @param footsteps_position is vector containing the position of the footsteps 
   * @param impact_times is vector containing the impact time of each footstep
   * @param com_position contains the position of the CoM at the beginning of the first step
   * @param double_support_init_time is the absolute time of the beginning of the Double
   *       Support phase
   * @param double_support_end_time is the absolute time of the end of the Double
   *       Support phase (note that this time is also the beginning of the Single
   *       Support phase)
   * @param single_support_end_time is the absolute time of the end of the Single
   *       Support phase
   * @param omega is the time constant of the 3D-LIPM
   */
  void addFirstStep(const std::vector<double> &footsteps_position,
		    const std::vector<double> &impact_times,
		    const double &com_position,
		    const double &double_support_init_time,
		    const double &double_support_end_time,
		    const double &single_support_end_time,
		    const double &omega);

  /**
   * Add the firsts Double and Single support phases
   * @param com_position contains the position of the CoM at the beginning of the first step
   * @param double_support_init_time is the absolute time of the beginning of the Double
   *       Support phase
   * @param double_support_end_time is the absolute time of the end of the Double
   *       Support phase (note that this time is also the beginning of the Single
   *       Support phase)
   */  
  void addLastStep(const double &com_position,
		   const double &double_support_init_time,
		   const double &double_support_end_time);
};
