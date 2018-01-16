/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

// std
#include <vector>
#include <memory>
#include <deque>

// Eigen3
#include <Eigen/Dense>

#include "iDynTree/Core/VectorFixSize.h"

#include "FootPrint.h"

/**
 * Virtual class that represents a coordiante (x or y) of the Divergent Component
 * of Motion trajectory during a general support phase (Single or Double support)
 */
class GeneralSupportTrajectory
{
 protected:
  
  std::pair<double, double> m_trajectoryDomain; /**< Time domain of the trajectory */
  
 public:
  /**
   * Constructor.
   * @param start_time is the init time of the trajectory
   * @param end_time is the end time of the trajectory
   */
  GeneralSupportTrajectory(const double &start_time, const double &end_time);
  
  /**
   * Pure virtual method. It returns the position of the DCM 
   * trajectory evaluated at time t
   * @param t is the trajectory evaluation time
   * @param dcmPos cartesian position of the Diverget Component of Motion 
   * @return true / false in case of success / failure
   */
  virtual bool getDcmPos(const double &t, iDynTree::Vector2& dcmPos) = 0;

  /**
   * Pure virtual method. It returns the velocity of the DCM 
   * trajectory evaluated at time t
   * @param t is the trajectory evaluation time 
   * @param dcmVel cartesian velocity of the Diverget Component of Motion 
   * @return true / false in case of success / failure
   */
  virtual bool getDcmVel(const double &t, iDynTree::Vector2& dcmVel) = 0;

  /**
   * Return true if the time t belongs to the trajectory time 
   * domain (i.e t belongs (start_time, end_time))
   * @param t is the time  
   * @return true if start_time <= t <=  end_time, false otherwise
   */
  bool timeBelongsToDomain(const double &t);

  const std::pair<double, double>& getTrajectoryDomain() const;
};

/**
 * This class represents a coordiante (x or y) of the Divergent Component
 * of Motion trajectory during a single support phase
 */
class SingleSupportTrajectory : public GeneralSupportTrajectory
{
 private:
  
  iDynTree::Vector2 m_dcmIos; /**< desired position of the DCM at the beginning of the step */
  iDynTree::Vector2 m_zmp; /**< desired position of the ZMP at the beginning of the step */
  double m_omega; /**< time constant of the 3D-LIPM */

  /** Absolute "init" time of the Single Support phase
   * it is important to notice that t0 is equal to the absolute time of the previous half
   * DS phase
   * Note that m_t0 is NOT equal to the start_time (aka the time of the beginning
   * of the SS trajectory. Because the SS trajectory is obtained under the hypotesis of
   * no DS phase)
   */
  double m_t0;

  /**
   * Evaluate the desired position of the DCM at the beginning of the trajectory
   * @param nextDcmIos is the position of the DCM at the beginning of the next step
   * @param stepDuration is the duration of the step
   */
  void setDcmIos(const iDynTree::Vector2 &nextDcmIos,
		 const double &stepDuration);
    
 public:
  /**
   * Constructor.
   * @param startTime is the init time of the trajectory
   * @param endTime is the end time of the trajectory
   * @param zmp is the desired position of the ZMP in the SS phase trajejctory 
   * @param t0 is the "init" time of the SS phase
   * @param omega time constant of the 3D-LIPM
   */
  SingleSupportTrajectory(const double &startTime,
			  const double &endTime,
			  const double &t0,
			  const double &stepDuration,
			  const double &omega,
			  const iDynTree::Vector2 &zmp,
			  const iDynTree::Vector2 &nextDcmIos);


  const iDynTree::Vector2& getDcmIos() const;
		  
  /**
   * Implementation of the getDcmPos method of the
   * GeneralSupportTrajectory class
   * @param t is the trajectory evaluation time 
   * @param dcmPos cartesian position of the Diverget Component of Motion 
   * @return true / false in case of success / failure
   */
  bool getDcmPos(const double &t, iDynTree::Vector2& dcmPos) override;

  /**
   * Implementation of the getDcmVel method of the
   * GeneralSupportTrajectory class
   * @param t is the trajectory evaluation time 
   * @param dcmVel cartesian velocity of the Diverget Component of Motion 
   * @return true / false in case of success / failure
   */
  bool getDcmVel(const double &t, iDynTree::Vector2& dcmVel) override;
};


/**
 * This class represents a coordiante (x or y) of the Divergent Component
 * of Motion trajectory during a double support phase
 */
class DoubleSupportTrajectory : public GeneralSupportTrajectory
{
 private:
  Eigen::Vector4d m_coefficentsX; /**< 3-th order x-trajectory parameters [a3, a2, a1, a0] */
  Eigen::Vector4d m_coefficentsY; /**< 3-th order y-trajectory parameters [a3, a2, a1, a0] */

  /**
   * Given desired boundary conditions (position and velocity) evaluate the cofficents of a
   * 3-th order polynomial
   * @param positionBoundaryConds contains the desired value of the polinomial at the beginning 
   * and at the end of the Double Support phase
   * @param velocityBoundaryConds contains the desired value of the polinomial derivative at the beginning 
   * and at the end of the Double Support phase
   * @param dsDuration duration of the Double Support phase
   * @return the vector containing the coefficents of the 3-th order polynomial
   */
  Eigen::Vector4d polinominalInterpolation(const iDynTree::Vector2 &positionBoundaryConds, 
					   const iDynTree::Vector2 &velocityBoundaryConds,
					   const double &dsDuration);

 public:
  /**
   * Constructor.
   * @param position_boundary_conds_x contains the desired position of the DCM x-coordinate 
   * at the beginning and at the end of the Double Support phase
   * @param velocity_boundary_conds_x contains the desired velocity of the DCM x-coordinate
   * at the beginning and at the end of the Double Support phase
   * @param position_boundary_conds_y contains the desired position of the DCM y-coordinate 
   * at the beginning and at the end of the Double Support phase
   * @param velocity_boundary_conds_y contains the desired velocity of the DCM y-coordinate
   * at the beginning and at the end of the Double Support phase
   * @param ds_duration duration of the double support
   * @param start_time is the init time of the trajectory
   * @param end_time is the end time of the trajectory
   */
  DoubleSupportTrajectory(const iDynTree::Vector2 &initPosition, 
			  const iDynTree::Vector2 &initVelocity,
			  const iDynTree::Vector2 &endPosition, 
			  const iDynTree::Vector2 &endVelocity,
			  const double &startTime,
			  const double &endTime);
  

  /**
   * Implementation of the getDcmPos method of the
   * GeneralSupportTrajectory class
   * @param t is the trajectory evaluation time 
   * @param dcmPos cartesian position of the Diverget Component of Motion 
   * @return true / false in case of success / failure
   */
  bool getDcmPos(const double &t, iDynTree::Vector2& dcmPos) override;

  /**
   * Implementation of the getDcmVel method of the
   * GeneralSupportTrajectory class
   * @param t is the trajectory evaluation time 
   * @param dcmVel cartesian velocity of the Diverget Component of Motion 
   * @return true / false in case of success / failure
   */
  bool getDcmVel(const double &t, iDynTree::Vector2& dcmVel) override;
};


/**
 * This class represents the  the Divergent Component of Motion trajectory 
 * during the whole walking patern
 */
class DcmTrajectoryGenerator
{
 private:

  // TODO MOVE INTO A VECTOR?? 
  std::deque<std::shared_ptr<GeneralSupportTrajectory>> m_trajectory; /**< Vector containing pointer of every trajectory phase. */

  std::pair<size_t, size_t> m_trajectoryDomain;
  
  double m_dT; /**< Planner period. */
  double m_omega; /**< Time constant of the 3D-LIPM. */
  std::vector<StepList::const_iterator> m_orderedSteps;  /**< Vector containing the both left and right footprint sorted into ascending order. */
  std::vector<size_t> m_phaseShift; /**< Vector containing the index when a change of phase (SS -> DS and viceversa) occours. */
  std::vector<iDynTree::Vector2> m_dcmPos;   /**< Vector containing the position of the DCM . */

  /**
   * Return the subtrajectory such that the time t belongs to the domain
   * @param t is the trajectory evaluation time 
   * @return shared_ptr to the subtrajectory
   */
  std::shared_ptr<GeneralSupportTrajectory> findSubTrajectory(const double &t);

  /**
   * Evaluate the timings for the last step.
   * @param singleSupportStartTime start time of the Single Support trajectory
   * @param singleSupportEndTime end time of the Single Support trajectory
   * @param singleSupportDuration difference between the sungleSupportEndTime and singleSupportT0
   * @param doubleSupportEndTime end time of the Single Support trajectory
   * @param singleSupportT0 
   */
  void getLastStepsTiming(double &singleSupportStartTime,
			  double &singleSupportEndTime,
			  double &singleSupportDuration,
			  double &doubleSupportEndTime,
			  double &singleSupportT0);

  /**
   * Evaluate the timings for a general step
   * @param singleSupportStartTime start time of the Single Support trajectory
   * @param singleSupportEndTime end time of the Single Support trajectory
   * @param singleSupportDuration difference between the sungleSupportEndTime and singleSupportT0
   * @param singleSupportT0 
   */
  void getStepsTiming(double &singleSupportStartTime,
		      double &singleSupportEndTime,
		      double &singleSupportDuration,
		      double &singleSupportT0);
  
  /**
   * Evaluate the timings for the last 
   * @param doubleSupportStartTime start time of the Double Support trajectory
   */
  void getFirstDoubleSupportTiming(double &doubleSupportStartTime);


  /**
   * Add the last Single and Double support phases
   * @param singleSupportStartTime start time of the Single Support trajectory
   * @param singleSupportEndTime end time of the Single Support trajectory
   * @param singleSupportDuration difference between the sungleSupportEndTime and singleSupportT0
   * @param doubleSupportEndTime end time of the Single Support trajectory
   * @param singleSupportT0 
   * @param comPosition contains the desired position of the CoM at the end of the last step
   * @param zmp contains the position of the ZMP at the beginning of the single support phase,
   * it is assumed coincedent with the center of the reference foot
   * @return true / false in case of success / failure
   */
  bool addLastStep(const double &singleSupportStartTime,
		   const double &singleSupportEndTime,
		   const double &singleSupportDuration,
		   const double &doubleSupportEndTime,
		   const double &singleSupportT0,
		   const iDynTree::Vector2 &comPosition,
		   const iDynTree::Vector2 &zmp);

  /**
   * Add the Single and Double support phases for a general steo
   * @param singleSupportStartTime start time of the Single Support trajectory
   * @param singleSupportEndTime end time of the Single Support trajectory
   * @param singleSupportDuration difference between the sungleSupportEndTime and singleSupportT0
   * @param singleSupportT0 
   * @param zmp contains the position of the ZMP at the beginning of the single support phase,
   * it is assumed coincedent with the center of the reference foot
   * @return true / false in case of success / failure
   */
  bool addNewStep(const double &singleSupportStartTime,
		  const double &singleSupportEndTime,
		  const double &singleSupportDuration,
		  const double &singleSupportT0,
		  const iDynTree::Vector2 &zmp);

  /**
   * Add the Single and Double support phases for a general steo
   * @param doubleSupportStartTime start time of the first Double Support trajectory
   * @param initPosition position of the DCM at the beginning of the Double Support trajectory
   * @param initVelocity velocity of the DCM at the beginning of the Double Support trajectory
   * @return true / false in case of success / failure
   */
  bool addFirstDoubleSupportPhase(const double &doubleSupportStartTime,
				  const iDynTree::Vector2 &initPosition,
				  const iDynTree::Vector2 &initVelocity);  

  /**
   * Evaluate the DCM position an time t
   * @param t is the trajectory evaluation time
   * @param dcmPos cartesian position of the Diverget Component of Motion 
   * @return true / false in case of success / failure
   */
  bool evaluateDcmPosition(const size_t &t, iDynTree::Vector2 &dcmPos);

  /**
   * Evaluate the DCM position for all time 
   * @return true / false in case of success / failure
   */
  bool evaluateDcmPosition();

 public:

  /**
   * Constructor.
   * @param dT is the period of the Trajectory generator planner
   * @param omega time constant of the 3D-LIPM
   */
  DcmTrajectoryGenerator(const double &dT, const double &omega);

  /**
   * Constructor.
   */
  DcmTrajectoryGenerator(){};

  void setOmega(const double &omega);


  void setdT(const double &dT);

  
  /**
   * Evaluate the position of the DCM at time t for the general
   * trajectory.
   * @return true / false in case of success / failure
   */
  bool getDcmPos(std::vector<StepList::const_iterator> &dcmPos);

  /**
   * Generate the Divergent Component of Motion trajectory
   * @param orderedSteps vector containing the both left and right footprint sorted into ascending order
   * @param firstStanceFoot is the footprint of the first stance foot
   * @param firstSwingFoot is the footprint of the first swing foot
   * @param phaseShift vector containing the index when a change of phase (SS -> DS and viceversa) occours.
   * @return true / false in case of success / failure
   */
  bool generateDcmTrajectory(const std::vector<StepList::const_iterator> &orderedSteps,
			     const StepList::const_iterator &firstStanceFoot,
			     const StepList::const_iterator &firstSwingFoot,
			     const std::vector<size_t> &phaseShift);


  /**
   * Get the position of the Divergent Component of Motion 
   * @return a vector containing the DCM position during all the trajectory domain
   */  
  const std::vector<iDynTree::Vector2>& getDcmPosition() const;
};

#endif
