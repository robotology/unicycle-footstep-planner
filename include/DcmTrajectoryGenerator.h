/**
 * @file DcmTrajectoryGenerator.h
 * @author Giulio Romualdi
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef DCMTRAJECTORYGENERATOR_H
#define DCMTRAJECTORYGENERATOR_H

// std
#include <vector>
#include <memory>
#include <deque>

// Eigen3
#include <Eigen/Dense>

// iDynTree
#include "iDynTree/Core/VectorFixSize.h"

#include "FootPrint.h"

# define MIN_DURATION_DOUBLE_SUPPORT_STANCE_PHASE 0.001

/**
 * Virtual class that represents a coordiante (x or y) of the Divergent Component
 * of Motion trajectory during a general support phase (Single or Double support).
 */
class GeneralSupportTrajectory
{
 protected:
  
  std::pair<double, double> m_trajectoryDomain; /**< Time domain of the trajectory */
  
 public:
  /**
   * Constructor.
   * @param startTime is the start time of the trajectory;
   * @param endTime is the end time of the trajectory.
   */
  GeneralSupportTrajectory(const double &startTime, const double &endTime);
  
  /**
   * Pure virtual method. It returns the position of the DCM 
   * trajectory evaluated at time t.
   * @param t is the trajectory evaluation time;
   * @param dcmPos cartesian position of the Diverget Component of Motion.
   * @return true / false in case of success / failure.
   */
  virtual bool getDcmPos(const double &t, iDynTree::Vector2& dcmPos) = 0;

  /**
   * Pure virtual method. It returns the velocity of the DCM 
   * trajectory evaluated at time t.
   * @param t is the trajectory evaluation time;
   * @param dcmVel cartesian velocity of the Diverget Component of Motion.
   * @return true / false in case of success / failure.
   */
  virtual bool getDcmVel(const double &t, iDynTree::Vector2& dcmVel) = 0;

  /**
   * Return true if the time t belongs to the trajectory time 
   * domain (i.e t belongs (startTime, endTime)).
   * @param t is the time.
   * @return true if startTime <= t <=  endTime, false otherwise.
   */
  bool timeBelongsToDomain(const double &t);

  /**
   * Get the trajectory domain.
   * @return time domain of the trajectory
   */
  const std::pair<double, double>& getTrajectoryDomain() const;
};

/**
 * This class represents a coordiante (x or y) of the Divergent Component
 * of Motion trajectory during a single support phase.
 */
class SingleSupportTrajectory : public GeneralSupportTrajectory
{
 private:
  
  iDynTree::Vector2 m_dcmIos; /**< Desired position of the DCM at the beginning of the step */
  iDynTree::Vector2 m_zmp; /**< Desired position of the ZMP at the beginning of the step */
  double m_omega; /**< Time constant of the 3D-LIPM */

  /** 
   * Absolute "init" time of the Single Support phase
   * it is important to notice that t0 is equal to the absolute time of the previous half
   * DS phase. 
   * Note that m_t0 is NOT equal to the start_time (aka the time of the beginning
   * of the SS trajectory. Because the SS trajectory is obtained under the hypotesis of
   * no DS phase).
   */
  double m_t0;

  /**
   * Evaluate the position of the DCM at the beginning of the trajectory.
   * @param nextDcmIos is the position of the DCM at the beginning of the next step;
   * @param stepDuration is the duration of the step.
   */
  void setDcmIos(const iDynTree::Vector2 &nextDcmIos,
		 const double &stepDuration);
    
 public:
  /**
   * Constructor.
   * @param startTime is the init time of the trajectory;
   * @param endTime is the end time of the trajectory;
   * @param zmp is the desired position of the ZMP in the SS phase trajejctory;
   * @param t0 is the "init" time of the SS phase;
   * @param omega time constant of the 3D-LIPM.
   */
  SingleSupportTrajectory(const double &startTime,
			  const double &endTime,
			  const double &t0,
			  const double &stepDuration,
			  const double &omega,
			  const iDynTree::Vector2 &zmp,
			  const iDynTree::Vector2 &nextDcmIos);


  /**
   * DCM init of step getter.
   * @return the position of the DCM at the beginning of SS phase.
   */
  const iDynTree::Vector2& getDcmIos() const;

  /**
   * ZMP getter.
   * @return the position of the ZMP.
   */
  const iDynTree::Vector2& getZmp() const;
  
  /**
   * Implementation of the getDcmPos method of the
   * GeneralSupportTrajectory class.
   * @param t is the trajectory evaluation time;
   * @param dcmPos is the cartesian position of the Diverget Component of Motion.
   * @return true / false in case of success / failure.
   */
  bool getDcmPos(const double &t, iDynTree::Vector2& dcmPos) override;

  /**
   * Implementation of the getDcmVel method of the
   * GeneralSupportTrajectory class.
   * @param t is the trajectory evaluation time;
   * @param dcmVel is cartesian velocity of the Diverget Component of Motion;
   * @return true / false in case of success / failure.
   */
  bool getDcmVel(const double &t, iDynTree::Vector2& dcmVel) override;
};

/**
 * This class represents a coordiante (x or y) of the Divergent Component
 * of Motion trajectory during a double support phase.
 */
class DoubleSupportTrajectory : public GeneralSupportTrajectory
{
 private:
  Eigen::Vector4d m_coefficentsX; /**< 3-th order x-trajectory parameters [a3, a2, a1, a0] */
  Eigen::Vector4d m_coefficentsY; /**< 3-th order y-trajectory parameters [a3, a2, a1, a0] */

  /**
   * Given desired boundary conditions (position and velocity) evaluate the cofficents of a
   * 3-th order polynomial.
   * @param positionBoundaryConds contains the desired values of the polinomial at the beginning 
   * and at the end of the Double Support phase;
   * @param velocityBoundaryConds contains the desired value of the polinomial derivative at the beginning 
   * and at the end of the Double Support phase;
   * @param dsDuration duration of the Double Support phase.
   * @return the vector containing the coefficents of the 3-th order polynomial.
   */
  Eigen::Vector4d polinominalInterpolation(const iDynTree::Vector2 &positionBoundaryConds, 
					   const iDynTree::Vector2 &velocityBoundaryConds,
					   const double &dsDuration);

 public:
  /**
   * Constructor.
   * @param initPosition vector containing the position at the beginning of the
   * Double Support phase;
   * @param initVelocity vector containing the velocity at the beginning of the
   * Double Support phase;
   * @param endPosition vector containing the position at the end of the
   * Double Support phase;
   * @param endVelocity vector containing the velocity at the end of the
   * Double Support phase;
   * @param startTime is the start time of the trajectory;
   * @param endTime is the end time of the trajectory.
   */
  DoubleSupportTrajectory(const iDynTree::Vector2 &initPosition, 
			  const iDynTree::Vector2 &initVelocity,
			  const iDynTree::Vector2 &endPosition, 
			  const iDynTree::Vector2 &endVelocity,
			  const double &startTime,
			  const double &endTime);
  
  /**
   * Implementation of the getDcmPos method of the
   * GeneralSupportTrajectory class.
   * @param t is the trajectory evaluation time;
   * @param dcmPos is the cartesian position of the Diverget Component of Motion.
   * @return true / false in case of success / failure.
   */
  bool getDcmPos(const double &t, iDynTree::Vector2& dcmPos) override;

  /**
   * Implementation of the getDcmVel method of the
   * GeneralSupportTrajectory class.
   * @param t is the trajectory evaluation time;
   * @param dcmVel is the cartesian velocity of the Diverget Component of Motion.
   * @return true / false in case of success / failure.
   */
  bool getDcmVel(const double &t, iDynTree::Vector2& dcmVel) override;
};

/**
 * This class represents the Divergent Component of Motion trajectory 
 * during the whole walking patern
 */
class DcmTrajectoryGenerator
{
 private:

  double m_dT; /**< Planner period. */
  double m_omega; /**< Time constant of the 3D-LIPM. */
    
  double m_maxDoubleSupportDuration; /**< Max duration of a DS phase. */ 
  double m_nominalDoubleSupportDuration; /**< Nominal duration of a DS phase. */
  bool m_pauseActive; /**< True if the pause feature is activate. */
  
  std::vector<std::shared_ptr<GeneralSupportTrajectory>> m_trajectory; /**< Vector containing pointer of every trajectory phase. */
  std::pair<size_t, size_t> m_trajectoryDomain; /**< Trajectory domain. */
 
  std::vector<StepList::const_iterator> m_orderedSteps;  /**< Vector containing the both left and right footprint sorted into ascending order. */
  std::vector<size_t> m_phaseShift; /**< Vector containing the index when a change of phase (SS -> DS and viceversa) occours. */
  std::vector<iDynTree::Vector2> m_dcmPos; /**< Vector containing the position of the DCM. */
  std::vector<iDynTree::Vector2> m_dcmVel; /**< Vector containing the velocity of the DCM. */

  /**
   * Evaluate the timings for the last step.
   * @param singleSupportStartTime start time of the Single Support trajectory;
   * @param singleSupportEndTime end time of the Single Support trajectory;
   * @param singleSupportDuration difference between the sungleSupportEndTime and singleSupportT0;
   * @param doubleSupportEndTime end time of the Single Support trajectory;
   * @param singleSupportT0 "fictitious" init time of the Single Support trajectory.
   */
  void getLastStepsTiming(double &singleSupportStartTime,
			  double &singleSupportEndTime,
			  double &singleSupportDuration,
			  double &doubleSupportEndTime,
			  double &singleSupportT0);

  /**
   * Evaluate the timings for a general step.
   * @param singleSupportStartTime start time of the Single Support trajectory;
   * @param singleSupportEndTime end time of the Single Support trajectory;
   * @param singleSupportDuration difference between the sungleSupportEndTime and singleSupportT0;
   * @param singleSupportT0 "fictitious" init time of the Single Support trajectory.
   */
  void getStepsTiming(double &singleSupportStartTime,
		      double &singleSupportEndTime,
		      double &singleSupportDuration,
		      double &singleSupportT0);
  
  /**
   * Evaluate the timings for the first double support phase.
   * @param doubleSupportStartTime start time of the Double Support trajectory.
   */
  void getFirstDoubleSupportTiming(double &doubleSupportStartTime);

  /**
   * Add the last Single and Double support phases.
   * @param singleSupportStartTime start time of the Single Support trajectory;
   * @param singleSupportEndTime end time of the Single Support trajectory;
   * @param singleSupportDuration difference between the sungleSupportEndTime and singleSupportT0;
   * @param doubleSupportEndTime end time of the Single Support trajectory;
   * @param singleSupportT0 "fictitious" init time of the Single Support trajectory;
   * @param comPosition contains the desired position of the CoM at the end of the last step;
   * @param zmp contains the position of the ZMP at the beginning of the single support phase,
   * it is assumed coincedent with the center of the reference foot.
   * @return true / false in case of success / failure.
   */
  bool addLastStep(const double &singleSupportStartTime,
		   const double &singleSupportEndTime,
		   const double &singleSupportDuration,
		   const double &doubleSupportEndTime,
		   const double &singleSupportT0,
		   const iDynTree::Vector2 &comPosition,
		   const iDynTree::Vector2 &zmp);

  /**
   * Add the Single and Double support phases for a general step.
   * @param singleSupportStartTime start time of the Single Support trajectory;
   * @param singleSupportEndTime end time of the Single Support trajectory;
   * @param singleSupportDuration difference between the sungleSupportEndTime and singleSupportT0;
   * @param singleSupportT0 "fictitious" init time of the Single Support trajectory;
   * @param zmp contains the position of the ZMP at the beginning of the single support phase,
   * it is assumed coincedent with the center of the reference foot.
   * @return true / false in case of success / failure.
   */
  bool addNewStep(const double &singleSupportStartTime,
		  const double &singleSupportEndTime,
		  const double &singleSupportDuration,
		  const double &singleSupportT0,
		  const iDynTree::Vector2 &zmp);

  /**
   * Add the Single and Double support phases for a general step
   * @param doubleSupportStartTime start time of the first Double Support trajectory;
   * @param initPosition position of the DCM at the beginning of the Double Support trajectory;
   * @param initVelocity velocity of the DCM at the beginning of the Double Support trajectory.
   * @return true / false in case of success / failure.
   */
  bool addFirstDoubleSupportPhase(const double &doubleSupportStartTime,
				  const iDynTree::Vector2 &initPosition,
				  const iDynTree::Vector2 &initVelocity);  

  /**
   * Evaluate the DCM position for all time.
   * @return true / false in case of success / failure.
   */
  bool evaluateDcmTrajectory();

 public:

  /**
   * Constructor.
   * @param dT is the period of the Trajectory generator planner;
   * @param omega time constant of the 3D-LIPM.
   */
  DcmTrajectoryGenerator(const double &dT, const double &omega);

  /**
   * Constructor.
   */
  DcmTrajectoryGenerator();
  
  /**
   * Set the time constant of the 3D-LIPM-
   * @param omega is the time constant of the 3D-LIPM.
   */  
  void setOmega(const double &omega);

  /**
   * Set the period of the Trajectory generator planner
   * @param dT is the period (in seconds) of the Trajectory generator planner
   */  
  void setdT(const double &dT);

  /**
   * Set the pause condition.
   * @param maxDoubleSupportDuration is the maximum duration of a DS phase;
   * @param nominalDoubleSupportDuration is the nominal duration of a DS phase.
   * @return true if the pause conditions are set, false otherwise.
   */  
  bool setPauseConditions(const double &maxDoubleSupportDuration, const double &nominalDoubleSupportDuration);
  
  /**
   * Generate the Divergent Component of Motion trajectory.
   * @param orderedSteps vector containing the both left and right footprint sorted into ascending impactTime order;
   * @param firstStanceFoot is the footprint of the first stance foot;
   * @param initPosition is the position of the DCM at the beginning of the trajectory;
   * @param initVelocity is the velocity of the DCM at the beginning of the trajectory;
   * @param phaseShift vector containing the index when a change of phase (SS -> DS and viceversa) occours.
   * @return true / false in case of success / failure.
   */
  bool generateDcmTrajectory(const std::vector<StepList::const_iterator> &orderedSteps,
			     const StepList::const_iterator &firstStanceFoot,
			     const iDynTree::Vector2 &initPosition,
			     const iDynTree::Vector2 &initVelocity,
			     const std::vector<size_t> &phaseShift);

  /**
   * Get the position of the Divergent Component of Motion.
   * @return a vector containing the DCM position during all the trajectory domain.
   */  
  const std::vector<iDynTree::Vector2>& getDcmPosition() const;

  /**
   * Get the velocity of the Divergent Component of Motion.
   * @return a vector containing the DCM velocity during all the trajectory domain.
   */  
  const std::vector<iDynTree::Vector2>& getDcmVelocity() const;
};

#endif
