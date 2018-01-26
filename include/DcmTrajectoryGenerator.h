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

struct DCMTrajectoryPoint {
  double time;
  iDynTree::Vector2 DCMPosition;
  iDynTree::Vector2 DCMVelocity;
};

typedef struct DCMTrajectoryPoint DCMTrajectoryPoint;

/**
 * GeneralSupportTrajectory is a virtual class that represents the trajectory of the  Divergent Component
 * of Motion during a general support phase (Single or Double support).
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
   * @param DCMPosition cartesian position of the Diverget Component of Motion;
   * @param checkDomainCondition flag used to check if the time belongs to the trajectory domain (default value true).
   * @return true / false in case of success / failure.
   */
  virtual bool getDCMPosition(const double &t, iDynTree::Vector2& DCMPosition, const bool &checkDomainCondition = true) = 0;

  /**
   * Pure virtual method. It returns the velocity of the DCM 
   * trajectory evaluated at time t.
   * @param t is the trajectory evaluation time;
   * @param DCMVelocity cartesian velocity of the Diverget Component of Motion;
   * @param checkDomainCondition flag used to check if the time belongs to the trajectory domain (default value true).
   * @return true / false in case of success / failure.
   */
  virtual bool getDCMVelocity(const double &t, iDynTree::Vector2& DCMVelocity, const bool &checkDomainCondition = true) = 0;

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
 * SingleSupportTrajectory class represents the trajectory of the Divergent Component
 * of Motion during a single support phase.
 */
class SingleSupportTrajectory : public GeneralSupportTrajectory
{
 private:
  
  iDynTree::Vector2 m_ZMP; /**< Desired position of the ZMP at the beginning of the step */
  double m_omega; /**< Time constant of the 3D-LIPM */

  double m_boundaryConditionTime; /**< Absolute time of the DCM boundary condition */
  iDynTree::Vector2 m_boundaryConditionDCMPosition; /**< Boundary condition of the DCM trajectory */
      
 public:
  /**
   * Constructor.
   * @param startTime is the init time of the trajectory;
   * @param endTime is the end time of the trajectory;
   * @param omega time constant of the 3D-LIPM;
   * @param ZMP is the desired position of the ZMP in the SS phase trajejctory;
   * @param boundaryCondition is the boundary condition on the DCM trajectory.
   */
  SingleSupportTrajectory(const double &startTime,
			  const double &endTime,
			  const double &omega,
			  const iDynTree::Vector2 &ZMP,
			  const DCMTrajectoryPoint& boundaryCondition);
  /**
   * ZMP getter.
   * @return the position of the ZMP.
   */
  const iDynTree::Vector2& getZMP() const;
  
  /**
   * Implementation of the getDCMPosition method of the
   * GeneralSupportTrajectory class.
   * @param t is the trajectory evaluation time;
   * @param DCMPosition is the cartesian position of the Diverget Component of Motion;
   * @param checkDomainCondition flag used to check if the time belongs to the trajectory domain (default value true).
   * @return true / false in case of success / failure.
   */
  bool getDCMPosition(const double &t, iDynTree::Vector2& DCMPosition, const bool &checkDomainCondition = true) override;

  /**
   * Implementation of the getDCMVelocity method of the
   * GeneralSupportTrajectory class.
   * @param t is the trajectory evaluation time;
   * @param DCMVelocity cartesian velocity of the Diverget Component of Motion;
   * @param checkDomainCondition flag used to check if the time belongs to the trajectory domain (default value true).
   * @return true / false in case of success / failure.
   */
  bool getDCMVelocity(const double &t, iDynTree::Vector2& DCMVelocity, const bool &checkDomainCondition = true) override;
};

/**
 * DoubleSupportTrajectory class represents the trajectory of the Divergent Component
 * of Motion during a double support phase.
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
   * @param initBoundaryCondition desired init position and velocity of the 
   * double support trajectory
   * @param endBoundaryCondition desired final position and velocity of the 
   * double support trajectory
   */
  DoubleSupportTrajectory(const DCMTrajectoryPoint &initBoundaryCondition,
			  const DCMTrajectoryPoint &finalBoundaryCondition);
  
  /**
   * Implementation of the getDCMPosition method of the
   * GeneralSupportTrajectory class.
   * @param t is the trajectory evaluation time;
   * @param DCMPosition is the cartesian position of the Diverget Component of Motion;
   * @param checkDomainCondition flag used to check if the time belongs to the trajectory domain (default value true).
   * @return true / false in case of success / failure.
   */
  bool getDCMPosition(const double &t, iDynTree::Vector2& DCMPosition, const bool &checkDomainCondition = true) override;

  /**
   * Implementation of the getDCMVelocity method of the
   * GeneralSupportTrajectory class.
   * @param t is the trajectory evaluation time;
   * @param DCMVelocity is the cartesian velocity of the Diverget Component of Motion;
   * @param checkDomainCondition flag used to check if the time belongs to the trajectory domain (default value true).
   * @return true / false in case of success / failure.
   */
  bool getDCMVelocity(const double &t, iDynTree::Vector2& DCMVelocity, const bool &checkDomainCondition = true) override;
};

/**
 * DCMTrajectoryGenerator class that represents the Divergent Component of Motion trajectory 
 * during the whole walking patern.
 */
class DCMTrajectoryGenerator
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
  std::vector<iDynTree::Vector2> m_DCMPosition; /**< Vector containing the position of the DCM. */
  std::vector<iDynTree::Vector2> m_DCMVelocity; /**< Vector containing the velocity of the DCM. */

  /**
   * Evaluate the timings for the last step.
   * @param singleSupportStartTime start time of the Single Support trajectory;
   * @param singleSupportEndTime end time of the Single Support trajectory;
   * @param doubleSupportEndTime end time of the Single Support trajectory;
   * @param singleSupportBoundaryConditionTime boundary condition time of the single support trajectory.
   */
  void getLastStepsTiming(double &singleSupportStartTime,
			  double &singleSupportEndTime,
			  double &doubleSupportEndTime,
			  double &singleSupportBoundaryConditionTime);

  /**
   * Evaluate the timings for a general step.
   * @param singleSupportStartTime start time of the Single Support trajectory;
   * @param singleSupportEndTime end time of the Single Support trajectory;
   * @param singleSupportBoundaryConditionTime boundary condition time of the single support trajectory.
   */
  void getStepsTiming(double &singleSupportStartTime,
		      double &singleSupportEndTime,
		      double &singleSupportBoundaryConditionTime);  
  /**
   * Evaluate the timings for the first double support phase.
   * @param doubleSupportStartTime start time of the Double Support trajectory.
   */
  void getFirstDoubleSupportTiming(double &doubleSupportStartTime);

  /**
   * Add the last Single and Double support phases.
   * @param singleSupportStartTime start time of the Single Support trajectory;
   * @param singleSupportEndTime end time of the Single Support trajectory;
   * @param doubleSupportEndTime end time of the Single Support trajectory;
   * @param ZMP contains the position of the ZMP at the beginning of the single support phase,
   * it is assumed coincedent with the center of the reference foot;
   * @param singleSupportBoundaryCondition contains the boundary position and time
   * of the single support trajectory.
   * @return true / false in case of success / failure.
   */
  bool addLastStep(const double &singleSupportStartTime,
		   const double &singleSupportEndTime,
		   const double &doubleSupportEndTime,
		   const iDynTree::Vector2 &ZMP,
		   const DCMTrajectoryPoint& singleSupportBoundaryCondition);
		   
  /**
   * Add the Single and Double support phases for a general step.
   * @param singleSupportStartTime start time of the Single Support trajectory;
   * @param singleSupportEndTime end time of the Single Support trajectory;
   * @param ZMP contains the position of the ZMP at the beginning of the single support phase,
   * it is assumed coincedent with the center of the reference foot;
   * @param singleSupportBoundaryCondition contains the position and the time of the boundary
   * of the single support trajectory.
   * @return true / false in case of success / failure.
   */
  bool addNewStep(const double &singleSupportStartTime,
		  const double &singleSupportEndTime,
		  const iDynTree::Vector2 &ZMP,
		  const DCMTrajectoryPoint &singleSupportBoundaryCondition);
    /**
   * Add the Single and Double support phases for a general step
   * @param doubleSupportInitBoundaryCondition contains the boundary condition of the single support trajectory.
   * @return true / false in case of success / failure.
   */
  bool addFirstDoubleSupportPhase(const DCMTrajectoryPoint &doubleSupportInitBoundaryCondition);

  /**
   * Evaluate the DCM position for all time.
   * @return true / false in case of success / failure.
   */
  bool evaluateDCMTrajectory();

 public:

  /**
   * Constructor.
   * @param dT is the period of the Trajectory generator planner;
   * @param omega time constant of the 3D-LIPM.
   */
  DCMTrajectoryGenerator(const double &dT, const double &omega);

  /**
   * Constructor.
   */
  DCMTrajectoryGenerator();
  
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
  bool generateDCMTrajectory(const std::vector<StepList::const_iterator> &orderedSteps,
			     const StepList::const_iterator &firstStanceFoot,
			     const iDynTree::Vector2 &initPosition,
			     const iDynTree::Vector2 &initVelocity,
			     const std::vector<size_t> &phaseShift);

  /**
   * Get the position of the Divergent Component of Motion.
   * @return a vector containing the DCM position during all the trajectory domain.
   */  
  const std::vector<iDynTree::Vector2>& getDCMPosition() const;

  /**
   * Get the velocity of the Divergent Component of Motion.
   * @return a vector containing the DCM velocity during all the trajectory domain.
   */  
  const std::vector<iDynTree::Vector2>& getDCMVelocity() const;
};

#endif
