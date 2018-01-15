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

// Eigen3
#include <Eigen/Dense>


#include "iDynTree/Core/VectorFixSize.h"

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

  /** absolute "init" time of the Single Support phase
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
  void evalDcmIos(const iDynTree::Vector2 &nextDcmIos,
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
			  const double &omega,
			  const iDynTree::Vector2 &zmp);
  
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
  Eigen::Vector4d polinominalInterpolation(const std::vector<double> &positionBoundaryConds, 
					   const std::vector<double> &velocityBoundaryConds,
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
  DoubleSupportTrajectory(const std::vector<double> &position_boundary_conds_x, 
			  const std::vector<double> &velocity_boundary_conds_x,
			  const std::vector<double> &position_boundary_conds_y, 
			  const std::vector<double> &velocity_boundary_conds_y,
			  const double &start_time,
			  const double &end_time);
  

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


#endif
