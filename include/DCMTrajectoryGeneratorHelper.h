/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef DCMTRAJECTORYGENERATORHELPER_H
#define DCMTRAJECTORYGENERATORHELPER_H


// std
#include <vector>
#include <memory>
#include <deque>

// Eigen3
#include <Eigen/Dense>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>

#include <FootPrint.h>
#include <StepPhase.h>

typedef struct {
    double time;
    iDynTree::Vector2 DCMPosition;
    iDynTree::Vector2 DCMVelocity;
} DCMTrajectoryPoint;

/**
 * GeneralSupportTrajectory is a virtual class that represents the trajectory of the  Divergent Component
 * of Motion during a general support phase (Single or Double support).
 */
class GeneralSupportTrajectory
{
 protected:

    std::pair<double, double> m_trajectoryDomain; /**< Time domain of the trajectory */
    double m_omega; /**< Time constant of the 3D-LIPM */

 public:
    /**
     * Constructor.
     * @param startTime is the start time of the trajectory;
     * @param endTime is the end time of the trajectory;
     * @param omega time constant of the linear inverted pendulum.
     */
    GeneralSupportTrajectory(const double &startTime, const double &endTime, const double& omega);

    virtual ~GeneralSupportTrajectory();

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
     * Pure virtual method. It returns the position of the ZMP
     * trajectory evaluated at time t.
     * @param t is the trajectory evaluation time;
     * @param ZMPPosition cartesian position of the Zero Moment Point;
     * @param checkDomainCondition flag used to check if the time belongs to the trajectory domain (default value true).
     * @return true / false in case of success / failure.
     */
    virtual bool getZMPPosition(const double &t, iDynTree::Vector2& ZMPVelocity, const bool &checkDomainCondition = true) = 0;

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
 * DCMTrajectoryGenerator class that represents the Divergent Component of Motion trajectory
 * during the whole walking patern.
 */
class DCMTrajectoryGeneratorHelper
{
 private:

    double m_dT; /**< Planner period. */
    double m_omega; /**< Time constant of the 3D-LIPM. */
    iDynTree::Vector2 m_leftZMPDelta; /**< Vector containing the desired left ZMP delta. */
    iDynTree::Vector2 m_rightZMPDelta; /**< Vector containing the desired left ZMP delta. */

    double m_maxDoubleSupportDuration; /**< Max duration of a DS phase. */
    double m_nominalDoubleSupportDuration; /**< Nominal duration of a DS phase. */
    bool m_pauseActive; /**< True if the pause feature is activate. */

    std::vector<std::shared_ptr<GeneralSupportTrajectory>> m_trajectory; /**< Vector containing pointer of every trajectory phase. */
    std::pair<size_t, size_t> m_trajectoryDomain; /**< Trajectory domain. */

    std::vector<iDynTree::Vector2> m_DCMPosition; /**< Vector containing the position of the DCM. */
    std::vector<iDynTree::Vector2> m_DCMVelocity; /**< Vector containing the velocity of the DCM. */
    std::vector<iDynTree::Vector2> m_ZMPPosition; /**< Vector containing the position of the ZMP. */
    std::vector<double> m_weightInLeft, m_weightInRight; /**< Vectors containing the percentage of weight carried by the foot. */


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
     * @param firstSwingFoot contains the position and angle of the first swing foot before it starts to swing
     * @return true / false in case of success / failure.
     */
    bool addFirstDoubleSupportPhase(const DCMTrajectoryPoint &doubleSupportInitBoundaryCondition,
                                    const Step *firstSwingFoot);


    /**
     * Evaluate the ZMP position using the DCM Position and Velocity
     * @param DCMPosition position of the DCM;
     * @param DCMVelocity velocity of the DCM;
     * @return position of the ZMP
     */
    iDynTree::Vector2 evaluateZMPPosition(const iDynTree::Vector2 &DCMPosition,
                                          const iDynTree::Vector2 &DCMVelocity);
    /**
     * Evaluate the DCM position for all time.
     * @return true / false in case of success / failure.
     */
    bool evaluateDCMTrajectory();

    /**
     * Get the ZMP Delta.
     * @param footprint ponter to the footprint;
     * @param zmpInGlobalCoordinates zmp position expressed in global coordinates.
     * @return true / galse in case of success /failure.
     */
    bool getZMPGlobalPosition(const Step *footprint,
                              iDynTree::Vector2 &zmpInGlobalCoordinates) const;

    bool computeFeetWeight(const std::vector<StepPhase> &lFootPhases, const std::vector<size_t> &phaseShift,
                           const FootPrint &left, const FootPrint &right, const std::vector<iDynTree::Vector2>& zmpPosition);

    friend class DCMTrajectoryGenerator;

    /**
     * Constructor.
     */
    DCMTrajectoryGeneratorHelper();

 public:

    /**
     * Set the time constant of the 3D-LIPM.
     * @param omega is the time constant of the 3D-LIPM.
     * @return true / false in case of success / failure.
     */
    bool setOmega(const double &omega);

    /**
     * Set the period of the Trajectory generator planner.
     * @param dT is the period (in seconds) of the Trajectory generator planner
     * @return true / false in case of success / failure.
     */
    bool setdT(const double &dT);

    /**
     * Set the desired x and y displacements of the ZMP expressed on the foot frames.
     * @param leftZMPDelta vector containing the desired ZMP displacement from the center
     * of the left foot;
     * @param rightZMPDelta vector containing the desired ZMP displacement from the center
     * of the right foot.
     */
    void setZMPDelta(const iDynTree::Vector2 &leftZMPDelta,
                     const iDynTree::Vector2 &rightZMPDelta);


    /**
     * Set the pause condition.
     * @param maxDoubleSupportDuration is the maximum duration of a DS phase;
     * @param nominalDoubleSupportDuration is the nominal duration of a DS phase.
     * @return true if the pause conditions are set, false otherwise.
     */
    bool setPauseConditions(bool pauseActive, const double &maxDoubleSupportDuration, const double &nominalDoubleSupportDuration);

    /**
     * Generate the Divergent Component of Motion trajectory.
     * @param orderedSteps vector containing the both left and right footprint sorted into ascending impactTime order;
     * @param lFootPhases contains the phase of the left foot trajctory (those of the right can be obtained from this vector too).
     * @param left set of steps of the left foot
     * @param right set of steps of the right foot
     * @param initPosition is the position of the DCM at the beginning of the trajectory;
     * @param initVelocity is the velocity of the DCM at the beginning of the trajectory;
     * @param phaseShift vector containing the index when a change of phase (SS -> DS and viceversa) occours.
     * @return true / false in case of success / failure.
     */
    bool generateDCMTrajectory(const std::vector<const Step *> &orderedSteps, const std::vector<StepPhase> &lFootPhases,
                               const FootPrint &left, const FootPrint &right,
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

    /**
     * Get the position of the ZMP
     * @return a vector containing the DCM velocity during all the trajectory domain.
     */
    const std::vector<iDynTree::Vector2>& getZMPPosition() const;

    /**
     * Output the weight percentage carried by each foot while walking, according to the DCM trajectory.
     */
    void getWeightPercentage(std::vector<double>& weightInLeft, std::vector<double>& weightInRight) const;
};

#endif // DCMTRAJECTORYGENERATORHELPER_H
