/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <DCMTrajectoryGeneratorHelper.h>

// std
#include <math.h>
#include <vector>
#include <iostream>

// eigen
#include <Eigen/Dense>

// iDynTree
#include "iDynTree/Core/VectorFixSize.h"
#include "iDynTree/Core/EigenHelpers.h"

//--------General Support Trajectory definition

GeneralSupportTrajectory::GeneralSupportTrajectory(const double &startTime, const double &endTime)
{
    // set the general support trajectory domain
    m_trajectoryDomain = std::make_pair(startTime, endTime);
}

GeneralSupportTrajectory::~GeneralSupportTrajectory()
{ }

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

//--------Single Support Trajectory declaration

/**
 * SingleSupportTrajectory class represents the trajectory of the Divergent Component
 * of Motion during a single support phase.
 */
class SingleSupportTrajectory : public GeneralSupportTrajectory
{
 private:

    friend class DCMTrajectoryGeneratorHelper;

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

//--------Single Support Trajectory definition

SingleSupportTrajectory::SingleSupportTrajectory(const double &startTime,
                                                 const double &endTime,
                                                 const double &omega,
                                                 const iDynTree::Vector2 &ZMP,
                                                 const DCMTrajectoryPoint& boundaryCondition):
    GeneralSupportTrajectory(startTime, endTime),
    m_ZMP(ZMP),
    m_omega(omega)
{
    m_boundaryConditionTime = boundaryCondition.time;
    m_boundaryConditionDCMPosition = boundaryCondition.DCMPosition;
}

const iDynTree::Vector2& SingleSupportTrajectory::getZMP() const
{
    return m_ZMP;
}

bool SingleSupportTrajectory::getDCMPosition(const double &t, iDynTree::Vector2 &DCMPosition,
                                             const bool &checkDomainCondition)
{
    // Evaluate the position of the DCM at time t
    if (checkDomainCondition)
        if (!timeBelongsToDomain(t)){
            std::cerr << "[SINGLE SUPPORT TRAJECTORY] the time t: " << t
                      << " does not belong to the trajectory domain." << std::endl;
            return false;
        }

    iDynTree::toEigen(DCMPosition) = iDynTree::toEigen(m_ZMP) +
            exp(m_omega * (t - m_boundaryConditionTime)) *
            (iDynTree::toEigen(m_boundaryConditionDCMPosition) - iDynTree::toEigen(m_ZMP));
    return true;

}

bool SingleSupportTrajectory::getDCMVelocity(const double &t, iDynTree::Vector2 &DCMVelocity,
                                             const bool &checkDomainCondition)
{
    // Evaluate the velocity of the DCM at time t
    if (checkDomainCondition)
        if (!timeBelongsToDomain(t)){
            std::cerr << "[SINGLE SUPPORT TRAJECTORY] the time t: " << t
                      << " does not belong to the trajectory domain." << std::endl;
            return false;
        }

    iDynTree::toEigen(DCMVelocity) = m_omega * exp(m_omega * (t - m_boundaryConditionTime)) *
            (iDynTree::toEigen(m_boundaryConditionDCMPosition) - iDynTree::toEigen(m_ZMP));
    return true;
}

//--------Double Support Trajectory declaration

/**
 * DoubleSupportTrajectory class represents the trajectory of the Divergent Component
 * of Motion during a double support phase.
 */
class DoubleSupportTrajectory : public GeneralSupportTrajectory
{
 private:

    friend class DCMTrajectoryGeneratorHelper;

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

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor.
     * @param initBoundaryCondition desired init position and velocity of the
     * double support trajectory
     * @param finalBoundaryCondition desired final position and velocity of the
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

//--------Double Support Trajectory definition

DoubleSupportTrajectory::DoubleSupportTrajectory(const DCMTrajectoryPoint &initBoundaryCondition,
                                                 const DCMTrajectoryPoint &finalBoundaryCondition):
    GeneralSupportTrajectory(initBoundaryCondition.time, finalBoundaryCondition.time)
{

    double dsDuration = finalBoundaryCondition.time - initBoundaryCondition.time;

    // get the boundary conditions
    iDynTree::Vector2 positionBoundaryCondsX;
    iDynTree::Vector2 positionBoundaryCondsY;
    iDynTree::Vector2 velocityBoundaryCondsX;
    iDynTree::Vector2 velocityBoundaryCondsY;

    iDynTree::Vector2 initPosition = initBoundaryCondition.DCMPosition;
    iDynTree::Vector2 initVelocity = initBoundaryCondition.DCMVelocity;

    iDynTree::Vector2 endPosition = finalBoundaryCondition.DCMPosition;
    iDynTree::Vector2 endVelocity = finalBoundaryCondition.DCMVelocity;

    // set X boundary conditions
    positionBoundaryCondsX(0) = initPosition(0);
    positionBoundaryCondsX(1) = endPosition(0);
    velocityBoundaryCondsX(0) = initVelocity(0);
    velocityBoundaryCondsX(1) = endVelocity(0);

    // set Y boundary conditions
    positionBoundaryCondsY(0) = initPosition(1);
    positionBoundaryCondsY(1) = endPosition(1);
    velocityBoundaryCondsY(0) = initVelocity(1);
    velocityBoundaryCondsY(1) = endVelocity(1);

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

bool DoubleSupportTrajectory::getDCMPosition(const double &t, iDynTree::Vector2 &DCMPosition,
                                             const bool &checkDomainCondition)
{
    if(checkDomainCondition)
        if (!timeBelongsToDomain(t)){
            std::cerr << "[DOUBLE SUPPORT TRAJECTORY] the time t: " << t
                      << " does not belong to the trajectory domain." << std::endl;
            return false;
        }

    // Evaluate the position of the desired DCM at time t
    double startTime = std::get<0>(m_trajectoryDomain);
    double time = t - startTime;
    Eigen::Vector4d tVector;
    tVector<< pow(time,3),
            pow(time,2),
            time,
            1;

    // evaluate booth x and y coordinates
    DCMPosition(0) = tVector.dot(m_coefficentsX);
    DCMPosition(1) = tVector.dot(m_coefficentsY);

    return true;
}


bool DoubleSupportTrajectory::getDCMVelocity(const double &t, iDynTree::Vector2 &DCMVelocity,
                                             const bool &checkDomainCondition)
{
    if(checkDomainCondition)
        if (!timeBelongsToDomain(t)){
            std::cerr << "[DOUBLE SUPPORT TRAJECTORY] the time t: " << t
                      << "does not belong to the trajectory domain." << std::endl;
            return false;
        }

    // Evaluate the position of the desired DCM at time t
    double startTime = std::get<0>(m_trajectoryDomain);
    double time = t - startTime;
    Eigen::Vector4d tVector;
    tVector<< 3 * pow(time,2),
            2 * time,
            1,
            0;

    // evaluate booth x and y coordinates
    DCMVelocity(0) = tVector.dot(m_coefficentsX);
    DCMVelocity(1) = tVector.dot(m_coefficentsY);

    return true;
}


DCMTrajectoryGeneratorHelper::DCMTrajectoryGeneratorHelper():
    m_dT(0.01),
    m_omega(9.81/0.5),
    m_pauseActive(false)
{}

bool DCMTrajectoryGeneratorHelper::setOmega(const double &omega)
{
    if (omega < 0){
        std::cerr << "[DCMTrajectoryGeneratorHelper::setOmega] The time constant of the 3D-LIPM must be a positive number."
                  << std::endl;
        return false;
    }

    m_omega = omega;
    return true;
}

bool DCMTrajectoryGeneratorHelper::setdT(const double &dT)
{
    if (dT < 0){
        std::cerr << "[DCMTrajectoryGeneratorHelper::setdT] The period of the planner must be a positve number."
                  << std::endl;
        return false;
    }

    m_dT = dT;
    return true;
}

void DCMTrajectoryGeneratorHelper::setZMPDelta(const iDynTree::Vector2 &leftZMPDelta,
                                               const iDynTree::Vector2 &rightZMPDelta)
{
    m_leftZMPDelta = leftZMPDelta;
    m_rightZMPDelta = rightZMPDelta;
}

bool DCMTrajectoryGeneratorHelper::getZMPDelta(const Step* footprint,
                                               iDynTree::Vector2 &ZMPDelta) const
{
    if(footprint->footName == "left"){
        ZMPDelta = m_leftZMPDelta;
        return true;
    }
    else if(footprint->footName == "right"){
        ZMPDelta = m_rightZMPDelta;
        return true;
    }
    else{
        std::cerr << "[DCMTrajectoryGeneratorHelper::getZMPDelta] The name of the footprint is neither left nor right. Does iCub have more legs?!"
                  << std::endl;
        return false;
    }
}

bool DCMTrajectoryGeneratorHelper::addLastStep(const double &singleSupportStartTime,
                                               const double &singleSupportEndTime,
                                               const double &doubleSupportEndTime,
                                               const iDynTree::Vector2 &ZMP,
                                               const DCMTrajectoryPoint &singleSupportBoundaryCondition)
{
    // evaluate the Single Support trajectory parameters
    std::shared_ptr<GeneralSupportTrajectory> newSingleSupport = nullptr;
    newSingleSupport = std::make_shared<SingleSupportTrajectory>(singleSupportStartTime, singleSupportEndTime,
                                                                 m_omega, ZMP, singleSupportBoundaryCondition);

    // instantiate position and velocity boundary conditions vectors
    DCMTrajectoryPoint doubleSupportInitBoundaryCondition;
    DCMTrajectoryPoint doubleSupportFinalBoundaryCondition;

    doubleSupportInitBoundaryCondition.time = singleSupportEndTime;
    doubleSupportFinalBoundaryCondition.time = doubleSupportEndTime;

    // set boundary conditions
    if(!newSingleSupport->getDCMPosition(doubleSupportInitBoundaryCondition.time,
                                         doubleSupportInitBoundaryCondition.DCMPosition)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::addLastStep] Error when the position of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!newSingleSupport->getDCMVelocity(doubleSupportInitBoundaryCondition.time,
                                         doubleSupportInitBoundaryCondition.DCMVelocity)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::addLastStep] Error when the velocity of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    // only for the last step the final position of the DCM coincides with le initial position
    doubleSupportFinalBoundaryCondition.DCMPosition = doubleSupportInitBoundaryCondition.DCMPosition;
    doubleSupportFinalBoundaryCondition.DCMVelocity.zero();

    std::shared_ptr<GeneralSupportTrajectory> newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                                                           doubleSupportFinalBoundaryCondition);
    // add the new Double Support phase
    m_trajectory.push_back(newDoubleSupport);

    // add the new Single Support phase
    m_trajectory.push_back(newSingleSupport);

    return true;
}

bool DCMTrajectoryGeneratorHelper::addNewStep(const double &singleSupportStartTime,
                                              const double &singleSupportEndTime,
                                              const iDynTree::Vector2 &ZMP,
                                              const DCMTrajectoryPoint &singleSupportBoundaryCondition)
{
    // evaluate the new Single Support trajectory parameters
    std::shared_ptr<GeneralSupportTrajectory> newSingleSupportTrajectory = nullptr;
    newSingleSupportTrajectory = std::make_shared<SingleSupportTrajectory>(singleSupportStartTime, singleSupportEndTime,
                                                                           m_omega, ZMP, singleSupportBoundaryCondition);

    // instantiate boundary conditions structs
    DCMTrajectoryPoint doubleSupportInitBoundaryCondition;
    DCMTrajectoryPoint doubleSupportFinalBoundaryCondition;

    // the end time of the Double Support phase coinceds with the beginning of the next Single Support phase
    std::shared_ptr<GeneralSupportTrajectory> nextSingleSupport = m_trajectory.back();

    // get the domain of the next SingleSupport trajectory
    const std::pair<double, double> nextSubTrajectoryDomain = nextSingleSupport->getTrajectoryDomain();
    doubleSupportFinalBoundaryCondition.time = std::get<0>(nextSubTrajectoryDomain);

    doubleSupportInitBoundaryCondition.time = singleSupportEndTime;

    // set the boundary conditions
    if(!newSingleSupportTrajectory->getDCMPosition(doubleSupportInitBoundaryCondition.time,
                                                   doubleSupportInitBoundaryCondition.DCMPosition)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::addNewStep] Error when the position of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!newSingleSupportTrajectory->getDCMVelocity(doubleSupportInitBoundaryCondition.time,
                                                   doubleSupportInitBoundaryCondition.DCMVelocity)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::addNewStep] Error when the velocity of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!nextSingleSupport->getDCMPosition(doubleSupportFinalBoundaryCondition.time,
                                          doubleSupportFinalBoundaryCondition.DCMPosition)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::addNewStep] Error when the position of the DCM in the previous SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!nextSingleSupport->getDCMVelocity(doubleSupportFinalBoundaryCondition.time,
                                          doubleSupportFinalBoundaryCondition.DCMVelocity)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::addNewStep] Error when the velocity of the DCM in the previous SS phase is evaluated." <<std::endl;
        return false;
    }

    // evaluate the new Double Support phase
    std::shared_ptr<GeneralSupportTrajectory> newDoubleSupport = nullptr;

    // check if pause features is active and if it is needed
    if (m_pauseActive &&
            (doubleSupportFinalBoundaryCondition.time - doubleSupportInitBoundaryCondition.time > m_maxDoubleSupportDuration)){
        // in this case the DS phase is splitted in three DS phase
        // In the first one the DCM of the robot has to reach the center of the feet convex hull
        // In the second one the robot has to stop (stance phase)
        // In the third one the DCM has to reach the "endPosition" with the "endVelocity"

        // evaluate the stance position boundary constraints
        DCMTrajectoryPoint doubleSupportStanceInitBoundaryCondition;
        DCMTrajectoryPoint doubleSupportStanceFinalBoundaryCondition;

        std::shared_ptr<SingleSupportTrajectory> nextSingleSupport_cast = std::static_pointer_cast<SingleSupportTrajectory>(nextSingleSupport);
        iDynTree::toEigen(doubleSupportStanceInitBoundaryCondition.DCMPosition) = (iDynTree::toEigen(ZMP) + iDynTree::toEigen(nextSingleSupport_cast->getZMP())) / 2;
        doubleSupportStanceInitBoundaryCondition.DCMVelocity.zero();
        // the constraints at the beginning and at the end of the double support stance phases are equal except for the times
        doubleSupportStanceFinalBoundaryCondition = doubleSupportStanceInitBoundaryCondition;

        doubleSupportStanceInitBoundaryCondition.time = doubleSupportInitBoundaryCondition.time + m_nominalDoubleSupportDuration / 2;
        doubleSupportStanceFinalBoundaryCondition.time = doubleSupportFinalBoundaryCondition.time - m_nominalDoubleSupportDuration / 2;

        // add the 3-th part of the Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportStanceFinalBoundaryCondition,
                                                                     doubleSupportFinalBoundaryCondition);
        m_trajectory.push_back(newDoubleSupport);

        // add 2-th part of the Double Support phase (stance phase)
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportStanceInitBoundaryCondition,
                                                                     doubleSupportStanceFinalBoundaryCondition);
        m_trajectory.push_back(newDoubleSupport);

        // add 1-th part of the Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                     doubleSupportStanceInitBoundaryCondition);
        m_trajectory.push_back(newDoubleSupport);
    }else{
        // add the new Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                     doubleSupportFinalBoundaryCondition);
        m_trajectory.push_back(newDoubleSupport);
    }
    // add the new Single Support phase
    m_trajectory.push_back(newSingleSupportTrajectory);

    return true;
}

bool DCMTrajectoryGeneratorHelper::addFirstDoubleSupportPhase(const DCMTrajectoryPoint &doubleSupportInitBoundaryCondition,
                                                              const Step* firstSwingFoot)
{
    // get the next Single Support trajectory
    std::shared_ptr<SingleSupportTrajectory> nextSingleSupport = std::static_pointer_cast<SingleSupportTrajectory>(m_trajectory.back());

    // get the domain of the next SingleSupport trajectory
    const std::pair<double, double> nextSubTrajectoryDomain = nextSingleSupport->getTrajectoryDomain();

    // instantiate the final boundary condition struct
    DCMTrajectoryPoint doubleSupportFinalBoundaryCondition;

    // the begining time of the Double Support phase coinceds with the beginning of the next Single Support phase
    doubleSupportFinalBoundaryCondition.time = std::get<0>(nextSubTrajectoryDomain);

    // set the boundary conditions
    if(!nextSingleSupport->getDCMPosition(doubleSupportFinalBoundaryCondition.time,
                                          doubleSupportFinalBoundaryCondition.DCMPosition)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::addFirstDoubleSupportPhase] Error when the position of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!nextSingleSupport->getDCMVelocity(doubleSupportFinalBoundaryCondition.time,
                                          doubleSupportFinalBoundaryCondition.DCMVelocity)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::addFirstDoubleSupportPhase] Error when the velocity of the DCM in the next SS phase is evaluated."
                  << std::endl;
        return false;
    }

    // evaluate the new Double Support phase
    std::shared_ptr<GeneralSupportTrajectory> newDoubleSupport = nullptr;

    // check if pause features is active and if it is needed
    if (m_pauseActive &&
            (doubleSupportFinalBoundaryCondition.time - doubleSupportInitBoundaryCondition.time > m_maxDoubleSupportDuration)){
        // in this case the DS phase is splitted in three DS phase
        // In the first one the DCM of the robot has to reach the center of the feet convex hull
        // In the second one the robot has to stop (stance phase)
        // In the third one the DCM has to reach the "endPosition" with the "endVelocity"

        // evaluate the stance position boundary constraints
        DCMTrajectoryPoint doubleSupportStanceInitBoundaryCondition;
        DCMTrajectoryPoint doubleSupportStanceFinalBoundaryCondition;

        iDynTree::Vector2 positionOfTheFirstSwingFoot;
        double yawAngle = firstSwingFoot->angle;

        iDynTree::Vector2 ZMPDelta;
        if(!getZMPDelta(firstSwingFoot, ZMPDelta)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::addFirstDoubleSupportPhase] Unable to get the ZMP Delta."
                      << std::endl;
            return false;
        }

        positionOfTheFirstSwingFoot(0) = firstSwingFoot->position(0) + cos(yawAngle) * ZMPDelta(0) - sin(yawAngle) * ZMPDelta(1);
        positionOfTheFirstSwingFoot(1) = firstSwingFoot->position(1) + sin(yawAngle) * ZMPDelta(0) + cos(yawAngle) * ZMPDelta(1);

        iDynTree::toEigen(doubleSupportStanceInitBoundaryCondition.DCMPosition) = (iDynTree::toEigen(positionOfTheFirstSwingFoot) + iDynTree::toEigen(nextSingleSupport->getZMP())) / 2;
        doubleSupportStanceInitBoundaryCondition.DCMVelocity.zero();
        // the constraints at the beginning and at the end of the double support stance phases are equal except for the times
        doubleSupportStanceFinalBoundaryCondition = doubleSupportStanceInitBoundaryCondition;

        doubleSupportStanceInitBoundaryCondition.time = doubleSupportInitBoundaryCondition.time + m_nominalDoubleSupportDuration / 2;
        doubleSupportStanceFinalBoundaryCondition.time = doubleSupportFinalBoundaryCondition.time - m_nominalDoubleSupportDuration / 2;

        // add the 3-th part of the Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportStanceFinalBoundaryCondition,
                                                                     doubleSupportFinalBoundaryCondition);
        m_trajectory.push_back(newDoubleSupport);

        // add 2-th part of the Double Support phase (stance phase)
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportStanceInitBoundaryCondition,
                                                                     doubleSupportStanceFinalBoundaryCondition);
        m_trajectory.push_back(newDoubleSupport);

        // add 1-th part of the Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                     doubleSupportStanceInitBoundaryCondition);
        m_trajectory.push_back(newDoubleSupport);
    }else{
        // add the new Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                     doubleSupportFinalBoundaryCondition);
        // add the new Double Support phase
        m_trajectory.push_back(newDoubleSupport);
    }

    return true;

}

void DCMTrajectoryGeneratorHelper::getLastStepsTiming(double &singleSupportStartTime,
                                                      double &singleSupportEndTime,
                                                      double &doubleSupportEndTime,
                                                      double &singleSupportBoundaryConditionTime)
{
    doubleSupportEndTime = m_phaseShift.back() * m_dT;
    m_phaseShift.pop_back();

    singleSupportEndTime = m_phaseShift.back() * m_dT;
    m_phaseShift.pop_back();

    singleSupportStartTime = m_phaseShift.back() * m_dT;
    m_phaseShift.pop_back();

    // note: the boundary condition of the single support trajectory
    //       coincides with the end time of the single support trajectory
    //       (valid only for the last step)
    singleSupportBoundaryConditionTime = singleSupportEndTime;
}


void DCMTrajectoryGeneratorHelper::getStepsTiming(double &singleSupportStartTime,
                                                  double &singleSupportEndTime,
                                                  double &singleSupportBoundaryConditionTime)
{
    singleSupportEndTime = m_phaseShift.back() * m_dT;
    m_phaseShift.pop_back();

    singleSupportStartTime = m_phaseShift.back() * m_dT;
    m_phaseShift.pop_back();

    std::shared_ptr<GeneralSupportTrajectory> nextSingleSupportTrajectory = m_trajectory.back();
    std::pair<double, double> nextSingleSupportTrajectoryDomain = nextSingleSupportTrajectory->getTrajectoryDomain();
    double nextsingleSupportStartTime = std::get<0>(nextSingleSupportTrajectoryDomain);
    singleSupportBoundaryConditionTime = (nextsingleSupportStartTime + singleSupportEndTime) / 2;
}

void DCMTrajectoryGeneratorHelper::getFirstDoubleSupportTiming(double &doubleSupportStartTime)
{
    doubleSupportStartTime = m_phaseShift.back() * m_dT;
    m_phaseShift.pop_back();
}

bool DCMTrajectoryGeneratorHelper::generateFixStanceDCMTrajectory(const iDynTree::Vector2 &initPosition,
                                                                  const iDynTree::Vector2 &initVelocity,
                                                                  const iDynTree::Vector2 &finalPosition,
                                                                  const std::vector<size_t> &phaseShift)
{
    m_trajectoryDomain = std::make_pair(phaseShift.front(), phaseShift.back());

    // reset the trajectory
    m_trajectory.clear();

    // instantiate position and velocity boundary conditions vectors
    DCMTrajectoryPoint doubleSupportInitBoundaryCondition;
    DCMTrajectoryPoint doubleSupportFinalBoundaryCondition;

    // set the initial boundary conditions
    doubleSupportInitBoundaryCondition.time = phaseShift.front() * m_dT;
    doubleSupportInitBoundaryCondition.DCMPosition = initPosition;
    doubleSupportInitBoundaryCondition.DCMVelocity = initVelocity;

    // set the final boundary conditions
    doubleSupportFinalBoundaryCondition.time = phaseShift.back() * m_dT;
    doubleSupportFinalBoundaryCondition.DCMPosition = finalPosition;
    doubleSupportFinalBoundaryCondition.DCMVelocity.zero();

    std::shared_ptr<GeneralSupportTrajectory> newDoubleSupport = nullptr;
    newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                 doubleSupportFinalBoundaryCondition);
    // add the new Double Support phase
    m_trajectory.push_back(newDoubleSupport);

    // evaluate the DCM trajectory
    if(!evaluateDCMTrajectory()){
        std::cerr << "[DCMTrajectoryGeneratorHelper::generateFixStanceDCMTrajectory] Error when the whole DCM trajectory is evaluated." << std::endl;
        return false;
    }
    return true;
}

bool DCMTrajectoryGeneratorHelper::generateDCMTrajectory(const std::vector<const Step*>& orderedSteps,
                                                         const Step firstStanceFoot,
                                                         const Step firstSwingFoot,
                                                         const iDynTree::Vector2 &initPosition,
                                                         const iDynTree::Vector2 &initVelocity,
                                                         const std::vector<size_t> &phaseShift)
{
    m_trajectoryDomain = std::make_pair(phaseShift.front(), phaseShift.back());
    m_phaseShift = phaseShift;
    m_orderedSteps = orderedSteps;

    // reset the trajectory
    m_trajectory.clear();

    m_firstStanceFoot = firstStanceFoot;
    m_firstSwingFoot = firstSwingFoot;

    // add the first stance foot at the beginning of the orderedStep vector
    m_orderedSteps.insert(m_orderedSteps.begin(), &(m_firstStanceFoot));

    double singleSupportStartTime;
    double singleSupportEndTime;
    double doubleSupportEndTime;
    double singleSupportBoundaryConditionTime;
    DCMTrajectoryPoint singleSupportBoundaryCondition;
    iDynTree::Vector2 lastZMP;
    iDynTree::Vector2 comPosition;
    iDynTree::Vector2 ZMPDelta;
    double yawAngle;

    // evaluate times for the last step
    getLastStepsTiming(singleSupportStartTime,
                       singleSupportEndTime,
                       doubleSupportEndTime,
                       singleSupportBoundaryConditionTime);

    // the ZMP is shifted before evaluate the DCM
    if(!getZMPDelta(m_orderedSteps.back(), ZMPDelta)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Unable to get the ZMP Delta."
                  << std::endl;
        return false;
    }

    yawAngle = m_orderedSteps.back()->angle;
    lastZMP(0) = m_orderedSteps.back()->position(0) + cos(yawAngle) * ZMPDelta(0) - sin(yawAngle) * ZMPDelta(1);
    lastZMP(1) = m_orderedSteps.back()->position(1) + sin(yawAngle) * ZMPDelta(0) + cos(yawAngle) * ZMPDelta(1);
    m_orderedSteps.pop_back();

    // evaluate the position of the Center of mass at the end of the trajectory
    iDynTree::Vector2 temp;
    if(!getZMPDelta(m_orderedSteps.back(), ZMPDelta)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Unable to get the ZMP Delta."
                  << std::endl;
        return false;
    }
    yawAngle = m_orderedSteps.back()->angle;
    temp(0) = m_orderedSteps.back()->position(0) + cos(yawAngle) * ZMPDelta(0) - sin(yawAngle) * ZMPDelta(1);
    temp(1) = m_orderedSteps.back()->position(1) + sin(yawAngle) * ZMPDelta(0) + cos(yawAngle) * ZMPDelta(1);
    iDynTree::toEigen(comPosition)  = (iDynTree::toEigen(lastZMP) + iDynTree::toEigen(temp)) / 2;

    singleSupportBoundaryCondition.time = singleSupportBoundaryConditionTime;
    singleSupportBoundaryCondition.DCMPosition = comPosition;
    singleSupportBoundaryCondition.DCMVelocity.zero();

    // the ZMP is shifted before evaluate the DCM
    if(!getZMPDelta(m_orderedSteps.back(), ZMPDelta)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Unable to get the ZMP Delta."
                  << std::endl;
        return false;
    }
    yawAngle = m_orderedSteps.back()->angle;
    lastZMP(0) = m_orderedSteps.back()->position(0) + cos(yawAngle) * ZMPDelta(0) - sin(yawAngle) * ZMPDelta(1);
    lastZMP(1) = m_orderedSteps.back()->position(1) + sin(yawAngle) * ZMPDelta(0) + cos(yawAngle) * ZMPDelta(1);
    m_orderedSteps.pop_back();

    // evaluate the last step
    if(!addLastStep(singleSupportStartTime, singleSupportEndTime,
                    doubleSupportEndTime, lastZMP, singleSupportBoundaryCondition)){
        std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Error when the DCM trajectory of the last step is generated." << std::endl;
        return false;
    }

    while (m_orderedSteps.size() > 0){
        getStepsTiming(singleSupportStartTime, singleSupportEndTime,
                       singleSupportBoundaryConditionTime);

        // the ZMP is shifted before evaluate the DCM
        if(!getZMPDelta(m_orderedSteps.back(), ZMPDelta)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Unable to get the ZMP Delta."
                      << std::endl;
            return false;
        }
        yawAngle = m_orderedSteps.back()->angle;
        lastZMP(0) = m_orderedSteps.back()->position(0) + cos(yawAngle) * ZMPDelta(0) - sin(yawAngle) * ZMPDelta(1);
        lastZMP(1) = m_orderedSteps.back()->position(1) + sin(yawAngle) * ZMPDelta(0) + cos(yawAngle) * ZMPDelta(1);
        m_orderedSteps.pop_back();

        // get the next Single Support trajectory
        std::shared_ptr<GeneralSupportTrajectory> nextSingleSupport = m_trajectory.back();

        // evaluate the single support boundary condition
        singleSupportBoundaryCondition.time = singleSupportBoundaryConditionTime;
        // NOTE: the DCM at the boundary condition time is outside the SS subtrajectory
        nextSingleSupport->getDCMPosition(singleSupportBoundaryConditionTime, singleSupportBoundaryCondition.DCMPosition, false);
        // the DCM velocity is not taken into account in the new SS trajectory generation
        singleSupportBoundaryCondition.DCMVelocity.zero();

        if(!addNewStep(singleSupportStartTime, singleSupportEndTime,
                       lastZMP, singleSupportBoundaryCondition)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Error when the DCM trajectory of a new step is generated." << std::endl;
            return false;
        }
    }

    // evaluate the first Double support phase
    DCMTrajectoryPoint firstDoubleSupportBoundaryCondition;
    firstDoubleSupportBoundaryCondition.DCMPosition = initPosition;
    firstDoubleSupportBoundaryCondition.DCMVelocity = initVelocity;
    getFirstDoubleSupportTiming(firstDoubleSupportBoundaryCondition.time);
    if(!addFirstDoubleSupportPhase(firstDoubleSupportBoundaryCondition, &(m_firstSwingFoot))){
        std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Error when the DCM trajectory of the first DS phase is generated." << std::endl;
        return false;
    }

    // evaluate the DCM trajectory
    if(!evaluateDCMTrajectory()){
        std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Error when the whole DCM trajectory is evaluated." << std::endl;
        return false;
    }

    return true;
}

iDynTree::Vector2 DCMTrajectoryGeneratorHelper::evaluateZMPPosition(const iDynTree::Vector2 &DCMPosition,
                                                                    const iDynTree::Vector2 &DCMVelocity)
{
    iDynTree::Vector2 ZMP;
    iDynTree::toEigen(ZMP) = iDynTree::toEigen(DCMPosition) - iDynTree::toEigen(DCMVelocity) / m_omega;
    return ZMP;
}

bool DCMTrajectoryGeneratorHelper::evaluateDCMTrajectory()
{
    size_t timeVectorLength = std::get<1>(m_trajectoryDomain) - std::get<0>(m_trajectoryDomain);

    // clear all the previous DCM position
    m_DCMPosition.clear();
    m_DCMPosition.reserve(timeVectorLength);

    // clear all the previous DCM velocity
    m_DCMVelocity.clear();
    m_DCMVelocity.reserve(timeVectorLength);

    m_ZMPPosition.clear();
    m_ZMPPosition.reserve(timeVectorLength);

    iDynTree::Vector2 DCMPosition, DCMVelocity;
    double time;
    std::vector<std::shared_ptr<GeneralSupportTrajectory>>::reverse_iterator subTrajectory = m_trajectory.rbegin();

    for (size_t t = 0; t < timeVectorLength; t++){
        time = (t + std::get<0>(m_trajectoryDomain)) * m_dT;
        double subTrajectoryEndTime  = std::get<1>((*subTrajectory)->getTrajectoryDomain());
        if (time > subTrajectoryEndTime){
            subTrajectory++;
        }

        if(!(*subTrajectory)->getDCMPosition(time, DCMPosition)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::evaluateDCMTrajectory] Error when the position of the DCM is evaluated." << std::endl;
            return false;
        }

        if(!(*subTrajectory)->getDCMVelocity(time, DCMVelocity)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::evaluateDCMTrajectory] Error when the velocity of the DCM is evaluated." << std::endl;
            return false;
        }
        m_DCMPosition.push_back(DCMPosition);
        m_DCMVelocity.push_back(DCMVelocity);
        m_ZMPPosition.push_back(evaluateZMPPosition(DCMPosition, DCMVelocity));
    }
    return true;
}

const std::vector<iDynTree::Vector2>& DCMTrajectoryGeneratorHelper::getDCMPosition() const
{
    return m_DCMPosition;
}

const std::vector<iDynTree::Vector2>& DCMTrajectoryGeneratorHelper::getDCMVelocity() const
{
    return m_DCMVelocity;
}

const std::vector<iDynTree::Vector2>& DCMTrajectoryGeneratorHelper::getZMPPosition() const
{
    return m_ZMPPosition;
}

bool DCMTrajectoryGeneratorHelper::setPauseConditions(bool pauseActive, const double &maxDoubleSupportDuration, const double &nominalDoubleSupportDuration)
{
    if (maxDoubleSupportDuration < 0){
        std::cerr << "[DCMTrajectoryGeneratorHelper::setPauseConditions] If the maxDoubleSupportDuration is negative, the robot won't pause in middle stance." << std::endl;
        m_pauseActive = false;
        return false;
    }

    m_pauseActive = pauseActive;
    m_maxDoubleSupportDuration = maxDoubleSupportDuration;

    if (m_pauseActive){
        if (nominalDoubleSupportDuration <= 0){
            std::cerr << "[DCMTrajectoryGeneratorHelper::setPauseConditions] The nominalDoubleSupportDuration is supposed to be positive." << std::endl;
            m_pauseActive = false;
            return false;
        }

        if (nominalDoubleSupportDuration > maxDoubleSupportDuration){
            std::cerr << "[DCMTrajectoryGeneratorHelper::setPauseConditions] The nominalDoubleSupportDuration cannot be greater than maxDoubleSupportDuration." << std::endl;
            m_pauseActive = false;
            return false;
        }
    }
    m_nominalDoubleSupportDuration = nominalDoubleSupportDuration;

    return true;
}
