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
                                                 const double &omega,
                                                 const iDynTree::Vector2 &ZMP,
                                                 const DCMTrajectoryPoint& boundaryCondition):
    GeneralSupportTrajectory(startTime, endTime),
    m_omega(omega),
    m_ZMP(ZMP)
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


DCMTrajectoryGenerator::DCMTrajectoryGenerator():
    m_pauseActive(false)
{}

DCMTrajectoryGenerator::DCMTrajectoryGenerator(const double &dT, const double &omega):
    m_omega(omega),
    m_dT(dT),
    m_pauseActive(false)
{}

void DCMTrajectoryGenerator::setOmega(const double &omega)
{
    m_omega = omega;
}

void DCMTrajectoryGenerator::setdT(const double &dT)
{
    m_dT = dT;
}

bool DCMTrajectoryGenerator::addLastStep(const double &singleSupportStartTime,
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
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!newSingleSupport->getDCMVelocity(doubleSupportInitBoundaryCondition.time,
                                         doubleSupportInitBoundaryCondition.DCMVelocity)){
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    // only for the last step the final position of the DCM coincides with le initial position
    doubleSupportFinalBoundaryCondition.DCMPosition = doubleSupportInitBoundaryCondition.DCMPosition;
    doubleSupportFinalBoundaryCondition.DCMVelocity.zero();

    double doubleSupportStartTime = singleSupportEndTime;
    std::shared_ptr<GeneralSupportTrajectory> newDoubleSupport = nullptr;
    newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                 doubleSupportFinalBoundaryCondition);
    // add the new Double Support phase
    m_trajectory.push_back(newDoubleSupport);

    // add the new Single Support phase
    m_trajectory.push_back(newSingleSupport);

    return true;
}

bool DCMTrajectoryGenerator::addNewStep(const double &singleSupportStartTime,
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
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!newSingleSupportTrajectory->getDCMVelocity(doubleSupportInitBoundaryCondition.time,
                                                   doubleSupportInitBoundaryCondition.DCMVelocity)){
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!nextSingleSupport->getDCMPosition(doubleSupportFinalBoundaryCondition.time,
                                          doubleSupportFinalBoundaryCondition.DCMPosition)){
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM in the previous SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!nextSingleSupport->getDCMVelocity(doubleSupportFinalBoundaryCondition.time,
                                          doubleSupportFinalBoundaryCondition.DCMVelocity)){
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM in the previous SS phase is evaluated." <<std::endl;
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
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportFinalBoundaryCondition,
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

bool DCMTrajectoryGenerator::addFirstDoubleSupportPhase(const DCMTrajectoryPoint &doubleSupportInitBoundaryCondition)
{
    // get the next Single Support trajectory
    std::shared_ptr<GeneralSupportTrajectory> nextSingleSupport = m_trajectory.back();

    // get the domain of the next SingleSupport trajectory
    const std::pair<double, double> nextSubTrajectoryDomain = nextSingleSupport->getTrajectoryDomain();

    // instantiate the final boundary condition struct
    DCMTrajectoryPoint doubleSupportFinalBoundaryCondition;

    // the begining time of the Double Support phase coinceds with the beginning of the next Single Support phase
    doubleSupportFinalBoundaryCondition.time = std::get<0>(nextSubTrajectoryDomain);

    // set the boundary conditions
    if(!nextSingleSupport->getDCMPosition(doubleSupportFinalBoundaryCondition.time,
                                          doubleSupportFinalBoundaryCondition.DCMPosition)){
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    if(!nextSingleSupport->getDCMVelocity(doubleSupportFinalBoundaryCondition.time,
                                          doubleSupportFinalBoundaryCondition.DCMVelocity)){
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM in the next SS phase is evaluated." <<std::endl;
        return false;
    }

    // evaluate the new Double Support phase
    std::shared_ptr<GeneralSupportTrajectory> newDoubleSupport = nullptr;
    newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                 doubleSupportFinalBoundaryCondition);
    // add the new Double Support phase
    m_trajectory.push_back(newDoubleSupport);
    return true;
}

void DCMTrajectoryGenerator::getLastStepsTiming(double &singleSupportStartTime,
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


void DCMTrajectoryGenerator::getStepsTiming(double &singleSupportStartTime,
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

void DCMTrajectoryGenerator::getFirstDoubleSupportTiming(double &doubleSupportStartTime)
{
    doubleSupportStartTime = m_phaseShift.back() * m_dT;
    m_phaseShift.pop_back();
}

bool DCMTrajectoryGenerator::generateDCMTrajectory(const std::vector<StepList::const_iterator> &orderedSteps,
                                                   const StepList::const_iterator &firstStanceFoot,
                                                   const iDynTree::Vector2 &initPosition,
                                                   const iDynTree::Vector2 &initVelocity,
                                                   const std::vector<size_t> &phaseShift)
{
    m_trajectoryDomain = std::make_pair(phaseShift.front(), phaseShift.back());
    m_phaseShift = phaseShift;
    m_orderedSteps = orderedSteps;

    // reset the trajectory
    m_trajectory.clear();

    // add the first stance foot at the beginning of the orderedStep vector
    m_orderedSteps.insert(m_orderedSteps.begin(), firstStanceFoot);

    double singleSupportStartTime;
    double singleSupportEndTime;
    double doubleSupportEndTime;
    double singleSupportBoundaryConditionTime;
    DCMTrajectoryPoint singleSupportBoundaryCondition;

    // evaluate times for the last step
    getLastStepsTiming(singleSupportStartTime,
                       singleSupportEndTime,
                       doubleSupportEndTime,
                       singleSupportBoundaryConditionTime);

    iDynTree::Vector2 lastZMP = m_orderedSteps.back()->position;
    m_orderedSteps.pop_back();

    // evaluate the position of the Center of mass at the end of the trajectory
    iDynTree::Vector2 comPosition;
    iDynTree::toEigen(comPosition)  = (iDynTree::toEigen(lastZMP) + iDynTree::toEigen(m_orderedSteps.back()->position)) / 2;

    singleSupportBoundaryCondition.time = singleSupportBoundaryConditionTime;
    singleSupportBoundaryCondition.DCMPosition = comPosition;
    singleSupportBoundaryCondition.DCMVelocity.zero();

    lastZMP = m_orderedSteps.back()->position;
    m_orderedSteps.pop_back();

    // evaluate the last step
    if(!addLastStep(singleSupportStartTime, singleSupportEndTime,
                    doubleSupportEndTime, lastZMP, singleSupportBoundaryCondition)){
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the DCM trajectory of the last step is generated." << std::endl;
        return false;
    }

    while (m_orderedSteps.size() > 0){
        getStepsTiming(singleSupportStartTime, singleSupportEndTime,
                       singleSupportBoundaryConditionTime);

        lastZMP = m_orderedSteps.back()->position;
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
            std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the DCM trajectory of a new step is generated." << std::endl;
            return false;
        }
    }

    // evaluate the first Double support phase
    DCMTrajectoryPoint firstDoubleSupportBoundaryCondition;
    firstDoubleSupportBoundaryCondition.DCMPosition = initPosition;
    firstDoubleSupportBoundaryCondition.DCMVelocity = initVelocity;
    getFirstDoubleSupportTiming(firstDoubleSupportBoundaryCondition.time);
    if(!addFirstDoubleSupportPhase(firstDoubleSupportBoundaryCondition)){
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the DCM trajectory of the first DS phase is generated." << std::endl;
        return false;
    }

    // evaluate the DCM trajectory
    if(!evaluateDCMTrajectory()){
        std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the whole DCM trajectory is evaluated." << std::endl;
        return false;
    }

    return true;
}

bool DCMTrajectoryGenerator::evaluateDCMTrajectory()
{
    size_t timeVectorLength = std::get<1>(m_trajectoryDomain) - std::get<0>(m_trajectoryDomain);
    std::vector<size_t> timeVector(timeVectorLength);

    // populate the time vector
    std::iota(std::begin(timeVector), std::end(timeVector), std::get<0>(m_trajectoryDomain));

    // clear all the previous DCM position
    m_DCMPosition.clear();
    m_DCMPosition.reserve(timeVectorLength);

    // clear all the previous DCM velocity
    m_DCMVelocity.clear();
    m_DCMVelocity.reserve(timeVectorLength);

    iDynTree::Vector2 DCMPosition, DCMVelocity;
    double time;
    std::vector<std::shared_ptr<GeneralSupportTrajectory>>::reverse_iterator subTrajectory = m_trajectory.rbegin();

    for (size_t t : timeVector){
        time = t * m_dT;
        double subTrajectoryEndTime  = std::get<1>((*subTrajectory)->getTrajectoryDomain());
        if (time > subTrajectoryEndTime){
            subTrajectory++;
        }

        if(!(*subTrajectory)->getDCMPosition(time, DCMPosition)){
            std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the position of the DCM is evaluated." << std::endl;
            return false;
        }

        if(!(*subTrajectory)->getDCMVelocity(time, DCMVelocity)){
            std::cerr << "[DCM TRAJECTORY GENERATOR] Error when the velocity of the DCM is evaluated." << std::endl;
            return false;
        }
        m_DCMPosition.push_back(DCMPosition);
        m_DCMVelocity.push_back(DCMVelocity);
    }
    return true;
}

const std::vector<iDynTree::Vector2>& DCMTrajectoryGenerator::getDCMPosition() const
{
    return m_DCMPosition;
}

const std::vector<iDynTree::Vector2>& DCMTrajectoryGenerator::getDCMVelocity() const
{
    return m_DCMVelocity;
}

bool DCMTrajectoryGenerator::setPauseConditions(const double &maxDoubleSupportDuration, const double &nominalDoubleSupportDuration)
{
    if (maxDoubleSupportDuration < 0){
        std::cerr << "[DCM TRAJECTORY GENERATOR] If the maxDoubleSupportDuration is negative, the robot won't pause in middle stance." << std::endl;
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

        if (nominalDoubleSupportDuration > maxDoubleSupportDuration){
            std::cerr << "[DCM TRAJECTORY GENERATOR] The nominalDoubleSupportDuration cannot be greater than maxDoubleSupportDuration." << std::endl;
            m_pauseActive = false;
            return false;
        }
    }
    m_nominalDoubleSupportDuration = nominalDoubleSupportDuration;

    return true;
}
