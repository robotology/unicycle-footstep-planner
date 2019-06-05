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

GeneralSupportTrajectory::GeneralSupportTrajectory(const double &startTime, const double &endTime,
                                                   const double &omega)
{
    // set the general support trajectory domain
    assert(startTime <= endTime);
    assert(omega > 0);

    m_trajectoryDomain = std::make_pair(startTime, endTime);
    m_omega = omega;
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

//    std::cerr << "[GENERAL SUPPORT TRAJECTORY] the time t: " << t << " does not belong to the trajectory domain" << std::endl;
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

    /**
     * Implementation of the getZMPPosition method of the
     * GeneralSupportTrajectory class.
     * @param t is the trajectory evaluation time;
     * @param ZMPPosition cartesian position of the Zero Moment Point;
     * @param checkDomainCondition flag used to check if the time belongs to the trajectory domain (default value true).
     * @return true / false in case of success / failure.
     */
    virtual bool getZMPPosition(const double &t, iDynTree::Vector2& ZMPVelocity, const bool &checkDomainCondition = true) override;
};

//--------Single Support Trajectory definition

SingleSupportTrajectory::SingleSupportTrajectory(const double &startTime,
                                                 const double &endTime,
                                                 const double &omega,
                                                 const iDynTree::Vector2 &ZMP,
                                                 const DCMTrajectoryPoint& boundaryCondition):
    GeneralSupportTrajectory(startTime, endTime, omega),
    m_ZMP(ZMP)
{
    m_boundaryConditionTime = boundaryCondition.time;
    m_boundaryConditionDCMPosition = boundaryCondition.DCMPosition;
}

bool SingleSupportTrajectory::getZMPPosition(const double &t, iDynTree::Vector2 &ZMPPosition,
                                             const bool &checkDomainCondition)
{
    // Evaluate the position of the ZMP at time t
    if (checkDomainCondition)
        if (!timeBelongsToDomain(t)){
            std::cerr << "[SINGLE SUPPORT TRAJECTORY] the time t: " << t
                      << " does not belong to the trajectory domain." << std::endl;
            return false;
        }

    ZMPPosition = m_ZMP;
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

    iDynTree::Vector4 m_coefficentsX; /**< 3-th order x-trajectory parameters [a3, a2, a1, a0] */
	iDynTree::Vector4 m_coefficentsY; /**< 3-th order y-trajectory parameters [a3, a2, a1, a0] */

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
    iDynTree::Vector4 polinominalInterpolation(const iDynTree::Vector2 &positionBoundaryConds,
                                               const iDynTree::Vector2 &velocityBoundaryConds,
                                               const double &dsDuration);



public:

    /**
     * Constructor.
     * @param initBoundaryCondition desired init position and velocity of the
     * double support trajectory
     * @param finalBoundaryCondition desired final position and velocity of the
     * double support trajectory
     * @param omega time constant of the 3D-LIPM
     */
    DoubleSupportTrajectory(const DCMTrajectoryPoint &initBoundaryCondition,
                            const DCMTrajectoryPoint &finalBoundaryCondition,
                            const double& omega);

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

    /**
     * Implementation of the getZMPPosition method of the
     * GeneralSupportTrajectory class.
     * @param t is the trajectory evaluation time;
     * @param ZMPPosition cartesian position of the Zero Moment Point;
     * @param checkDomainCondition flag used to check if the time belongs to the trajectory domain (default value true).
     * @return true / false in case of success / failure.
     */
    virtual bool getZMPPosition(const double &t, iDynTree::Vector2& ZMPVelocity, const bool &checkDomainCondition = true) override;

};

//--------Double Support Trajectory definition

DoubleSupportTrajectory::DoubleSupportTrajectory(const DCMTrajectoryPoint &initBoundaryCondition,
                                                 const DCMTrajectoryPoint &finalBoundaryCondition,
                                                 const double &omega):
    GeneralSupportTrajectory(initBoundaryCondition.time, finalBoundaryCondition.time, omega)
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

iDynTree::Vector4 DoubleSupportTrajectory::polinominalInterpolation(const iDynTree::Vector2 &positionBoundaryConds,
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

	iDynTree::Vector4 output;

	iDynTree::toEigen(output) = coefficents;

    return output;
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
    DCMPosition(0) = tVector.dot(iDynTree::toEigen(m_coefficentsX));
    DCMPosition(1) = tVector.dot(iDynTree::toEigen(m_coefficentsY));

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
    DCMVelocity(0) = tVector.dot(iDynTree::toEigen(m_coefficentsX));
    DCMVelocity(1) = tVector.dot(iDynTree::toEigen(m_coefficentsY));

    return true;
}

bool DoubleSupportTrajectory::getZMPPosition(const double &t, iDynTree::Vector2 &ZMPPosition,
                                             const bool &checkDomainCondition)
{
    // Evaluate the position of the ZMP at time t
    if (checkDomainCondition)
        if (!timeBelongsToDomain(t)){
            std::cerr << "[DOUBLE SUPPORT TRAJECTORY] the time t: " << t
                      << " does not belong to the trajectory domain." << std::endl;
            return false;
        }

    // We can avoid to check the Domain condition since it was already evaluated above
    iDynTree::Vector2 DCMPosition, DCMVelocity;
    getDCMPosition(t, DCMPosition);
    getDCMVelocity(t, DCMVelocity);

    iDynTree::toEigen(ZMPPosition) = iDynTree::toEigen(DCMPosition) - iDynTree::toEigen(DCMVelocity) / m_omega;

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

bool DCMTrajectoryGeneratorHelper::getZMPGlobalPosition(const Step* footprint,
                                               iDynTree::Vector2 &zmpInGlobalCoordinates) const
{
    iDynTree::Vector2 zmpDelta;
    if(footprint->footName == "left"){
        zmpDelta = m_leftZMPDelta;
    }
    else if(footprint->footName == "right"){
        zmpDelta = m_rightZMPDelta;
    }
    else{
        std::cerr << "[DCMTrajectoryGeneratorHelper::getZMPDelta] The name of the footprint is neither left nor right. Does iCub have more legs?!"
                  << std::endl;
        return false;
    }


    double yawAngle = footprint->angle;
    zmpInGlobalCoordinates(0) = footprint->position(0) + cos(yawAngle) * zmpDelta(0) - sin(yawAngle) * zmpDelta(1);
    zmpInGlobalCoordinates(1) = footprint->position(1) + sin(yawAngle) * zmpDelta(0) + cos(yawAngle) * zmpDelta(1);

    return true;
}

bool DCMTrajectoryGeneratorHelper::computeFeetWeight(const std::vector<StepPhase> &lFootPhases, const std::vector<size_t> &phaseShift,
                                                     const FootPrint &left, const FootPrint &right,
                                                     const std::vector<iDynTree::Vector2> &zmpPosition)
{
    {
        m_weightInLeft.resize(zmpPosition.size());
        m_weightInRight.resize(zmpPosition.size());

        Eigen::Vector2d feetDistance;
        Eigen::Vector2d ZMPDistanceFromLeftFoot;

        iDynTree::Position leftFootZMPOffset;
        iDynTree::Position rightFootZMPOffset;

        size_t instant = 0;
        size_t endOfPhase;

        const StepList& leftSteps = left.getSteps();
        StepList::const_iterator leftState = leftSteps.begin();

        const StepList& rightSteps = right.getSteps();
        StepList::const_iterator rightState = rightSteps.begin();

        iDynTree::Vector2 leftFootPosition, rightFootPosition;
        double leftYawAngle, rightYawAngle;

        for (size_t phase = 1; phase < phaseShift.size(); ++phase){ //the first value is useless (it is simply 0)
            endOfPhase = phaseShift[phase];

            if (lFootPhases[instant] == StepPhase::Stance){
                while (instant < endOfPhase){
                    m_weightInLeft[instant] = 1.0;
                    m_weightInRight[instant] = 0.0;
                    instant++;
                }

                if (rightState + 1 == rightSteps.cend()){
                    std::cerr << "[DCMTrajectoryGenerator::computeFeetWeight] Something went wrong. The step phases are not coherent with the right foot." << std::endl; //It's not possible to have a swing phase as last phase.
                    return false;
                }

                ++rightState;
            } else if (lFootPhases[instant] == StepPhase::Swing){
                while (instant < endOfPhase){
                    m_weightInLeft[instant] = 1.0;
                    m_weightInRight[instant] = 0.0;
                    instant++;
                }

                if (leftState + 1 == leftSteps.cend()){
                    std::cerr << "[DCMTrajectoryGenerator::computeFeetWeight] Something went wrong. The step phases are not coherent with the left foot." << std::endl; //It's not possible to have a swing phase as last phase.
                    return false;
                }

                ++leftState;
            } else {

                leftYawAngle = leftState->angle;
                leftFootPosition(0) = leftState->position(0) + cos(leftYawAngle) * m_leftZMPDelta(0) - sin(leftYawAngle) * m_leftZMPDelta(1);
                leftFootPosition(1) = leftState->position(1) + sin(leftYawAngle) * m_leftZMPDelta(0) + cos(leftYawAngle) * m_leftZMPDelta(1);

                rightYawAngle = rightState->angle;
                rightFootPosition(0) = rightState->position(0) + cos(rightYawAngle) * m_rightZMPDelta(0) - sin(rightYawAngle) * m_rightZMPDelta(1);
                rightFootPosition(1) = rightState->position(1) + sin(rightYawAngle) * m_rightZMPDelta(0) + cos(rightYawAngle) * m_rightZMPDelta(1);

                while (instant < endOfPhase){
                    feetDistance = iDynTree::toEigen(rightFootPosition) - iDynTree::toEigen(leftFootPosition);

                    ZMPDistanceFromLeftFoot = iDynTree::toEigen(zmpPosition[instant]) - iDynTree::toEigen(leftFootPosition);

                    m_weightInLeft[instant] = ZMPDistanceFromLeftFoot.norm() / feetDistance.norm();
                    m_weightInRight[instant] = 1 - ZMPDistanceFromLeftFoot.norm() / feetDistance.norm();
                    ++instant;
                }

            }

        }
        return true;
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

    // only for the last step the final position of the DCM coincides with le initial position
    doubleSupportFinalBoundaryCondition.DCMPosition = singleSupportBoundaryCondition.DCMPosition;
    doubleSupportInitBoundaryCondition.DCMPosition = singleSupportBoundaryCondition.DCMPosition;

    doubleSupportInitBoundaryCondition.DCMVelocity.zero();
    doubleSupportFinalBoundaryCondition.DCMVelocity.zero();

    std::shared_ptr<DoubleSupportTrajectory> newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                                                          doubleSupportFinalBoundaryCondition,
                                                                                                          m_omega);
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
        iDynTree::Vector2 ZMPAtNextSingleSupport;
        nextSingleSupport->getZMPPosition(0, ZMPAtNextSingleSupport, false);
        iDynTree::toEigen(doubleSupportStanceInitBoundaryCondition.DCMPosition) = (iDynTree::toEigen(ZMP) + iDynTree::toEigen(ZMPAtNextSingleSupport)) / 2;
        doubleSupportStanceInitBoundaryCondition.DCMVelocity.zero();
        // the constraints at the beginning and at the end of the double support stance phases are equal except for the times
        doubleSupportStanceFinalBoundaryCondition = doubleSupportStanceInitBoundaryCondition;

        doubleSupportStanceInitBoundaryCondition.time = doubleSupportInitBoundaryCondition.time + m_nominalDoubleSupportDuration / 2;
        doubleSupportStanceFinalBoundaryCondition.time = doubleSupportFinalBoundaryCondition.time - m_nominalDoubleSupportDuration / 2;

        // add the 3-th part of the Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportStanceFinalBoundaryCondition,
                                                                     doubleSupportFinalBoundaryCondition,
                                                                     m_omega);
        m_trajectory.push_back(newDoubleSupport);

        // add 2-th part of the Double Support phase (stance phase)
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportStanceInitBoundaryCondition,
                                                                     doubleSupportStanceFinalBoundaryCondition,
                                                                     m_omega);
        m_trajectory.push_back(newDoubleSupport);

        // add 1-th part of the Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                     doubleSupportStanceInitBoundaryCondition,
                                                                     m_omega);
        m_trajectory.push_back(newDoubleSupport);
    }else{
        // add the new Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                     doubleSupportFinalBoundaryCondition,
                                                                     m_omega);
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

        if(!getZMPGlobalPosition(firstSwingFoot, positionOfTheFirstSwingFoot)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::addFirstDoubleSupportPhase] Unable to get the ZMP Delta."
                      << std::endl;
            return false;
        }

        iDynTree::Vector2 ZMPAtNextSingleSupport;
        nextSingleSupport->getZMPPosition(0, ZMPAtNextSingleSupport, false);
        iDynTree::toEigen(doubleSupportStanceInitBoundaryCondition.DCMPosition) = (iDynTree::toEigen(positionOfTheFirstSwingFoot) + iDynTree::toEigen(ZMPAtNextSingleSupport)) / 2;
        doubleSupportStanceInitBoundaryCondition.DCMVelocity.zero();
        // the constraints at the beginning and at the end of the double support stance phases are equal except for the times
        doubleSupportStanceFinalBoundaryCondition = doubleSupportStanceInitBoundaryCondition;

        doubleSupportStanceInitBoundaryCondition.time = doubleSupportInitBoundaryCondition.time + m_nominalDoubleSupportDuration / 2;
        doubleSupportStanceFinalBoundaryCondition.time = doubleSupportFinalBoundaryCondition.time - m_nominalDoubleSupportDuration / 2;

        // add the 3-th part of the Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportStanceFinalBoundaryCondition,
                                                                     doubleSupportFinalBoundaryCondition,
                                                                     m_omega);
        m_trajectory.push_back(newDoubleSupport);

        // add 2-th part of the Double Support phase (stance phase)
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportStanceInitBoundaryCondition,
                                                                     doubleSupportStanceFinalBoundaryCondition,
                                                                     m_omega);
        m_trajectory.push_back(newDoubleSupport);

        // add 1-th part of the Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                     doubleSupportStanceInitBoundaryCondition,
                                                                     m_omega);
        m_trajectory.push_back(newDoubleSupport);
    }else{
        // add the new Double Support phase
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                     doubleSupportFinalBoundaryCondition,
                                                                     m_omega);
        // add the new Double Support phase
        m_trajectory.push_back(newDoubleSupport);
    }

    return true;

}

bool DCMTrajectoryGeneratorHelper::generateDCMTrajectory(const std::vector<const Step*>& orderedSteps,
                                                         const std::vector<StepPhase> &lFootPhases,
                                                         const FootPrint &left, const FootPrint &right,
                                                         const iDynTree::Vector2 &initPosition,
                                                         const iDynTree::Vector2 &initVelocity,
                                                         const std::vector<size_t> &phaseShift)
{
    if (orderedSteps.size() < 2) {
        std::cerr << "[ERROR][DCMTrajectoryGeneratorHelper::generateDCMTrajectory] The orderedSteps vector is supposed to contain at least two elements.";
        std::cerr << "(the intial positions of the feet)." << std::endl;
        return false;
    }

    m_trajectoryDomain = std::make_pair(phaseShift.front(), phaseShift.back());

    // reset the trajectory
    m_trajectory.clear();

    // add the first stance foot at the beginning of the orderedStep vector
    double singleSupportStartTime;
    double singleSupportEndTime;
    double doubleSupportEndTime;
    double singleSupportBoundaryConditionTime;
    DCMTrajectoryPoint singleSupportBoundaryCondition;
    iDynTree::Vector2 lastZMP;
    iDynTree::Vector2 endDCMPosition;

    //The timings computed from here on assume that the first instant is at 0.0

    if (orderedSteps.size() > 2) { //At least one step is made

        size_t phaseIndex = phaseShift.size() - 1;
        size_t orderedStepsIndex = orderedSteps.size() - 1;

        doubleSupportEndTime = (phaseShift[phaseIndex]) * m_dT; //The last element of the phaseShift vector corresponds to the length of the trajectory, and the last phase is supposed to be a double support phase.
        phaseIndex--;

        singleSupportEndTime = (phaseShift[phaseIndex]) * m_dT; //The last but one element indicates the beginning of the last double support phase, thus the end of the single support phase
        phaseIndex--;

        singleSupportStartTime = (phaseShift[phaseIndex]) * m_dT;
        phaseIndex--;


        // the ZMP is shifted before evaluate the DCM
        if(!getZMPGlobalPosition(orderedSteps[orderedStepsIndex], lastZMP)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Unable to get the ZMP Position."
                      << std::endl;
            return false;
        }
        orderedStepsIndex--;

        // evaluate the position of the Center of mass at the end of the trajectory
        iDynTree::Vector2 otherFootZMP;
        if(!getZMPGlobalPosition(orderedSteps[orderedStepsIndex], otherFootZMP)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Unable to get the ZMP Position."
                      << std::endl;
            return false;
        }
        orderedStepsIndex--;
        iDynTree::toEigen(endDCMPosition)  = (iDynTree::toEigen(lastZMP) + iDynTree::toEigen(otherFootZMP)) / 2; //middle of the last two elements of ordered step

        // note: the boundary condition of the single support trajectory
        //       coincides with the end time of the single support trajectory
        //       (valid only for the last step)
        singleSupportBoundaryCondition.time = singleSupportEndTime;
        singleSupportBoundaryCondition.DCMPosition = endDCMPosition;
        singleSupportBoundaryCondition.DCMVelocity.zero();

        // evaluate the last step
        if(!addLastStep(singleSupportStartTime, singleSupportEndTime,
                        doubleSupportEndTime, otherFootZMP, singleSupportBoundaryCondition)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Error when the DCM trajectory of the last step is generated." << std::endl;
            return false;
        }

        while (phaseIndex > 1){

            singleSupportEndTime = phaseShift[phaseIndex] * m_dT;
            phaseIndex--;

            singleSupportStartTime = phaseShift[phaseIndex] * m_dT;
            phaseIndex--;

            // get the next Single Support trajectory (remember that we are constructing the trajectory from the end)
            std::shared_ptr<GeneralSupportTrajectory> nextSingleSupportTrajectory = m_trajectory.back();
            double nextsingleSupportStartTime = nextSingleSupportTrajectory->getTrajectoryDomain().first;
            singleSupportBoundaryConditionTime = (nextsingleSupportStartTime + singleSupportEndTime) / 2;

            // the ZMP is shifted before evaluate the DCM
            if(!getZMPGlobalPosition(orderedSteps[orderedStepsIndex], lastZMP)){
                std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Unable to get the ZMP Position."
                          << std::endl;
                return false;
            }
            orderedStepsIndex--;


            // evaluate the single support boundary condition
            singleSupportBoundaryCondition.time = singleSupportBoundaryConditionTime;
            // NOTE: the DCM at the boundary condition time is outside the SS subtrajectory
            nextSingleSupportTrajectory->getDCMPosition(singleSupportBoundaryConditionTime, singleSupportBoundaryCondition.DCMPosition, false);
            // the DCM velocity is not taken into account in the new SS trajectory generation
            singleSupportBoundaryCondition.DCMVelocity.zero();

            if(!addNewStep(singleSupportStartTime, singleSupportEndTime,
                           lastZMP, singleSupportBoundaryCondition)){
                std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Error when the DCM trajectory of a new step is generated." << std::endl;
                return false;
            }
        }

        // evaluate the first double support phase

        DCMTrajectoryPoint firstDoubleSupportInitialCondition;
        firstDoubleSupportInitialCondition.DCMPosition = initPosition;
        firstDoubleSupportInitialCondition.DCMVelocity = initVelocity;

        firstDoubleSupportInitialCondition.time = 0.0; //the first phase is a double support, thus its end is given by the beninning of the second phase

        const Step* firstSwingFoot;

        if (lFootPhases[0] == StepPhase::SwitchIn) {
            firstSwingFoot = (orderedSteps[0]->footName == "right") ? orderedSteps[0] : orderedSteps[1];
        } else {
            firstSwingFoot = (orderedSteps[0]->footName == "left") ? orderedSteps[0] : orderedSteps[1];
        }

        if(!addFirstDoubleSupportPhase(firstDoubleSupportInitialCondition, firstSwingFoot)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Error when the DCM trajectory of the first DS phase is generated." << std::endl;
            return false;
        }
    } else {

        // instantiate position and velocity boundary conditions vectors
        DCMTrajectoryPoint doubleSupportInitBoundaryCondition;
        DCMTrajectoryPoint doubleSupportFinalBoundaryCondition;

        // set the initial boundary conditions
        doubleSupportInitBoundaryCondition.time = phaseShift.front() * m_dT;
        doubleSupportInitBoundaryCondition.DCMPosition = initPosition;
        doubleSupportInitBoundaryCondition.DCMVelocity = initVelocity;

        iDynTree::Vector2 firstFootZMP, secondFootZMP;

        if(!getZMPGlobalPosition(orderedSteps[0], firstFootZMP)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Unable to get the ZMP Position."
                      << std::endl;
            return false;
        }

        if(!getZMPGlobalPosition(orderedSteps[1], secondFootZMP)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Unable to get the ZMP Position."
                      << std::endl;
            return false;
        }

        // set the final boundary conditions
        doubleSupportFinalBoundaryCondition.time = phaseShift.back() * m_dT;
        iDynTree::toEigen(doubleSupportFinalBoundaryCondition.DCMPosition) = (iDynTree::toEigen(firstFootZMP) + iDynTree::toEigen(secondFootZMP)) / 2;
        doubleSupportFinalBoundaryCondition.DCMVelocity.zero();

        std::shared_ptr<GeneralSupportTrajectory> newDoubleSupport = nullptr;
        newDoubleSupport = std::make_shared<DoubleSupportTrajectory>(doubleSupportInitBoundaryCondition,
                                                                     doubleSupportFinalBoundaryCondition,
                                                                     m_omega);
        // add the new Double Support phase
        m_trajectory.push_back(newDoubleSupport);
    }


    // evaluate the DCM trajectory
    if(!evaluateDCMTrajectory()){
        std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Error when the whole DCM trajectory is evaluated." << std::endl;
        return false;
    }

    if (!computeFeetWeight(lFootPhases, phaseShift, left, right, m_ZMPPosition)) {
        std::cerr << "[DCMTrajectoryGeneratorHelper::generateDCMTrajectory] Failed while computing the feet weight." << std::endl;
        return false;
    }

    return true;
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

    iDynTree::Vector2 DCMPosition, DCMVelocity, ZMPPosition;
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

        if(!(*subTrajectory)->getZMPPosition(time, ZMPPosition)){
            std::cerr << "[DCMTrajectoryGeneratorHelper::evaluateDCMTrajectory] Error when the position of the ZMP is evaluated." << std::endl;
            return false;
        }

        m_DCMPosition.push_back(DCMPosition);
        m_DCMVelocity.push_back(DCMVelocity);
        m_ZMPPosition.push_back(ZMPPosition);
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

void DCMTrajectoryGeneratorHelper::getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInRight) const
{
    weightInLeft = m_weightInLeft;
    weightInRight = m_weightInRight;
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
