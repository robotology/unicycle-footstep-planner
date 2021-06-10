/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicycleFoot.h"
#include "Eigen/Core"
#include "iDynTree/Core/EigenHelpers.h"
#include <cmath>
#include <iostream>

bool UnicycleFoot::computeRotationMatrix(double theta, iDynTree::MatrixDynSize &R)
{
    double c_theta = std::cos(theta);
    double s_theta = std::sin(theta);

    if((R.rows() != 2)||(R.cols() != 2))
        R.resize(2,2);

    R(0,0) = c_theta;
    R(0,1) = -s_theta;
    R(1,0) = s_theta;
    R(1,1) = c_theta;

    return true;
}

UnicycleFoot::UnicycleFoot(std::shared_ptr<FootPrint> footPrint_ptr)
    :m_steps_ptr(footPrint_ptr)
{
    m_distanceSet = false;
    m_distance.zero();
    m_bufferR.resize(2,2);
    iDynTree::toEigen(m_bufferR).setIdentity();
    m_yawOffset = 0.0;
}

bool UnicycleFoot::setDistanceFromUnicycle(const iDynTree::Vector2 &nominalDistance)
{
    m_distance = nominalDistance;
    m_distanceSet = true;
    return true;
}

const iDynTree::Vector2 &UnicycleFoot::distanceFromUnicycle() const
{
    return m_distance;
}

bool UnicycleFoot::setYawOffsetInRadians(double yawOffsetInRadians)
{
    m_yawOffset = yawOffsetInRadians;
    return true;
}

double UnicycleFoot::yawOffsetInRadians() const
{
    return m_yawOffset;
}

bool UnicycleFoot::addStepFromUnicycle(const UnicycleState &unicycleState, double impactTime)
{
    if(!m_distanceSet){
        std::cerr << "First you have to set the unicycle distance." << std::endl;
        return false;
    }
    Step newStep;

    newStep.impactTime = impactTime;
    newStep.angle = unicycleState.angle + m_yawOffset;

    computeRotationMatrix(unicycleState.angle, m_bufferR);

    iDynTree::toEigen(newStep.position) = iDynTree::toEigen(unicycleState.position) + iDynTree::toEigen(m_bufferR)*iDynTree::toEigen(m_distance);

    return m_steps_ptr->addStep(newStep);
}

bool UnicycleFoot::addParallelStep(const UnicycleFoot &otherFoot, double impactTime)
{
    if (impactTime < 0){
        std::cerr << "The impactTime is expected to be non-negative" << std::endl;
        return false;
    }

    if (otherFoot.numberOfSteps() == 0){
        std::cerr << "Cannot take a step with respect the other foot if its numberOfSteps is 0." << std::endl;
        return false;
    }

    Step newStep, otherStep;

    if (!otherFoot.getLastStep(otherStep))
        return false;

    newStep.impactTime = impactTime;
    double unicycleAngle = otherFoot.getUnicycleAngleFromStep(otherStep);
    newStep.angle = unicycleAngle + m_yawOffset;

    computeRotationMatrix(unicycleAngle, m_bufferR);

    iDynTree::toEigen(newStep.position) = iDynTree::toEigen(otherStep.position) + iDynTree::toEigen(m_bufferR)*(iDynTree::toEigen(m_distance) - iDynTree::toEigen(otherFoot.distanceFromUnicycle()));

    return m_steps_ptr->addStep(newStep);
}

bool UnicycleFoot::getLastStep(Step &lastStep) const
{
    return m_steps_ptr->getLastStep(lastStep);
}

bool UnicycleFoot::setTinyStepLength(double length)
{
    if (length < 0){
        std::cerr << "The length is supposed to be non-negative." <<std::endl;
        return false;
    }
    m_minimumStep = length;
    return true;
}

bool UnicycleFoot::isTinyStep(const UnicycleState &unicycleState)
{
    Step previousStep;
    if (!m_steps_ptr->getLastStep(previousStep))
        return false;

    iDynTree::Vector2& previousPosition = previousStep.position;

    computeRotationMatrix(unicycleState.angle, m_bufferR);
    iDynTree::Vector2 newPosition;
    iDynTree::toEigen(newPosition) = iDynTree::toEigen(unicycleState.position) + iDynTree::toEigen(m_bufferR)*iDynTree::toEigen(m_distance);

    double stepLength = std::sqrt(std::pow((newPosition(0) - previousPosition(0)), 2) + std::pow((newPosition(1) - previousPosition(1)), 2));

    return (stepLength < m_minimumStep);
}

size_t UnicycleFoot::numberOfSteps() const
{
    return m_steps_ptr->numberOfSteps();
}

bool UnicycleFoot::getUnicycleStateFromStep(const Step& inputStep,
                                            UnicycleState &unicycleState)
{
    if(!m_distanceSet){
        std::cerr << "First you have to set the unicycle distance." << std::endl;
        return false;
    }
    unicycleState.angle = getUnicycleAngleFromStep(inputStep);
    computeRotationMatrix(unicycleState.angle, m_bufferR);

    iDynTree::toEigen(unicycleState.position) =
            iDynTree::toEigen(inputStep.position) - iDynTree::toEigen(m_bufferR)*iDynTree::toEigen(m_distance);

    return true;
}

double UnicycleFoot::getUnicycleAngleFromStep(const Step &inputStep) const
{
    return inputStep.angle - m_yawOffset;
}

bool UnicycleFoot::getFootPositionFromUnicycle(const UnicycleState &unicycleState, iDynTree::Vector2 &footPosition)
{
    if(!m_distanceSet){
        std::cerr << "First you have to set the unicycle distance." << std::endl;
        return false;
    }

    computeRotationMatrix(unicycleState.angle, m_bufferR);

    iDynTree::toEigen(footPosition) =
            iDynTree::toEigen(unicycleState.position) + iDynTree::toEigen(m_bufferR)*iDynTree::toEigen(m_distance);

    return true;
}
