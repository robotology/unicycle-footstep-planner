/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicycleOptimization.h"
#include "iDynTree/Core/VectorFixSize.h"

void UnicycleOptimization::updateState(const iDynTree::Vector2 &rPl, double deltaAngle, double deltaTime, const iDynTree::Vector2 &newStepPosition, const iDynTree::Vector2 &newStepRelativePosition)
{
    m_stateBuffer(0) = rPl(0);
    m_stateBuffer(1) = rPl(1);
    m_stateBuffer(2) = deltaAngle;
    m_stateBuffer(3) = deltaTime;
    m_stateBuffer(4) = newStepPosition(0);
    m_stateBuffer(5) = newStepPosition(1);
    m_stateBuffer(6) = newStepRelativePosition(0);
    m_stateBuffer(7) = newStepRelativePosition(1);
}

UnicycleOptimization::UnicycleOptimization()
{
    m_lengthConstraint = std::make_shared<MaxLength>();
    m_widthConstraint = std::make_shared<MinWidth>();
    m_angleConstraint = std::make_shared<MaxAngle>();
    m_stepInEllipsoidConstraint = std::make_shared<StepInEllipsoid>();
    m_backwardLengthConstraint = std::make_shared<MaxLengthBackward>();

    m_cost = std::make_shared<UnicycleCost>();

    m_problem.addConstraint(m_lengthConstraint);
    m_problem.addConstraint(m_widthConstraint);
    m_problem.addConstraint(m_angleConstraint);
    m_problem.addConstraint(m_stepInEllipsoidConstraint);
    m_problem.addConstraint(m_backwardLengthConstraint);

    m_problem.addLagrangeTerm(1.0, m_cost);

    m_stateBuffer.resize(8);
}

bool UnicycleOptimization::setMaxLength(double maxLength)
{
    if(maxLength < 0){
        std::cerr << "The maxLength is supposed to be positive." << std::endl;
        return false;
    }

    iDynTree::VectorDynSize upperBound;
    upperBound.resize(1);
    upperBound(0) = maxLength;

    return m_lengthConstraint->setUpperBound(upperBound);
}

bool UnicycleOptimization::setMaxLengthBackward(double maxLengthBackward)
{
    if(maxLengthBackward < 0){
        std::cerr << "The maxLengthBackward is supposed to be positive." << std::endl;
        return false;
    }

    iDynTree::VectorDynSize upperBound;
    upperBound.resize(1);
    upperBound(0) = maxLengthBackward;

    return m_backwardLengthConstraint->setUpperBound(upperBound);
}

bool UnicycleOptimization::setMinWidth(double minWidth)
{
    if(minWidth < 0){
        std::cerr << "The minWidth is supposed to be positive." << std::endl;
        return false;
    }

    iDynTree::VectorDynSize lowerBound;
    lowerBound.resize(1);
    lowerBound(0) = minWidth;

    return m_widthConstraint->setLowerBound(lowerBound);
}

bool UnicycleOptimization::setMaxAngleVariation(double maxAngle)
{
    if(maxAngle < 0){
        std::cerr << "The maxLength is supposed to be positive." << std::endl;
        return false;
    }

    iDynTree::VectorDynSize upperBound;
    upperBound.resize(1);
    upperBound(0) = maxAngle;

    return m_angleConstraint->setUpperBound(upperBound);
}

bool UnicycleOptimization::setFreeSpaceEllipse(const FreeSpaceEllipse &freeSpaceEllipse)
{
    return m_stepInEllipsoidConstraint->setFreeSpaceEllipse(freeSpaceEllipse);
}

bool UnicycleOptimization::setCostWeights(double positionWeight, double timeWeight)
{
    return m_cost->setPositionWeight(positionWeight) && m_cost->setTimeWeight(timeWeight);
}

bool UnicycleOptimization::getCostValue(const iDynTree::Vector2 &rPl, double deltaAngle, double deltaTime, const iDynTree::Vector2 &newStepPosition, const iDynTree::Vector2 &newStepRelativePosition, double &cost)
{
    updateState(rPl, deltaAngle, deltaTime, newStepPosition, newStepRelativePosition);

    if(!m_problem.costsEvaluation(0.0, m_stateBuffer, m_emptyBuffer, cost)){
        std::cerr <<"Error evaluating the cost." << std::endl;
        return false;
    }

    return true;
}

bool UnicycleOptimization::areConstraintsSatisfied(const iDynTree::Vector2 &rPl, double deltaAngle, double deltaTime, const iDynTree::Vector2 &newStepPosition, const iDynTree::Vector2 &newStepRelativePosition)
{
    updateState(rPl, deltaAngle, deltaTime, newStepPosition, newStepRelativePosition);

    return m_problem.isFeasiblePoint(0.0, m_stateBuffer, m_emptyBuffer);
}

void UnicycleOptimization::printViolatedConstraints(const iDynTree::Vector2 &rPl, double deltaAngle, double deltaTime, const iDynTree::Vector2 &newStepPosition, const iDynTree::Vector2 &newStepRelativePosition)
{
    updateState(rPl, deltaAngle, deltaTime, newStepPosition, newStepRelativePosition);

    if (!(m_lengthConstraint->isFeasiblePoint(0.0, m_stateBuffer, m_emptyBuffer))) {
        std::cerr << "  - Maximum length constraint." << std::endl;
    }
    if (!(m_widthConstraint->isFeasiblePoint(0.0, m_stateBuffer, m_emptyBuffer))) {
        std::cerr << "  - Minimum width constraint." << std::endl;
    }
    if (!(m_angleConstraint->isFeasiblePoint(0.0, m_stateBuffer, m_emptyBuffer))) {
        std::cerr << "  - Maximum angle constraint." << std::endl;
    }
    if (!(m_stepInEllipsoidConstraint->isFeasiblePoint(0.0, m_stateBuffer, m_emptyBuffer))) {
        std::cerr << "  - Step in free ellipsoid constraint." << std::endl;
    }
}


MaxLength::MaxLength()
    :iDynTree::optimalcontrol::Constraint(1,"MaxLength")
{
    m_upperBound.resize(1);
    m_upperBound.zero();
}

bool MaxLength::evaluateConstraint(double, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &, iDynTree::VectorDynSize &constraint){
    if(state.size() != 8){
        std::cerr << "Wrong state dimension." << std::endl;
        return false;
    }

    constraint.resize(1);
    constraint(0) = std::sqrt(std::pow(state(0), 2) + std::pow(state(1), 2));
    return true;
}

bool MaxLength::isFeasiblePoint(double time, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &control){
    iDynTree::VectorDynSize constraint;
    if (!evaluateConstraint(time, state, control, constraint))
        return false;
    if (!m_isUpperBounded || (constraint(0) < m_upperBound(0)))
        return true;
    return false;
}

MinWidth::MinWidth()
    :iDynTree::optimalcontrol::Constraint(1,"MinWidth")
{
    m_lowerBound.resize(1);
    m_lowerBound.zero();
}

bool MinWidth::evaluateConstraint(double, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &, iDynTree::VectorDynSize &constraint){
    if(state.size() != 8){
        std::cerr << "Wrong state dimension." << std::endl;
        return false;
    }

    constraint.resize(1);
    constraint(0) = state(1);
    return true;
}

bool MinWidth::isFeasiblePoint(double time, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &control){
    iDynTree::VectorDynSize constraint;
    if (!evaluateConstraint(time, state, control, constraint))
        return false;
    if (!m_isLowerBounded || (constraint(0) > m_lowerBound(0)))
        return true;
    return false;
}

MaxAngle::MaxAngle()
    :iDynTree::optimalcontrol::Constraint(1,"MaxAngle")
{
    m_upperBound.resize(1);
    m_upperBound.zero();
}

bool MaxAngle::evaluateConstraint(double, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &, iDynTree::VectorDynSize &constraint){
    if(state.size() != 8){
        std::cerr << "Wrong state dimension." << std::endl;
        return false;
    }

    constraint.resize(1);
    constraint(0) = std::abs(state(2));
    return true;
}

bool MaxAngle::isFeasiblePoint(double time, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &control){
    iDynTree::VectorDynSize constraint;
    if (!evaluateConstraint(time, state, control, constraint))
        return false;
    if (!m_isUpperBounded || (constraint(0) < m_upperBound(0)))
        return true;
    return false;
}

StepInEllipsoid::StepInEllipsoid()
    :iDynTree::optimalcontrol::Constraint(1,"StepInEllipsoid")
{
    m_lowerBound.resize(1);
    m_lowerBound(0) = 0.5;
    this->setLowerBound(m_lowerBound);
}

bool StepInEllipsoid::setFreeSpaceEllipse(const FreeSpaceEllipse &freeSpaceEllipse)
{
    m_freeSpace = freeSpaceEllipse;
    return true;
}

bool StepInEllipsoid::evaluateConstraint(double, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &, iDynTree::VectorDynSize &constraint){
    if(state.size() != 8){
        std::cerr << "Wrong state dimension." << std::endl;
        return false;
    }

    constraint.resize(1);
    iDynTree::Vector2 newPosition;
    newPosition(0) = state(4);
    newPosition(1) = state(5);

    constraint(0) = m_freeSpace.isPointInside(newPosition);
    return true;
}

MaxLengthBackward::MaxLengthBackward()
    :iDynTree::optimalcontrol::Constraint(1,"MaxLengthBackward")
{
    m_upperBound.resize(1);
    m_upperBound.zero();
}

bool MaxLengthBackward::evaluateConstraint(double, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &, iDynTree::VectorDynSize &constraint){
    if(state.size() != 8){
        std::cerr << "Wrong state dimension." << std::endl;
        return false;
    }

    constraint.resize(1);
    bool walkingBackward = state(6) < 0;
    constraint(0) = state(6) < 0 ? std::sqrt(std::pow(state(6), 2) + std::pow(state(7), 2)) : 0; //The constraint is active only when walking backward
    return true;
}

bool MaxLengthBackward::isFeasiblePoint(double time, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &control){
    iDynTree::VectorDynSize constraint;
    if (!evaluateConstraint(time, state, control, constraint))
        return false;
    if (!m_isUpperBounded || (constraint(0) < m_upperBound(0)))
        return true;
    return false;
}

UnicycleCost::UnicycleCost()
    :iDynTree::optimalcontrol::Cost("UnicycleCost")
{
    m_timeWeight = 0;
    m_positionWeight = 0;
}

bool UnicycleCost::costEvaluation(double time, const iDynTree::VectorDynSize &state, const iDynTree::VectorDynSize &control, double &costValue){
    if(state.size() != 8)
        return false;

    costValue = m_timeWeight * std::pow(1/state(3), 2) + m_positionWeight * (std::pow(state(0), 2) + std::pow(state(1), 2));
    return true;
}

bool UnicycleCost::setPositionWeight(double positionWeight){
    if (positionWeight < 0){
        std::cerr << "The weight is supposed to be nonnegative" << std::endl;
        return false;
    }
    m_positionWeight = positionWeight;
    return true;
}

bool UnicycleCost::setTimeWeight(double timeWeight){
    if (timeWeight < 0){
        std::cerr << "The weight is supposed to be nonnegative" << std::endl;
        return false;
    }
    m_timeWeight = timeWeight;
    return true;
}
