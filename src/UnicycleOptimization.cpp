/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicycleOptimization.h"
#include "iDynTree/Core/VectorFixSize.h"

UnicycleOptimization::UnicycleOptimization()
{
    m_lengthConstraint = std::make_shared<MaxLength>();
    m_widthConstraint = std::make_shared<MinWidth>();
    m_angleConstraint = std::make_shared<MaxAngle>();

    m_cost = std::make_shared<UnicycleCost>();

    m_problem.addContraint(m_lengthConstraint);
    m_problem.addContraint(m_widthConstraint);
    m_problem.addContraint(m_angleConstraint);

    m_problem.addLagrangeTerm(1.0, m_cost);

    m_stateBuffer.resize(4);
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

bool UnicycleOptimization::setCostWeights(double positionWeight, double timeWeight)
{
    return m_cost->setPositionWeight(positionWeight) && m_cost->setTimeWeight(timeWeight);
}

bool UnicycleOptimization::getCostValue(const iDynTree::Vector2 &rPl, double deltaAngle, double deltaTime, double &cost)
{
    m_stateBuffer(0) = rPl(0);
    m_stateBuffer(1) = rPl(1);
    m_stateBuffer(2) = deltaAngle;
    m_stateBuffer(3) = deltaTime;

    if(!m_problem.costsEvaluation(0.0, m_stateBuffer, m_emptyBuffer, cost)){
        std::cerr <<"Error evaluating the cost." << std::endl;
        return false;
    }

    return true;
}

bool UnicycleOptimization::areConstraintsSatisfied(const iDynTree::Vector2 &rPl, double deltaAngle, double deltaTime)
{
    m_stateBuffer(0) = rPl(0);
    m_stateBuffer(1) = rPl(1);
    m_stateBuffer(2) = deltaAngle;
    m_stateBuffer(3) = deltaTime;

    return m_problem.isFeasiblePoint(0.0, m_stateBuffer, m_emptyBuffer);
}

void UnicycleOptimization::printViolatedConstraints(const iDynTree::Vector2 &rPl, double deltaAngle, double deltaTime)
{
    m_stateBuffer(0) = rPl(0);
    m_stateBuffer(1) = rPl(1);
    m_stateBuffer(2) = deltaAngle;
    m_stateBuffer(3) = deltaTime;
    if (m_lengthConstraint->isFeasiblePoint(0.0, m_stateBuffer, m_emptyBuffer)) {
        std::cerr << "  - Maximum length constraint." << std::endl;
    }
    if (m_widthConstraint->isFeasiblePoint(0.0, m_stateBuffer, m_emptyBuffer)) {
        std::cerr << "  - Minimum width constraint." << std::endl;
    }
    if (m_angleConstraint->isFeasiblePoint(0.0, m_stateBuffer, m_emptyBuffer)) {
        std::cerr << "  - Maximum angle constraint." << std::endl;
    }
}

