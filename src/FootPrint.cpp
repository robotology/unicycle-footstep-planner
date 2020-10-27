/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "FootPrint.h"
#include <iostream>
#include <algorithm>

FootPrint::FootPrint()
{
    m_steps = std::make_shared<StepList>();
}

FootPrint::FootPrint(std::shared_ptr<StepList> &steps)
{
    m_steps = steps;
    std::sort(m_steps->begin(), m_steps->end(), [](const Step&a, const Step&b) { return a.impactTime < b.impactTime;});
}

bool FootPrint::addStep(const iDynTree::Vector2 &position, double theta, double impactTime)
{
    if (!m_steps){
        std::cerr << "Empty step pointer." <<std::endl;
        return false;
    }

    if (impactTime < 0){
        std::cerr << "The impactTime is expected to be non-negative" << std::endl;
        return false;
    }

    Step newStep;

    newStep.impactTime = impactTime;
    newStep.angle = theta;
    newStep.position = position;
    newStep.footName = m_footName;

    m_steps->push_back(newStep);

    return true;
}

bool FootPrint::addStep(const Step &newStep)
{
    return addStep(newStep.position, newStep.angle, newStep.impactTime);
}

bool FootPrint::getLastStep(Step &lastStep) const
{
    if (!m_steps)
        return false;

    if(m_steps->size() == 0)
        return false;

    lastStep = m_steps->back();
    return true;
}

bool FootPrint::dropPastSteps()
{
    if( numberOfSteps() < 1)
        return false;
    m_steps->erase(m_steps->begin(), m_steps->begin() + (numberOfSteps()-1));
    return true;
}

bool FootPrint::keepOnlyPresentStep(double time)
{
    Step presentStep;
    StepList::reverse_iterator dropPoint= std::find_if(m_steps->rbegin(), m_steps->rend(),
                                      [time](const Step & a) -> bool { return a.impactTime <= time; });
    if (dropPoint == m_steps->rend()){
        std::cerr << "No step has impactTime lower than the specified time." << std::endl;
        return false;
    }

    presentStep.angle = dropPoint->angle;
    presentStep.position = dropPoint->position;
    presentStep.impactTime = dropPoint->impactTime;
    clearSteps();
    return addStep(presentStep);
}

void FootPrint::clearSteps()
{
    m_steps->clear();
}

void FootPrint::clearLastStep()
{
    if (numberOfSteps() != 0)
        m_steps->pop_back();
}

size_t FootPrint::numberOfSteps() const
{
    if (!m_steps)
        return 0;

    return m_steps->size();
}

const StepList &FootPrint::getSteps() const
{
    return *m_steps;
}

StepList &FootPrint::getSteps()
{
    return *m_steps;
}

void FootPrint::setFootName(const std::string& footName)
{
    m_footName = footName;
}

const std::string& FootPrint::getFootName() const
{
    return m_footName;
}
