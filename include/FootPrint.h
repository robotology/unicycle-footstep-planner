/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef FOOTPRINT_H
#define FOOTPRINT_H

#include <deque>
#include <memory>
#include "iDynTree/Core/VectorFixSize.h"

typedef struct{
    iDynTree::Vector2 position;
    double angle;
    double impactTime;
}Step;

typedef std::deque<Step> StepList;

class FootPrint {
    std::shared_ptr<StepList> m_steps;

public:

    FootPrint();

    FootPrint(std::shared_ptr<StepList> &steps); //it sorts the step

    bool addStep(const iDynTree::Vector2& position, double theta, double impactTime);

    bool addStep(const Step& newStep);

    bool getLastStep(Step& lastStep) const;

    bool dropPastSteps(); //remove all the steps except the last one

    bool keepOnlyPresentStep(double time); //drops all the steps except the last whose impacttime is lower than time.

    void clearSteps();

    void clearLastStep();

    size_t numberOfSteps() const;

    const StepList& getSteps() const;

};

#endif // FOOTPRINT_H
