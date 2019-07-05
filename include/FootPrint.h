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
    std::string footName{"footprint"};
}Step;

typedef std::deque<Step> StepList;

class FootPrint {
    std::shared_ptr<StepList> m_steps;
    std::string m_footName{"footprint"};
public:

    FootPrint();

    FootPrint(std::shared_ptr<StepList> &steps); //it sorts the step

    /* FootPrint(const FootPrint& other) = delete; */

    bool addStep(const iDynTree::Vector2& position, double theta, double impactTime);

    bool addStep(const Step& newStep);

    bool getLastStep(Step& lastStep) const;

    bool dropPastSteps(); //remove all the steps except the last one

    bool keepOnlyPresentStep(double time); //drops all the steps except the last whose impacttime is lower than time.

    void clearSteps();

    void clearLastStep();

    size_t numberOfSteps() const;

    const StepList& getSteps() const;


    StepList& getSteps();

    /**
     * Set the foot name
     * @param footName name of the foot (e.g. left or right)
     */
    void setFootName(const std::string& footName);

    /**
     * Get the foot name
     * @return the name of the foot (e.g. left or right)
     */
    const std::string& getFootName() const;
};

#endif // FOOTPRINT_H
