/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLEOPTIMIZATION_H
#define UNICYCLEOPTIMIZATION_H

#include "FreeSpaceEllipse.h"
#include <iDynTree/Constraint.h>
#include <iDynTree/Cost.h>
#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <cmath>
#include <string>
#include <iostream>
#include <memory>

//the state here is [r_P_l_x; r_P_l_y; deltaAngle; deltaTime; newStepPosition_x; newStepPosition_y; newStepRelativePosition_x, newStepRelativePosition_y]
//The control is not used

class MaxLength : public iDynTree::optimalcontrol::Constraint{
public:
    MaxLength();

    bool evaluateConstraint(double , const iDynTree::VectorDynSize &state,
                            const iDynTree::VectorDynSize &,
                            iDynTree::VectorDynSize &constraint);

    bool isFeasiblePoint(double time, const iDynTree::VectorDynSize &state,
                         const iDynTree::VectorDynSize &control);
};

class MinWidth : public iDynTree::optimalcontrol::Constraint{
public:
    MinWidth();

    bool evaluateConstraint(double , const iDynTree::VectorDynSize &state,
                            const iDynTree::VectorDynSize &,
                            iDynTree::VectorDynSize &constraint);

    bool isFeasiblePoint(double time, const iDynTree::VectorDynSize &state,
                         const iDynTree::VectorDynSize &control);

};

class MaxAngle : public iDynTree::optimalcontrol::Constraint{
public:
    MaxAngle();

    bool evaluateConstraint(double , const iDynTree::VectorDynSize &state,
                            const iDynTree::VectorDynSize &,
                            iDynTree::VectorDynSize &constraint);

    bool isFeasiblePoint(double time, const iDynTree::VectorDynSize &state,
                         const iDynTree::VectorDynSize &control);
};

class StepInEllipsoid : public iDynTree::optimalcontrol::Constraint{

    FreeSpaceEllipse m_freeSpace;

public:
    StepInEllipsoid();

    bool setFreeSpaceEllipse(const FreeSpaceEllipse& freeSpaceEllipse);

    bool evaluateConstraint(double , const iDynTree::VectorDynSize &state,
                            const iDynTree::VectorDynSize &,
                            iDynTree::VectorDynSize &constraint);
};

class MaxLengthBackward : public iDynTree::optimalcontrol::Constraint{
public:
    MaxLengthBackward();

    bool evaluateConstraint(double , const iDynTree::VectorDynSize &state,
                            const iDynTree::VectorDynSize &,
                            iDynTree::VectorDynSize &constraint);

    bool isFeasiblePoint(double time, const iDynTree::VectorDynSize &state,
                         const iDynTree::VectorDynSize &control);
};

class UnicycleCost : public iDynTree::optimalcontrol::Cost{
    double m_timeWeight;
    double m_positionWeight;
public:

    UnicycleCost();

    bool costEvaluation(double time, const iDynTree::VectorDynSize &state,
                        const iDynTree::VectorDynSize &control, double &costValue);

    bool setPositionWeight(double positionWeight);

    bool setTimeWeight(double timeWeight);

};

class UnicycleOptimization{
    iDynTree::optimalcontrol::OptimalControlProblem m_problem;
    std::shared_ptr<MaxLength> m_lengthConstraint;
    std::shared_ptr<MinWidth> m_widthConstraint;
    std::shared_ptr<MaxAngle> m_angleConstraint;
    std::shared_ptr<StepInEllipsoid> m_stepInEllipsoidConstraint;
    std::shared_ptr<MaxLengthBackward> m_backwardLengthConstraint;
    std::shared_ptr<UnicycleCost> m_cost;

    iDynTree::VectorDynSize m_stateBuffer, m_emptyBuffer;

    void updateState(const iDynTree::Vector2& rPl, double deltaAngle, double deltaTime, const iDynTree::Vector2 &newStepPosition, const iDynTree::Vector2 &newStepRelativePosition);

public:
    UnicycleOptimization();
    //constraints setting
    bool setMaxLength(double maxLength);
    bool setMaxLengthBackward(double maxLengthBackward);
    bool setMinWidth(double minWidth);
    bool setMaxAngleVariation(double maxAngle); //in radians!
    bool setFreeSpaceEllipse(const FreeSpaceEllipse& freeSpaceEllipse);

    //cost setting
    bool setCostWeights(double positionWeight, double timeWeight);

    bool getCostValue(const iDynTree::Vector2& rPl, double deltaAngle, double deltaTime, const iDynTree::Vector2 &newStepPosition, const iDynTree::Vector2 &newStepRelativePosition, double& cost);
    bool areConstraintsSatisfied(const iDynTree::Vector2& rPl, double deltaAngle, double deltaTime, const iDynTree::Vector2 &newStepPosition, const iDynTree::Vector2 &newStepRelativePosition);
    void printViolatedConstraints(const iDynTree::Vector2& rPl, double deltaAngle, double deltaTime, const iDynTree::Vector2 &newStepPosition, const iDynTree::Vector2 &newStepRelativePosition);
};




#endif // UNICYCLEOPTIMIZATION_H
