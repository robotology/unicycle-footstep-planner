/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLEOPTIMIZATION_H
#define UNICYCLEOPTIMIZATION_H

#include <iDynTree/Constraint.h>
#include <iDynTree/Cost.h>
#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <cmath>
#include <string>
#include <iostream>
#include <memory>

//the state here is [r_P_l_x; r_P_l_y; deltaAngle; deltaTime]
//The control is not used

class MaxLength : public iDynTree::optimalcontrol::Constraint{
public:
    MaxLength()
        :iDynTree::optimalcontrol::Constraint(1,"MaxLength")
    {
        m_upperBound.resize(1);
        m_upperBound.zero();
    }

    bool evaluateConstraint(double time, const iDynTree::VectorDynSize &state,
                            const iDynTree::VectorDynSize &control,
                            iDynTree::VectorDynSize &constraint){
        if(state.size() != 4){
            std::cerr << "Wrong state dimension." << std::endl;
            return false;
        }

        constraint.resize(1);
        constraint(0) = std::sqrt(std::pow(state(0), 2) + std::pow(state(1), 2));
        return true;
    }

    bool isFeasiblePoint(double time, const iDynTree::VectorDynSize &state,
                         const iDynTree::VectorDynSize &control){
        iDynTree::VectorDynSize constraint;
        if (!evaluateConstraint(time, state, control, constraint))
            return false;
        if (!m_isUpperBounded || (constraint(0) < m_upperBound(0)))
            return true;
        return false;
    }
};

class MinWidth : public iDynTree::optimalcontrol::Constraint{
public:
    MinWidth()
        :iDynTree::optimalcontrol::Constraint(1,"MinWidth")
    {
        m_lowerBound.resize(1);
        m_lowerBound.zero();
    }

    bool evaluateConstraint(double time, const iDynTree::VectorDynSize &state,
                            const iDynTree::VectorDynSize &control,
                            iDynTree::VectorDynSize &constraint){
        if(state.size() != 4){
            std::cerr << "Wrong state dimension." << std::endl;
            return false;
        }

        constraint.resize(1);
        constraint(0) = state(1);
        return true;
    }

    bool isFeasiblePoint(double time, const iDynTree::VectorDynSize &state,
                         const iDynTree::VectorDynSize &control){
        iDynTree::VectorDynSize constraint;
        if (!evaluateConstraint(time, state, control, constraint))
            return false;
        if (!m_isLowerBounded || (constraint(0) > m_lowerBound(0)))
            return true;
        return false;
    }

};


class MaxAngle : public iDynTree::optimalcontrol::Constraint{
public:
    MaxAngle()
        :iDynTree::optimalcontrol::Constraint(1,"MaxAngle")
    {
        m_upperBound.resize(1);
        m_upperBound.zero();
    }

    bool evaluateConstraint(double time, const iDynTree::VectorDynSize &state,
                            const iDynTree::VectorDynSize &control,
                            iDynTree::VectorDynSize &constraint){
        if(state.size() != 4){
            std::cerr << "Wrong state dimension." << std::endl;
            return false;
        }

        constraint.resize(1);
        constraint(0) = std::abs(state(2));
        return true;
    }

    bool isFeasiblePoint(double time, const iDynTree::VectorDynSize &state,
                         const iDynTree::VectorDynSize &control){
        iDynTree::VectorDynSize constraint;
        if (!evaluateConstraint(time, state, control, constraint))
            return false;
        if (!m_isUpperBounded || (constraint(0) < m_upperBound(0)))
            return true;
        return false;
    }
};

class UnicycleCost : public iDynTree::optimalcontrol::Cost{
    double m_timeWeight;
    double m_positionWeight;
public:

    UnicycleCost()
        :iDynTree::optimalcontrol::Cost("UnicycleCost")
    {
        m_timeWeight = 0;
        m_positionWeight = 0;
    }

    bool costEvaluation(double time, const iDynTree::VectorDynSize &state,
                        const iDynTree::VectorDynSize &control, double &costValue){
        if(state.size() != 4)
            return false;

        costValue = m_timeWeight * std::pow(1/state(3), 2) + m_positionWeight * (std::pow(state(0), 2) + std::pow(state(1), 2));
        return true;
    }

    bool setPositionWeight(double positionWeight){
        if (positionWeight < 0){
            std::cerr << "The weight is supposed to be nonnegative" << std::endl;
            return false;
        }
        m_positionWeight = positionWeight;
        return true;
    }

    bool setTimeWeight(double timeWeight){
        if (timeWeight < 0){
            std::cerr << "The weight is supposed to be nonnegative" << std::endl;
            return false;
        }
        m_timeWeight = timeWeight;
        return true;
    }

};

class UnicycleOptimization{
    iDynTree::optimalcontrol::OptimalControlProblem m_problem;
    std::shared_ptr<MaxLength> m_lengthConstraint;
    std::shared_ptr<MinWidth> m_widthConstraint;
    std::shared_ptr<MaxAngle> m_angleConstraint;
    std::shared_ptr<UnicycleCost> m_cost;

    iDynTree::VectorDynSize m_stateBuffer, m_emptyBuffer;

public:
    UnicycleOptimization();
    //constraints setting
    bool setMaxLength(double maxLength);
    bool setMinWidth(double minWidth);
    bool setMaxAngleVariation(double maxAngle); //in radians!

    //cost setting
    bool setCostWeights(double positionWeight, double timeWeight);

    bool getCostValue(const iDynTree::Vector2& rPl, double deltaAngle, double deltaTime, double& cost);
    bool areConstraintsSatisfied(const iDynTree::Vector2& rPl, double deltaAngle, double deltaTime);
    void printViolatedConstraints(const iDynTree::Vector2& rPl, double deltaAngle, double deltaTime);
};




#endif // UNICYCLEOPTIMIZATION_H
