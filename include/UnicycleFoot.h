/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLEFOOT_H
#define UNICYCLEFOOT_H

#include <deque>
#include <memory>
#include "FootPrint.h"
#include "iDynTree/Core/VectorFixSize.h"
#include "iDynTree/Core/MatrixDynSize.h"

struct UnicycleState
{
    iDynTree::Vector2 position;
    double angle;
};

class UnicycleFoot {
    std::shared_ptr<FootPrint> m_steps_ptr;
    iDynTree::Vector2 m_distance;
    bool m_distanceSet;
    double m_minimumStep;
    double m_yawOffset;
    iDynTree::MatrixDynSize m_bufferR;

    bool computeRotationMatrix(double theta, iDynTree::MatrixDynSize& R);

public:

    UnicycleFoot(std::shared_ptr<FootPrint> footPrint_ptr);

    bool setDistanceFromUnicycle(const iDynTree::Vector2& nominalDistance);

    const iDynTree::Vector2& distanceFromUnicycle() const;

    bool setYawOffsetInRadians(double yawOffsetInRadians);

    double yawOffsetInRadians() const;

    bool addStepFromUnicycle(const UnicycleState &unicycleState, double impactTime);

    bool addParallelStep(const UnicycleFoot& otherFoot, double impactTime);

    bool getLastStep(Step& lastStep) const;

    bool setTinyStepLength(double length);

    bool isTinyStep(const UnicycleState &unicycleState); //given the position of the unicycle return true if the step would be tiny

    size_t numberOfSteps() const;

    bool getUnicycleStateFromStep(const Step &inputStep, UnicycleState &unicycleState);

    double getUnicycleAngleFromStep(const Step &inputStep) const;

    bool getFootPositionFromUnicycle(const UnicycleState &unicycleState, iDynTree::Vector2 &footPosition);

};

#endif // UNICYCLEFOOT_H
