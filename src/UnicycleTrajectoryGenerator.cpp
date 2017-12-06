/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicycleTrajectoryGenerator.h"

UnicycleTrajectoryGenerator::UnicycleTrajectoryGenerator()
{

}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT, const InitialState& weightInLeftAtMergePoint)
{
    return computeNewSteps(leftFoot, rightFoot) && interpolate(*leftFoot, *rightFoot, initTime, dT, weightInLeftAtMergePoint);
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT)
{
    return computeNewSteps(leftFoot, rightFoot) && interpolate(*leftFoot, *rightFoot, initTime, dT);
}
