/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLETRAJECTORYGENERATOR_H
#define UNICYCLETRAJECTORYGENERATOR_H

#include "UnicyclePlanner.h"
#include "FootPrintsInterpolator.h"
#include <memory>

class UnicycleTrajectoryGenerator : public UnicyclePlanner, public FeetInterpolator
{
public:
    UnicycleTrajectoryGenerator();

    //DO NOT FORGET TO CALL ALL THE INITIALIZATION METHODS OF BOTH FEETINTERPOLATOR AND UNICYCLEPLANNER

    bool generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT,
                                const InitialState &weightInLeftAtMergePoint); //both feet are supposed to start on the ground at zero velocity. The initTime must be greater than the maximum of the first impactTime of the two feet. The first step has half switch time. The FootPrints needs to be ordered! warmStart must be set to false if it is the very first step

    bool generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT); //the initial call
};

#endif // UNICYCLETRAJECTORYGENERATOR_H
