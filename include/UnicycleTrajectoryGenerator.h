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
    std::shared_ptr<FootPrint> m_left, m_right;
public:
    UnicycleTrajectoryGenerator();

    //DO NOT FORGET TO CALL ALL THE INITIALIZATION METHODS OF BOTH FEETINTERPOLATOR AND UNICYCLEPLANNER

    bool generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT,
                                const InitialState &weightInLeftAtMergePoint); //both feet are supposed to start on the ground at zero velocity. The initTime must be greater than the maximum of the first impactTime of the two feet. The first step has half switch time. The FootPrints needs to be ordered!

    bool generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT);

    bool generateAndInterpolate(double initTime, double dT, double endTime);

    bool generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT, double endTime);
    
    bool reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint);

    bool reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint,
                    const Step &measuredLeft, const Step &measuredRight);

    bool reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint, bool correctLeft,
                    const iDynTree::Vector2 &measuredPosition, double measuredAngle);


    bool generateAndInterpolateDcm(double initTime, double dT, double endTime);

    bool generateAndInterpolateDcm(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT, double endTime);
    
    bool reGenerateDcm(double initTime, double dT, double endTime, const DcmInitialState &dcmBoundaryConditionAtMergePoint);
};

#endif // UNICYCLETRAJECTORYGENERATOR_H
