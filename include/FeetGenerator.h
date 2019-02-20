/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef FEETGENERATOR_H
#define FEETGENERATOR_H

#include <StepPhase.h>
#include <FootPrint.h>

#include <iDynTree/Core/Transform.h>

#include <vector>

class FeetGenerator {

public:

    virtual ~FeetGenerator();

    virtual bool setStepHeight(double stepHeight) = 0;

    virtual bool setPitchDelta(double pitchAngle = 0.0) = 0;

    virtual bool setFootApexTime(double swingTimeRatio = 0.5) = 0;

    virtual bool setFootLandingVelocity(double landingVelocity = 0.0) = 0;

    virtual void getFeetTrajectories(std::vector<iDynTree::Transform>& lFootTrajectory, std::vector<iDynTree::Transform>& rFootTrajectory) const = 0;

    virtual void getFeetTwistsInMixedRepresentation(std::vector<iDynTree::Twist> &lFootTwistsInMixedRepresentation, std::vector<iDynTree::Twist> &rFootTwistsInMixedRepresentation) const = 0;

    virtual void getFeetAccelerationInMixedRepresentation(std::vector<iDynTree::SpatialAcc> &lFootAccelerationInMixedRepresentation, std::vector<iDynTree::SpatialAcc> &rFootAccelerationInMixedRepresentation) const = 0;
};

#endif // FEETGENERATOR_H
