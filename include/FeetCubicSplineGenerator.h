/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef FEETCUBICSPLINEGENERATOR_H
#define FEETCUBICSPLINEGENERATOR_H

#include <StepPhase.h>
#include <FootPrint.h>
#include <FeetGenerator.h>

#include <iDynTree/Core/Transform.h>

#include <vector>
#include <memory>

class FeetCubicSplineGenerator : public FeetGenerator{
    friend class UnicycleGenerator;

    class FeetCubicSplineGeneratorImplementation;
    std::unique_ptr<FeetCubicSplineGeneratorImplementation> m_pimpl;

    FeetCubicSplineGenerator();

    bool computeNewTrajectories(double dT,
                                const FootPrint &left, const FootPrint &right,
                                const std::vector<StepPhase>& lFootPhases,
                                const std::vector<StepPhase>& rFootPhases,
                                const std::vector<size_t>& phaseShift);

public:

    virtual ~FeetCubicSplineGenerator() final;

    virtual bool setStepHeight(double stepHeight) final;

    virtual bool setPitchDelta(double pitchAngle = 0.0) final; //DEGREES

    virtual bool setFootApexTime(double swingTimeRatio = 0.5) final;

    virtual bool setFootLandingVelocity(double landingVelocity = 0.0) final;

    virtual void getFeetTrajectories(std::vector<iDynTree::Transform>& lFootTrajectory, std::vector<iDynTree::Transform>& rFootTrajectory) const final;

    virtual void getFeetTwistsInMixedRepresentation(std::vector<iDynTree::Twist> &lFootTwistsInMixedRepresentation, std::vector<iDynTree::Twist> &rFootTwistsInMixedRepresentation) const final;

   virtual void getFeetAccelerationInMixedRepresentation(std::vector<iDynTree::Vector6> &lFootAccelerationInMixedRepresentation, std::vector<iDynTree::Vector6> &rFootAccelerationInMixedRepresentation) const final;

};

#endif // FEETCUBICSPLINEGENERATOR_H
