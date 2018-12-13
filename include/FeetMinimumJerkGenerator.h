/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef FEETMINIMUMJERKGENERATOR_H
#define FEETMINIMUMJERKGENERATOR_H

#include <StepPhase.h>
#include <FootPrint.h>

#include <iDynTree/Core/Transform.h>

#include <vector>
#include <memory>

class FeetMinimumJerkGenerator {
    friend class UnicycleGenerator;

    class FeetMinimumJerkGeneratorImplementation;
    std::unique_ptr<FeetMinimumJerkGeneratorImplementation> m_pimpl;

    FeetMinimumJerkGenerator();

    bool computeNewTrajectories(double dT,
                                const FootPrint &left, const FootPrint &right,
                                const std::vector<StepPhase>& lFootPhases,
                                const std::vector<StepPhase>& rFootPhases,
                                const std::vector<size_t>& phaseShift);

public:

    ~FeetMinimumJerkGenerator();

    bool setStepHeight(double stepHeight);

    bool setPitchDelta(double pitchAngle = 0.0); //DEGREES

    bool setFootApexTime(double swingTimeRatio = 0.5);

    bool setFootLandingVelocity(double landingVelocity = 0.0);

    void getFeetTrajectories(std::vector<iDynTree::Transform>& lFootTrajectory, std::vector<iDynTree::Transform>& rFootTrajectory) const;

    void getFeetTwistsInMixedRepresentation(std::vector<iDynTree::Twist> &lFootTwistsInMixedRepresentation, std::vector<iDynTree::Twist> &rFootTwistsInMixedRepresentation) const;

    //In a future release, also accelerations will be provided

};

#endif // FEETMINIMUMJERKGENERATOR_H
