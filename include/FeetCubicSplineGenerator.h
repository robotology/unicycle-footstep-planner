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

#include <iDynTree/Core/Transform.h>

#include <vector>
#include <memory>

class FeetCubicSplineGenerator {
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

    ~FeetCubicSplineGenerator();

    bool setStepHeight(double stepHeight);

    bool setPitchDelta(double pitchAngle = 0.0); //DEGREES

    bool setFootApexTime(double swingTimeRatio = 0.5);

    bool setFootLandingVelocity(double landingVelocity = 0.0);

    void getFeetTrajectories(std::vector<iDynTree::Transform>& lFootTrajectory, std::vector<iDynTree::Transform>& rFootTrajectory) const;

};

#endif // FEETCUBICSPLINEGENERATOR_H
