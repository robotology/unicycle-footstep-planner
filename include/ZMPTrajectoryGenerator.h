/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef ZMPTRAJECTORYGENERATOR_H
#define ZMPTRAJECTORYGENERATOR_H

#include <StepPhase.h>
#include <FootPrint.h>
#include <vector>

typedef struct{
    double initialPosition;
    double initialVelocity;
    double initialAcceleration;
} WeightInitialState;

class ZMPTrajectoryGenerator {
    friend class UnicycleGenerator;

    class ZMPTrajectoryGeneratorImplementation;
    ZMPTrajectoryGeneratorImplementation *m_pimpl;

    bool computeNewTrajectories(double dT, double switchPercentage, double maxStepTime,
                                double nominalStepTime, bool pauseActive, const std::vector<size_t> &mergePoints,
                                const FootPrint &left, const FootPrint &right,
                                const std::vector<const Step*>& orderedSteps,
                                const std::vector<StepPhase>& lFootPhases,
                                const std::vector<StepPhase>& rFootPhases,
                                const std::vector<size_t>& phaseShift);

    ZMPTrajectoryGenerator();

public:

    ~ZMPTrajectoryGenerator();

    bool setWeightInitialState(const WeightInitialState &initialState);

    bool setPreviousSteps(const Step &previousLeft, const Step &previousRight); //These are the steps used to compute a previous ZMP trajectory. This allows to preserve continuity across callings.

    bool setStanceZMPDelta(const iDynTree::Vector2& offsetInLeftFootFrame, const iDynTree::Vector2& offsetInRightFootFrame);

    bool setInitialSwitchZMPDelta(const iDynTree::Vector2& offsetInLeftFootFrame, const iDynTree::Vector2& offsetInRightFootFrame); //it is the position the ZMP should have when the switch to the other foot begins.

};


#endif // ZMPTRAJECTORYGENERATOR_H
