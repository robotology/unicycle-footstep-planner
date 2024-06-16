/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the BSD-3-Clause license, see LICENSE
 *
 */

#ifndef ZMPTRAJECTORYGENERATOR_H
#define ZMPTRAJECTORYGENERATOR_H

#include <StepPhase.h>
#include <FootPrint.h>
#include <vector>
#include <memory>

typedef struct{
    double initialPosition;
    double initialVelocity;
    double initialAcceleration;
} InitialState;

class ZMPTrajectoryGenerator {
    friend class UnicycleGenerator;

    class ZMPTrajectoryGeneratorImplementation;
    std::unique_ptr<ZMPTrajectoryGeneratorImplementation> m_pimpl;

    bool computeNewTrajectories(double initTime, double dT, double switchPercentage, double maxStepTime,
                                double nominalStepTime, bool pauseActive, const std::vector<size_t> &mergePoints,
                                const FootPrint &left, const FootPrint &right,
                                const std::vector<const Step*>& orderedSteps,
                                const std::vector<StepPhase>& lFootPhases,
                                const std::vector<StepPhase>& rFootPhases,
                                const std::vector<size_t>& phaseShift);

    ZMPTrajectoryGenerator();

public:

    ~ZMPTrajectoryGenerator();

    bool setWeightInitialState(const InitialState &initialState);

    bool setPreviousSteps(const Step &previousLeft, const Step &previousRight); //These are the steps used to compute a previous ZMP trajectory. This allows to preserve continuity across callings.

    bool setStanceZMPDelta(const iDynTree::Vector2& offsetInLeftFootFrame, const iDynTree::Vector2& offsetInRightFootFrame);

    bool setInitialSwitchZMPDelta(const iDynTree::Vector2& offsetInLeftFootFrame, const iDynTree::Vector2& offsetInRightFootFrame); //it is the position the ZMP should have when the switch to the other foot begins.

    // Getters

    void getWeightPercentage(std::vector<double>& weightInLeft, std::vector<double>& weightInRight) const;

    void getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInLeftFirstDerivative,
                             std::vector<double> &weightInLeftSecondDerivative,std::vector<double> &weightInRight,
                             std::vector<double> &weightInRightFirstDerivative, std::vector<double> &weightInRightSecondDerivative) const;

    void getZMPTrajectory(std::vector<iDynTree::Vector2> &ZMPTrajectory) const;

    void getZMPTrajectory(std::vector<iDynTree::Vector2> &ZMPTrajectory, std::vector<iDynTree::Vector2> &ZMPVelocity,
                          std::vector<iDynTree::Vector2> &ZMPAcceleration) const;

    void getLocalZMPTrajectories(std::vector<iDynTree::Vector2>& leftZMPTrajectory, std::vector<iDynTree::Vector2>& rightZMPTrajectory) const;

    void getLocalZMPTrajectories(std::vector<iDynTree::Vector2> &leftZMPTrajectory, std::vector<iDynTree::Vector2> &leftZMPVelocity,
                                 std::vector<iDynTree::Vector2> &leftZMPAcceleration, std::vector<iDynTree::Vector2>& rightZMPTrajectory,
                                 std::vector<iDynTree::Vector2> &rightZMPVelocity, std::vector<iDynTree::Vector2> &rightZMPAcceleration) const;

    void getInitialStatesAtMergePoints(std::vector<InitialState>& initialStates) const;

};


#endif // ZMPTRAJECTORYGENERATOR_H
