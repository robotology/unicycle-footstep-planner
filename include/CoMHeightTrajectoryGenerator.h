/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the BSD-3-Clause license, see LICENSE
 *
 */

#ifndef COMHEIGHTTRAJECTORYGENERATOR_H
#define COMHEIGHTTRAJECTORYGENERATOR_H

#include <StepPhase.h>
#include <cstddef>
#include <vector>
#include <memory>

class CoMHeightTrajectoryGenerator {

    friend class UnicycleGenerator;

    class CoMHeightTrajectoryGeneratorImplementation;
    std::unique_ptr<CoMHeightTrajectoryGeneratorImplementation> m_pimpl;

    bool computeNewTrajectories(double dT, const std::vector<StepPhase> &leftPhases, const std::vector<size_t> &phaseShift);

    CoMHeightTrajectoryGenerator();

public:

    ~CoMHeightTrajectoryGenerator();

    bool setCoMHeightSettings(double comHeight, double comHeightStanceDelta); //they are the nominal comHeight and a delta which is summed up during stance phase


    void getCoMHeightTrajectory(std::vector<double>& CoMHeightTrajectory) const;

    void getCoMHeightVelocity(std::vector<double>& CoMHeightVelocity) const;

    void getCoMHeightAccelerationProfile(std::vector<double>& CoMHeightAccelerationProfile) const;

};

#endif // COMHEIGHTTRAJECTORYGENERATOR_H
