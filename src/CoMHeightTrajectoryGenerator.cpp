/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the BSD-3-Clause license, see LICENSE
 *
 */

#include <cassert>
#include <iostream>
#include <mutex>

#include <CoMHeightTrajectoryGenerator.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/CubicSpline.h>


class CoMHeightTrajectoryGenerator::CoMHeightTrajectoryGeneratorImplementation {
public:

    std::vector<double> CoMHeightTrajectory, CoMHeightVelocity, CoMHeightAcceleration;
    double CoMHeight = -1.0, CoMHeightDelta = 0.0;
    iDynTree::VectorDynSize hBuffer, timesBuffer;
    iDynTree::CubicSpline heightSpline;

    std::mutex mutex;

    CoMHeightTrajectoryGeneratorImplementation()
        : hBuffer(3)
        , timesBuffer(3)
        , heightSpline(3)
    { }
};

bool CoMHeightTrajectoryGenerator::computeNewTrajectories(double dT, const std::vector<StepPhase>& leftPhases, const std::vector<size_t>& phaseShift)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    //NOTE this must be called after createPhasesTimings
    if (m_pimpl->CoMHeight < 0){
        std::cerr << "[CoMHeightTrajectoryGenerator::computeNewTrajectories] First you have to set the nominal CoM height." << std::endl;
        return false;
    }

    if (m_pimpl->CoMHeightTrajectory.size() != leftPhases.size())
        m_pimpl->CoMHeightTrajectory.resize(leftPhases.size());

    if (m_pimpl->CoMHeightVelocity.size() != leftPhases.size())
        m_pimpl->CoMHeightVelocity.resize(leftPhases.size());

    if (m_pimpl->CoMHeightAcceleration.size() != leftPhases.size())
        m_pimpl->CoMHeightAcceleration.resize(leftPhases.size());

    if (m_pimpl->CoMHeight < 0){
        std::cerr << "[CoMHeightTrajectoryGenerator::computeNewTrajectories] First you have to set the nominal CoM height." << std::endl;
        return false;
    }

    size_t endOfPhase, initialInstant;
    double interpolationTime, stanceLength;
    size_t instant = 0;
    for (size_t phase = 1; phase < phaseShift.size(); ++phase){ //the first value is useless
        endOfPhase = phaseShift[phase];

        if ((leftPhases[instant] == StepPhase::SwitchIn)||(leftPhases[instant] == StepPhase::SwitchOut)){
            while (instant < endOfPhase){
                m_pimpl->CoMHeightTrajectory[instant] = m_pimpl->CoMHeight;
                m_pimpl->CoMHeightVelocity[instant] = 0.0;
                m_pimpl->CoMHeightAcceleration[instant] = 0.0;
                instant++;
            }
        } else if ((leftPhases[instant] == StepPhase::Stance)||(leftPhases[instant] == StepPhase::Swing)){

            stanceLength = (endOfPhase - instant) * dT;
            m_pimpl->hBuffer(0) = m_pimpl->CoMHeight;
            m_pimpl->timesBuffer(0) = 0.0;
            m_pimpl->heightSpline.setInitialConditions(0.0, 0.0);

            m_pimpl->hBuffer(1) = m_pimpl->CoMHeight + m_pimpl->CoMHeightDelta;
            m_pimpl->timesBuffer(1) = stanceLength/2;

            m_pimpl->hBuffer(2) = m_pimpl->CoMHeight;
            m_pimpl->timesBuffer(2) = stanceLength;
            m_pimpl->heightSpline.setFinalConditions(0.0, 0.0);

            if (!m_pimpl->heightSpline.setData(m_pimpl->timesBuffer, m_pimpl->hBuffer)){
                std::cerr << "[CoMHeightTrajectoryGenerator::computeNewTrajectories] Failed to initialize the height spline." << std::endl;
                return false;
            }

            initialInstant = instant;
            while (instant < endOfPhase){
                interpolationTime = (instant - initialInstant) * dT;
                m_pimpl->CoMHeightTrajectory[instant] = m_pimpl->heightSpline.evaluatePoint(interpolationTime,
                                                                                            m_pimpl->CoMHeightVelocity[instant],
                                                                                            m_pimpl->CoMHeightAcceleration[instant]);
                instant++;
            }
        } else {
            std::cerr << "[CoMHeightTrajectoryGenerator::computeNewTrajectories] Unrecognized step phase." <<std::endl;
            return false;
        }
    }

    return true;
}

CoMHeightTrajectoryGenerator::CoMHeightTrajectoryGenerator()
    : m_pimpl(new CoMHeightTrajectoryGeneratorImplementation)
{
    assert(m_pimpl);
}

CoMHeightTrajectoryGenerator::~CoMHeightTrajectoryGenerator()
{
}

bool CoMHeightTrajectoryGenerator::setCoMHeightSettings(double comHeight, double comHeightStanceDelta)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (comHeight < 0){
        std::cerr << "[CoMHeightTrajectoryGenerator::setCoMHeightSettings] The comHeight is supposed to be positive." << std::endl;
        return false;
    }

    if ((comHeight + comHeightStanceDelta) < 0.0){
        std::cerr << "[CoMHeightTrajectoryGenerator::setCoMHeightSettings] The comHeightStanceDelta cannot be greater than the nominal comHeight." << std::endl;
        return false;
    }

    m_pimpl->CoMHeight = comHeight;
    m_pimpl->CoMHeightDelta = comHeightStanceDelta;

    return true;
}

void CoMHeightTrajectoryGenerator::getCoMHeightTrajectory(std::vector<double> &CoMHeightTrajectory) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    CoMHeightTrajectory = m_pimpl->CoMHeightTrajectory;
}

void CoMHeightTrajectoryGenerator::getCoMHeightVelocity(std::vector<double> &CoMHeightVelocity) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    CoMHeightVelocity = m_pimpl->CoMHeightVelocity;
}

void CoMHeightTrajectoryGenerator::getCoMHeightAccelerationProfile(std::vector<double> &CoMHeightAccelerationProfile) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    CoMHeightAccelerationProfile = m_pimpl->CoMHeightAcceleration;
}
