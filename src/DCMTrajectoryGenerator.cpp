/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <DCMTrajectoryGenerator.h>
#include <DCMTrajectoryGeneratorHelper.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <cassert>
#include <iostream>
#include <cmath>
#include <mutex>

class DCMTrajectoryGenerator::DCMTrajectoryGeneratorImplementation {
public:

    DCMInitialState initialState;
    bool initialStateSpecified;
    std::mutex mutex;
    iDynTree::Vector2 offsetInLeftFoot, offsetInRightFoot;
    DCMTrajectoryGeneratorHelper helper;


    DCMTrajectoryGeneratorImplementation()
        : initialStateSpecified(false)
    {
        offsetInLeftFoot.zero();
        offsetInRightFoot.zero();
    }

};

DCMTrajectoryGenerator::DCMTrajectoryGenerator()
    : m_pimpl(std::make_unique<DCMTrajectoryGeneratorImplementation>())
{

}

bool DCMTrajectoryGenerator::computeNewTrajectories(double initTime, double dT, double switchPercentage, double maxStepTime,
                                                    double nominalStepTime, bool pauseActive, const std::vector<const Step *> &orderedSteps,
                                                    const std::vector<size_t> &phaseShift, const std::vector<StepPhase> &lFootPhases,
                                                    const FootPrint &left, const FootPrint &right)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (!(m_pimpl->initialStateSpecified)) {
        iDynTree::Vector2 leftFootPosition, rightFootPosition;

        // the desired position of the DCM can be shifted on x and y position
        // note that this method is called only for the first steps in this particular case the
        // rotation matrix between the world frame and booth feet frames is the identity
        iDynTree::toEigen(leftFootPosition) = iDynTree::toEigen(left.getSteps().front().position) + iDynTree::toEigen(m_pimpl->offsetInLeftFoot);
        iDynTree::toEigen(rightFootPosition) = iDynTree::toEigen(right.getSteps().front().position) + iDynTree::toEigen(m_pimpl->offsetInRightFoot);

        iDynTree::toEigen(m_pimpl->initialState.initialPosition) = (iDynTree::toEigen(leftFootPosition) + iDynTree::toEigen(rightFootPosition)) / 2;

        m_pimpl->initialState.initialVelocity.zero();
    }

    if (left.numberOfSteps() < 1){
        std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] No steps in the left pointer." << std::endl;
        return false;
    }

    if (right.numberOfSteps() < 1){
        std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] No steps in the right pointer." << std::endl;
        return false;
    }

    if (dT <= 0){
        std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] The dT is supposed to be positive." << std::endl;
        return false;
    }

    double startLeft = left.getSteps().front().impactTime;
    double startRight = right.getSteps().front().impactTime;

    if (initTime < std::max(startLeft, startRight)){
        std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] The initTime must be greater or equal than the maximum of the first impactTime of the two feet."
                  << std::endl;
        return false;
    }

    if (!m_pimpl->helper.setdT(dT)){
        std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] Failed while the DCM trajectory generator period is set." << std::endl;
        return false;
    }

    double maxSwitchTime = switchPercentage * maxStepTime;
    double nominalSwitchTime = switchPercentage * nominalStepTime;

    if (!m_pimpl->helper.setPauseConditions(pauseActive, maxSwitchTime, nominalSwitchTime)) {
        std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] Failed to set pause conditions." << std::endl;
        return false;
    }

    m_pimpl->helper.setZMPDelta(m_pimpl->offsetInLeftFoot, m_pimpl->offsetInRightFoot);


    // generate DCM trajectory
    iDynTree::Vector2 initDCMPosition, initDCMVelocity;

    initDCMPosition = m_pimpl->initialState.initialPosition;
    initDCMVelocity = m_pimpl->initialState.initialVelocity;

    if (!m_pimpl->helper.generateDCMTrajectory(orderedSteps, lFootPhases, left, right, initDCMPosition, initDCMVelocity, phaseShift)){
        std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] Failed while computing the DCM trajectories." << std::endl;
        return false;
    }

    m_pimpl->initialStateSpecified = false;

return true;

}

DCMTrajectoryGenerator::~DCMTrajectoryGenerator()
{ }

bool DCMTrajectoryGenerator::setDCMInitialState(const DCMInitialState &initialState)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->initialState = initialState;
    m_pimpl->initialStateSpecified = true;
    return true;
}

bool DCMTrajectoryGenerator::setOmega(const double &omega)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    return m_pimpl->helper.setOmega(omega);
}

bool DCMTrajectoryGenerator::setFootOriginOffset(const iDynTree::Vector2 &offsetInLeftFootFrame, const iDynTree::Vector2 &offsetInRightFootFrame)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->offsetInLeftFoot = offsetInLeftFootFrame;
    m_pimpl->offsetInRightFoot = offsetInRightFootFrame;
    return true;
}

const std::vector<iDynTree::Vector2> &DCMTrajectoryGenerator::getDCMPosition() const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);
    return m_pimpl->helper.getDCMPosition();
}

const std::vector<iDynTree::Vector2> &DCMTrajectoryGenerator::getDCMVelocity() const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);
    return m_pimpl->helper.getDCMVelocity();
}


const std::vector<iDynTree::Vector2> &DCMTrajectoryGenerator::getZMPPosition() const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);
    return m_pimpl->helper.getZMPPosition();
}

void DCMTrajectoryGenerator::getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInRight) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    return m_pimpl->helper.getWeightPercentage(weightInLeft, weightInRight);
}
