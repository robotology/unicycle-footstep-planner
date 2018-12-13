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
    std::vector<double> weightInLeft, weightInRight;


    DCMTrajectoryGeneratorImplementation()
        : initialStateSpecified(false)
    {
        offsetInLeftFoot.zero();
        offsetInRightFoot.zero();
    }

    bool computeFeetWeight(const std::vector<StepPhase> &lFootPhases, const std::vector<size_t> &phaseShift, const FootPrint &left, const FootPrint &right, const std::vector<iDynTree::Vector2>& zmpPosition)
    {
        weightInLeft.resize(zmpPosition.size());
        weightInRight.resize(zmpPosition.size());

        Eigen::Vector2d feetDistance;
        Eigen::Vector2d ZMPDistanceFromLeftFoot;

        iDynTree::Position leftFootZMPOffset;
        iDynTree::Position rightFootZMPOffset;

        size_t instant = 0;
        size_t endOfPhase;

        const StepList& leftSteps = left.getSteps();
        StepList::const_iterator leftState = leftSteps.begin();

        const StepList& rightSteps = right.getSteps();
        StepList::const_iterator rightState = rightSteps.begin();

        iDynTree::Vector2 leftFootPosition, rightFootPosition;
        double leftYawAngle, rightYawAngle;

        for (size_t phase = 1; phase < phaseShift.size(); ++phase){ //the first value is useless (it is simply 0)
            endOfPhase = phaseShift[phase];

            if (lFootPhases[instant] == StepPhase::Stance){
                while (instant < endOfPhase){
                    weightInLeft[instant] = 1.0;
                    weightInRight[instant] = 0.0;
                    instant++;
                }

                if (rightState + 1 == rightSteps.cend()){
                    std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] Something went wrong. The step phases are not coherent with the right foot." << std::endl; //It's not possible to have a swing phase as last phase.
                    return false;
                }

                ++rightState;
            } else if (lFootPhases[instant] == StepPhase::Swing){
                while (instant < endOfPhase){
                    weightInLeft[instant] = 1.0;
                    weightInRight[instant] = 0.0;
                    instant++;
                }

                if (leftState + 1 == leftSteps.cend()){
                    std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] Something went wrong. The step phases are not coherent with the left foot." << std::endl; //It's not possible to have a swing phase as last phase.
                    return false;
                }

                ++leftState;
            } else {

                leftYawAngle = leftState->angle;
                leftFootPosition(0) = leftState->position(0) + cos(leftYawAngle) * offsetInLeftFoot(0) - sin(leftYawAngle) * offsetInLeftFoot(1);
                leftFootPosition(1) = leftState->position(1) + sin(leftYawAngle) * offsetInLeftFoot(0) + cos(leftYawAngle) * offsetInLeftFoot(1);

                rightYawAngle = rightState->angle;
                rightFootPosition(0) = rightState->position(0) + cos(rightYawAngle) * offsetInRightFoot(0) - sin(rightYawAngle) * offsetInRightFoot(1);
                rightFootPosition(1) = rightState->position(1) + sin(rightYawAngle) * offsetInRightFoot(0) + cos(rightYawAngle) * offsetInRightFoot(1);

                while (instant < endOfPhase){
                    feetDistance = iDynTree::toEigen(rightFootPosition) - iDynTree::toEigen(leftFootPosition);

                    ZMPDistanceFromLeftFoot = iDynTree::toEigen(zmpPosition[instant]) - iDynTree::toEigen(leftFootPosition);

                    weightInLeft[instant] = ZMPDistanceFromLeftFoot.norm() / feetDistance.norm();
                    weightInRight[instant] = 1 - ZMPDistanceFromLeftFoot.norm() / feetDistance.norm();
                    ++instant;
                }

            }

        }
        return true;
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

    const StepList& leftSteps = left.getSteps();
    const StepList& rightSteps = right.getSteps();


    if(orderedSteps.size() != 0){
        Step firstStanceFoot, firstSwingFoot;
//         during the first step both the left and the right feet impact time are equal to 0 therefore the next footsteps are used to
//         evaluate witch foot is the first stance foot
        if (leftSteps.front().impactTime == 0 && rightSteps.front().impactTime == 0){
            firstStanceFoot = (leftSteps[1].impactTime > rightSteps[1].impactTime) ? leftSteps[0] : rightSteps[0];
            firstSwingFoot = (leftSteps[1].impactTime > rightSteps[1].impactTime) ? rightSteps[0] : leftSteps[0];
        } else {
            firstStanceFoot = (leftSteps.front().impactTime > rightSteps.front().impactTime) ? leftSteps[0] : rightSteps[0];
            firstSwingFoot = (leftSteps.front().impactTime > rightSteps.front().impactTime) ? rightSteps[0] : leftSteps[0];
        }
        if (!m_pimpl->helper.generateDCMTrajectory(orderedSteps, firstStanceFoot, firstSwingFoot, initDCMPosition, initDCMVelocity, phaseShift)){
            std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] Failed while computing the DCM trajectories." << std::endl;
            return false;
        }
    } else {
        // in this case both the feet do not move. So the final DCM position is given by the position average of the left and right feet positions
        iDynTree::Vector2 finalDCMPosition;
        iDynTree::Vector2 leftFootPosition, rightFootPosition;

        double leftYawAngle = leftSteps.front().angle;
        leftFootPosition(0) = leftSteps.front().position(0) + cos(leftYawAngle) * m_pimpl->offsetInLeftFoot(0) - sin(leftYawAngle) * m_pimpl->offsetInLeftFoot(1);
        leftFootPosition(1) = leftSteps.front().position(1) + sin(leftYawAngle) * m_pimpl->offsetInLeftFoot(0) + cos(leftYawAngle) * m_pimpl->offsetInLeftFoot(1);

        double rightYawAngle = rightSteps.front().angle;
        rightFootPosition(0) = rightSteps.front().position(0) + cos(rightYawAngle) * m_pimpl->offsetInRightFoot(0) - sin(rightYawAngle) * m_pimpl->offsetInRightFoot(1);
        rightFootPosition(1) = rightSteps.front().position(1) + sin(rightYawAngle) * m_pimpl->offsetInRightFoot(0) + cos(rightYawAngle) * m_pimpl->offsetInRightFoot(1);

        iDynTree::toEigen(finalDCMPosition) = (iDynTree::toEigen(leftFootPosition) + iDynTree::toEigen(rightFootPosition)) / 2;

        if(!m_pimpl->helper.generateFixStanceDCMTrajectory(initDCMPosition, initDCMVelocity, finalDCMPosition, phaseShift)){
            std::cerr << "[DCMTrajectoryGenerator::interpolateDCM] Failed while computing the Stance DCM trajectories." << std::endl;
            return false;
        }
    }

    // compute the weight on each foot
    if(!(m_pimpl->computeFeetWeight(lFootPhases, phaseShift, left, right, m_pimpl->helper.getZMPPosition()))){
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

void DCMTrajectoryGenerator::getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInRight) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    weightInLeft = m_pimpl->weightInLeft;
    weightInRight = m_pimpl->weightInRight;

}
