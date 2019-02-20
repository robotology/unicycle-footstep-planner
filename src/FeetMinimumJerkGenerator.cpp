/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <FeetMinimumJerkGenerator.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/CubicSpline.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <cassert>
#include <mutex>

class FeetMinimumJerkGenerator::FeetMinimumJerkGeneratorImplementation {
public:

    double stepHeight = -1.0, pitchDelta = 0.0, swingApex = 0.5, landingVelocity = 0.0;

    iDynTree::VectorDynSize xPositionsBuffer, yPositionsBuffer, zPositionsBuffer, yawsBuffer, pitchAnglesBuffer, timesBuffer, zTimesBuffer;

    std::vector<iDynTree::Transform> leftTrajectory, rightTrajectory;
    std::vector<iDynTree::Twist> leftMixedTwist, rightMixedTwist;
    std::vector<iDynTree::SpatialAcc> leftMixedAcceleration, rightMixedAcceleration;

    std::mutex mutex;

    FeetMinimumJerkGeneratorImplementation()
        : xPositionsBuffer(2)
        , yPositionsBuffer(2)
        , zPositionsBuffer(3)
        , yawsBuffer(2)
        , pitchAnglesBuffer(3)
        , timesBuffer(2)
        , zTimesBuffer(3)
    { }

    bool minimumJerk(const double &initialPoint,
                     const double &finalPoint,
                     const double &t,
                     const double &trajectoryDuration,
                     double &positionOutput,
                     double &velocityOutput,
                     double &accelerationOutput)
    {
        double normalizedTime = t/trajectoryDuration;
        if((normalizedTime < 0) || (normalizedTime > 1)){
            std::cerr << "[FeetMinimumJerkGenerator::minimumJerk] The time has to be a positive number less than 1 "
                      << " Remember to normalize the time."
                      << std::endl;
            return false;
        }

        double p = 6 * std::pow(normalizedTime,5) - 15 * std::pow(normalizedTime,4) + 10 * std::pow(normalizedTime,3);
        double dp = (5 * 6 * std::pow(normalizedTime,4) - 4 * 15 * std::pow(normalizedTime,3) + 3 * 10 * std::pow(normalizedTime,2)) / trajectoryDuration;
        double ddp = (4 * 5 * 6 * std::pow(normalizedTime,3) - 3 * 4 * 15 * std::pow(normalizedTime,2) + 2 * 3 * 10 * normalizedTime) / std::pow(trajectoryDuration, 2);
        positionOutput = p * (finalPoint - initialPoint) + initialPoint;
        velocityOutput = dp * (finalPoint - initialPoint);
        accelerationOutput = ddp * (finalPoint - initialPoint);
        return true;
    }


    bool interpolateFoot(double dT, const std::vector<size_t>& phaseShift, const std::vector<StepPhase> &stepPhase,
                         const FootPrint &foot, std::vector<iDynTree::Transform> &output, std::vector<iDynTree::Twist> &outputTwistsInMixedRepresentation, std::vector<iDynTree::SpatialAcc> &outputAccelerationInMixedRepresentation) {
        //NOTE this must be called after createPhasesTimings

        if (stepHeight < 0){
            std::cerr << "[FeetMinimumJerkGenerator::computeNewTrajectories] First you have to set the step height." << std::endl;
            return false;
        }

        output.resize(stepPhase.size());
        outputTwistsInMixedRepresentation.resize(stepPhase.size());
        outputAccelerationInMixedRepresentation.resize(stepPhase.size());

        const StepList& steps = foot.getSteps();
        StepList::const_iterator footState = steps.begin();
        size_t instant = 0;
        iDynTree::Transform newTransform;
        iDynTree::Position newPosition;
        iDynTree::Vector3 linearVelocity;
        iDynTree::Vector3 linearAcceleration;
        double swingLength, pitchAngle, yawAngle;
        iDynTree::Vector3 rpyDerivative;
        iDynTree::Vector3 rpySecondDerivative;
        iDynTree::AngularMotionVector3 rightTrivializedAngVelocity;
        iDynTree::Vector3 rightTrivializedAngAcceleration;
        rpyDerivative.zero();
        rpySecondDerivative.zero();
        size_t endOfPhase;

        for (size_t phase = 1; phase < phaseShift.size(); ++phase){ //the first value is useless

            endOfPhase = phaseShift[phase];

            if (stepPhase[instant] == StepPhase::Swing){
                //create the interpolation points
                //increase footState
                //prepare splines
                //iterate for every instant asking the interpolator

                swingLength = (endOfPhase-instant) * dT;

                xPositionsBuffer(0) = footState->position(0);
                yPositionsBuffer(0) = footState->position(1);
                zPositionsBuffer(0) = 0.0;
                yawsBuffer(0) = footState->angle;
                pitchAnglesBuffer(0) = 0.0;
                timesBuffer(0) = 0.0;
                zTimesBuffer(0) = 0.0;

                zPositionsBuffer(1) = stepHeight;
                pitchAnglesBuffer(1) = pitchDelta;
                zTimesBuffer(1) = swingApex * swingLength;

                if (footState + 1 == steps.cend()){
                    std::cerr << "[FeetMinimumJerkGenerator::computeNewTrajectories] Something went wrong. stepPhase and foot don't seem to be coherent." << std::endl;
                    return false;
                }
                ++footState;

                xPositionsBuffer(1) = footState->position(0);
                yPositionsBuffer(1) = footState->position(1);
                pitchAnglesBuffer(2) = 0.0;

                yawsBuffer(1) = footState->angle;
                timesBuffer(1) = swingLength;
                zPositionsBuffer(2) = 0.0;
                zTimesBuffer(2) = swingLength;


                size_t startSwingInstant = instant;
                double interpolationTime;
                double interpolationTime0 = (instant - startSwingInstant)*dT;
                double halfDuration = swingLength * swingApex;


                while (instant < std::round((endOfPhase - startSwingInstant) * swingApex) + startSwingInstant){
                    interpolationTime = (instant - startSwingInstant)*dT;

                    if(!minimumJerk(xPositionsBuffer(0), xPositionsBuffer(1), (interpolationTime - interpolationTime0),
                                    swingLength, newPosition(0), linearVelocity(0), linearAcceleration(0))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }

                    if(!minimumJerk(yPositionsBuffer(0), yPositionsBuffer(1), (interpolationTime - interpolationTime0),
                                    swingLength, newPosition(1), linearVelocity(1), linearAcceleration(1))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }

                    if(!minimumJerk(zPositionsBuffer(0), zPositionsBuffer(1), (interpolationTime - interpolationTime0),
                                    halfDuration, newPosition(2), linearVelocity(2), linearAcceleration(2))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }

                    if(!minimumJerk(pitchAnglesBuffer(0), pitchAnglesBuffer(1), (interpolationTime - interpolationTime0),
                                    halfDuration, pitchAngle, rpyDerivative(1), rpySecondDerivative(1))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }

                    if(!minimumJerk(yawsBuffer(0), yawsBuffer(1), (interpolationTime - interpolationTime0),
                                    swingLength, yawAngle, rpyDerivative(2), rpySecondDerivative(2))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }
                    newTransform.setPosition(newPosition);
                    newTransform.setRotation(iDynTree::Rotation::RPY(0.0, pitchAngle, yawAngle));

                    if (newPosition(2) < 0){
                        std::cerr << "[FEETINTERPOLATOR] The z of the foot may be negative. Continuing anyway." <<std::endl;
                    }
                    output[instant] = newTransform;
                    iDynTree::toEigen(rightTrivializedAngVelocity) = iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivative(0.0, pitchAngle, yawAngle)) *
                            iDynTree::toEigen(rpyDerivative);
                    outputTwistsInMixedRepresentation[instant].setLinearVec3(linearVelocity);
                    outputTwistsInMixedRepresentation[instant].setAngularVec3(rightTrivializedAngVelocity);

                    iDynTree::toEigen(rightTrivializedAngAcceleration) =
                        iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivativeRateOfChange(0.0, pitchAngle, yawAngle,
                                                                                                        rpyDerivative(0), rpyDerivative(1), rpyDerivative(2))) *
                        iDynTree::toEigen(rpyDerivative) +
                        iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivative(0.0, pitchAngle, yawAngle)) *
                        iDynTree::toEigen(rpySecondDerivative);

                    iDynTree::toEigen(outputAccelerationInMixedRepresentation[instant].getLinearVec3()) = iDynTree::toEigen(linearAcceleration);
                    iDynTree::toEigen(outputAccelerationInMixedRepresentation[instant].getAngularVec3()) = iDynTree::toEigen(rightTrivializedAngAcceleration);

                    ++instant;
                }

                halfDuration = swingLength * (1 - swingApex);

                double interpolationTime1 = (instant - startSwingInstant)*dT;
                while (instant < endOfPhase){
                    interpolationTime = (instant - startSwingInstant)*dT;

                    if(!minimumJerk(xPositionsBuffer(0), xPositionsBuffer(1), (interpolationTime - interpolationTime0),
                                    swingLength, newPosition(0), linearVelocity(0), linearAcceleration(0))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }

                    if(!minimumJerk(yPositionsBuffer(0), yPositionsBuffer(1), (interpolationTime - interpolationTime0),
                                    swingLength, newPosition(1), linearVelocity(1), linearAcceleration(1))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }

                    if(!minimumJerk(zPositionsBuffer(1), zPositionsBuffer(2), (interpolationTime - interpolationTime1),
                                    halfDuration, newPosition(2), linearVelocity(2), linearAcceleration(2))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }


                    if(!minimumJerk(pitchAnglesBuffer(1), pitchAnglesBuffer(2), (interpolationTime - interpolationTime1),
                                    halfDuration, pitchAngle, rpyDerivative(1), rpySecondDerivative(1))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }

                    if(!minimumJerk(yawsBuffer(0), yawsBuffer(1), (interpolationTime - interpolationTime0),
                                    swingLength, yawAngle, rpyDerivative(2), rpySecondDerivative(2))){
                        std::cerr << "[FEETINTERPOLATOR] Unable to evaluate the trajectory";
                        return false;
                    }

                    newTransform.setPosition(newPosition);
                    newTransform.setRotation(iDynTree::Rotation::RPY(0.0, pitchAngle, yawAngle));

                    if (newPosition(2) < 0){
                        std::cerr << "[FEETINTERPOLATOR] The z of the foot may be negative. Continuing anyway." <<std::endl;
                    }

                    output[instant] = newTransform;
                    iDynTree::toEigen(rightTrivializedAngVelocity) = iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivative(0.0, pitchAngle, yawAngle)) *
                            iDynTree::toEigen(rpyDerivative);
                    outputTwistsInMixedRepresentation[instant].setLinearVec3(linearVelocity);
                    outputTwistsInMixedRepresentation[instant].setAngularVec3(rightTrivializedAngVelocity);

                    iDynTree::toEigen(rightTrivializedAngAcceleration) =
                        iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivativeRateOfChange(0.0, pitchAngle, yawAngle,
                                                                                                        rpyDerivative(0), rpyDerivative(1), rpyDerivative(2))) *
                            iDynTree::toEigen(rpyDerivative) +
                            iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivative(0.0, pitchAngle, yawAngle)) *
                            iDynTree::toEigen(rpySecondDerivative);

                    iDynTree::toEigen(outputAccelerationInMixedRepresentation[instant].getLinearVec3()) = iDynTree::toEigen(linearAcceleration);
                    iDynTree::toEigen(outputAccelerationInMixedRepresentation[instant].getAngularVec3()) = iDynTree::toEigen(rightTrivializedAngAcceleration);

                    ++instant;
                }

            } else { //keep foot position constant
                newPosition(0) = footState->position(0);
                newPosition(1) = footState->position(1);
                newPosition(2) = 0.0;                   //ground here is assumed to be at 0 level;
                newTransform.setPosition(newPosition);
                newTransform.setRotation(iDynTree::Rotation::RPY(0.0, 0.0, footState->angle));


                while (instant < endOfPhase){
                    output[instant] = newTransform;
                    outputTwistsInMixedRepresentation[instant].zero();
                    outputAccelerationInMixedRepresentation[instant].zero();
                    ++instant;
                }
            }
        }

        return true;
    }

};


FeetMinimumJerkGenerator::FeetMinimumJerkGenerator()
    : m_pimpl(std::make_unique<FeetMinimumJerkGeneratorImplementation>())
{ }

bool FeetMinimumJerkGenerator::computeNewTrajectories(double dT, const FootPrint &left, const FootPrint &right, const std::vector<StepPhase> &lFootPhases,
                                                      const std::vector<StepPhase> &rFootPhases, const std::vector<size_t> &phaseShift)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (!(m_pimpl->interpolateFoot(dT, phaseShift, lFootPhases, left, m_pimpl->leftTrajectory, m_pimpl->leftMixedTwist, m_pimpl->leftMixedAcceleration))){
        std::cerr << "[FeetMinimumJerkGenerator::computeNewTrajectories] Failed while interpolating left foot trajectory." << std::endl;
        return false;
    }

    if (!(m_pimpl->interpolateFoot(dT, phaseShift, rFootPhases, right, m_pimpl->rightTrajectory, m_pimpl->rightMixedTwist, m_pimpl->rightMixedAcceleration))){
        std::cerr << "[FeetMinimumJerkGenerator::computeNewTrajectories] Failed while interpolating left foot trajectory." << std::endl;
        return false;
    }
    return true;
}

FeetMinimumJerkGenerator::~FeetMinimumJerkGenerator()
{
}

bool FeetMinimumJerkGenerator::setStepHeight(double stepHeight)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (stepHeight < 0){
        std::cerr << "[FeetMinimumJerkGenerator::setStepHeight] The stepHeight is supposed to be positive." << std::endl;
        return false;
    }
    m_pimpl->stepHeight = stepHeight;
    return true;
}

bool FeetMinimumJerkGenerator::setPitchDelta(double pitchAngle)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->pitchDelta = iDynTree::deg2rad(pitchAngle);
    return true;
}

bool FeetMinimumJerkGenerator::setFootApexTime(double swingTimeRatio)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if ((swingTimeRatio <= 0)||(swingTimeRatio >= 1)){
        std::cerr << "[FeetMinimumJerkGenerator::setFootApexTime] The swingTimeRatio is supposed to be chosen in the interval (0, 1)." << std::endl;
        return false;
    }
    m_pimpl->swingApex = swingTimeRatio;
    return true;
}

bool FeetMinimumJerkGenerator::setFootLandingVelocity(double landingVelocity)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->landingVelocity = landingVelocity;
    return true;
}

void FeetMinimumJerkGenerator::getFeetTrajectories(std::vector<iDynTree::Transform> &lFootTrajectory, std::vector<iDynTree::Transform> &rFootTrajectory) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    lFootTrajectory = m_pimpl->leftTrajectory;
    rFootTrajectory = m_pimpl->rightTrajectory;
}

void FeetMinimumJerkGenerator::getFeetTwistsInMixedRepresentation(std::vector<iDynTree::Twist> &lFootTwistsInMixedRepresentation, std::vector<iDynTree::Twist> &rFootTwistsInMixedRepresentation) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    lFootTwistsInMixedRepresentation = m_pimpl->leftMixedTwist;
    rFootTwistsInMixedRepresentation = m_pimpl->rightMixedTwist;
}

void FeetMinimumJerkGenerator::getFeetAccelerationInMixedRepresentation(std::vector<iDynTree::SpatialAcc> &lFootAccelerationInMixedRepresentation, std::vector<iDynTree::SpatialAcc> &rFootAccelerationInMixedRepresentation) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    lFootAccelerationInMixedRepresentation = m_pimpl->leftMixedAcceleration;
    rFootAccelerationInMixedRepresentation = m_pimpl->rightMixedAcceleration;
}
