/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <FeetCubicSplineGenerator.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/CubicSpline.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <cassert>
#include <mutex>

class FeetCubicSplineGenerator::FeetCubicSplineGeneratorImplementation {
public:

    double stepHeight = -1.0, pitchDelta = 0.0, swingApex = 0.5, landingVelocity = 0.0;

    iDynTree::VectorDynSize xPositionsBuffer, yPositionsBuffer, zPositionsBuffer, yawsBuffer, pitchAnglesBuffer, timesBuffer, zTimesBuffer;
    iDynTree::CubicSpline xSpline, ySpline, zSpline, yawSpline, pitchSpline;

    std::vector<iDynTree::Transform> leftTrajectory, rightTrajectory;

    std::vector<iDynTree::Twist> leftMixedTwist, rightMixedTwist;

    std::vector<iDynTree::SpatialAcc> leftMixedAcceleration, rightMixedAcceleration;

    std::mutex mutex;

    FeetCubicSplineGeneratorImplementation()
        : xPositionsBuffer(2)
        , yPositionsBuffer(2)
        , zPositionsBuffer(3)
        , yawsBuffer(2)
        , pitchAnglesBuffer(3)
        , timesBuffer(2)
        , zTimesBuffer(3)
        , xSpline(2)
        , ySpline(2)
        , zSpline(3)
        , yawSpline(2)
        , pitchSpline(3)
    { }


    bool interpolateFoot(double dT, const std::vector<size_t>& phaseShift, const std::vector<StepPhase> &stepPhase, const FootPrint &foot, std::vector<iDynTree::Transform> &output, std::vector<iDynTree::Twist> &outputTwistsInMixedRepresentation, std::vector<iDynTree::SpatialAcc> &outputAccelerationInMixedRepresentation) {
        //NOTE this must be called after createPhasesTimings

        if (stepHeight < 0){
            std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] First you have to set the step height." << std::endl;
            return false;
        }

        if (output.size() != stepPhase.size())
            output.resize(stepPhase.size());

        outputTwistsInMixedRepresentation.resize(stepPhase.size());
        outputAccelerationInMixedRepresentation.resize(stepPhase.size());

        const StepList& steps = foot.getSteps();
        StepList::const_iterator footState = steps.begin();
        size_t instant = 0;
        iDynTree::Transform newTransform;
        iDynTree::Position newPosition;
        double dummyAcceleration;
        double swingLength, pitchAngle, yawAngle;
        size_t endOfPhase;
        iDynTree::Vector3 linearVelocity;
        iDynTree::Vector3 rpyDerivative;
        iDynTree::Vector3 linearAcceleration;
        iDynTree::Vector3 rpySecondDerivative;
        iDynTree::AngularMotionVector3 rightTrivializedAngVelocity;
        iDynTree::Vector3 rightTrivializedAngAcceleration;
        rpyDerivative.zero();
        rpySecondDerivative.zero();

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
                yawsBuffer(0) = footState->angle;
                pitchAnglesBuffer(0) = 0.0;
                timesBuffer(0) = 0.0;
                zTimesBuffer(0) = 0.0;

                zPositionsBuffer(1) = stepHeight;
                pitchAnglesBuffer(1) = pitchDelta;
                zTimesBuffer(1) = swingApex * swingLength;

                if (footState + 1 == steps.cend()){
                    std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] Something went wrong. stepPhase and foot don't seem to be coherent." << std::endl;
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

                //preparation for splines

                xSpline.setInitialConditions(0.0, 0.0);
                ySpline.setInitialConditions(0.0, 0.0);
                zSpline.setInitialConditions(0.0, 0.0);
                yawSpline.setInitialConditions(0.0, 0.0);
                pitchSpline.setInitialConditions(0.0, 0.0);

                xSpline.setFinalConditions(0.0, 0.0);
                ySpline.setFinalConditions(0.0, 0.0);
                zSpline.setFinalConditions(landingVelocity, 0.0); //we may think of non-null final acceleration for the z
                yawSpline.setFinalConditions(0.0, 0.0);
                pitchSpline.setInitialConditions(0.0, 0.0);

                if (!xSpline.setData(timesBuffer, xPositionsBuffer)){
                    std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] Failed to initialize the x-dimension spline." << std::endl;
                    return false;
                }
                if (!ySpline.setData(timesBuffer, yPositionsBuffer)){
                    std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] Failed to initialize the y-dimension spline." << std::endl;
                    return false;
                }
                if (!zSpline.setData(zTimesBuffer, zPositionsBuffer)){
                    std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] Failed to initialize the z-dimension spline." << std::endl;
                    return false;
                }
                if (!yawSpline.setData(timesBuffer, yawsBuffer)){
                    std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] Failed to initialize the yaw-dimension spline." << std::endl;
                    return false;
                }
                if (!pitchSpline.setData(zTimesBuffer, pitchAnglesBuffer)){
                    std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] Failed to initialize the yaw-dimension spline." << std::endl;
                    return false;
                }

                size_t startSwingInstant = instant;
                double interpolationTime;
                while (instant < endOfPhase){
                    interpolationTime = (instant - startSwingInstant)*dT;
                    newPosition(0) = xSpline.evaluatePoint(interpolationTime, linearVelocity(0), linearAcceleration(0));
                    newPosition(1) = ySpline.evaluatePoint(interpolationTime, linearVelocity(1), linearAcceleration(1));
                    newPosition(2) = zSpline.evaluatePoint(interpolationTime, linearVelocity(2), linearAcceleration(2));
                    pitchAngle = pitchSpline.evaluatePoint(interpolationTime, rpyDerivative(1), rpySecondDerivative(1));
                    yawAngle = yawSpline.evaluatePoint(interpolationTime, rpyDerivative(2), rpySecondDerivative(2));
                    newTransform.setPosition(newPosition);
                    newTransform.setRotation(iDynTree::Rotation::RPY(0.0, pitchAngle, yawAngle));

                    if (newPosition(2) < 0){
                        std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] The z of the foot goes negative. Continuing anyway." << std::endl;
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



FeetCubicSplineGenerator::FeetCubicSplineGenerator()
    : m_pimpl(new FeetCubicSplineGeneratorImplementation)
{
    assert(m_pimpl);
}

bool FeetCubicSplineGenerator::computeNewTrajectories(double dT, const FootPrint &left, const FootPrint &right, const std::vector<StepPhase> &lFootPhases,
                                                      const std::vector<StepPhase> &rFootPhases, const std::vector<size_t> &phaseShift)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (!(m_pimpl->interpolateFoot(dT, phaseShift, lFootPhases, left, m_pimpl->leftTrajectory, m_pimpl->leftMixedTwist, m_pimpl->leftMixedAcceleration))){
        std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] Failed while interpolating left foot trajectory." << std::endl;
        return false;
    }

    if (!(m_pimpl->interpolateFoot(dT, phaseShift, rFootPhases, right, m_pimpl->rightTrajectory, m_pimpl->rightMixedTwist, m_pimpl->rightMixedAcceleration))){
        std::cerr << "[FeetCubicSplineGenerator::computeNewTrajectories] Failed while interpolating left foot trajectory." << std::endl;
        return false;
    }
    return true;
}

FeetCubicSplineGenerator::~FeetCubicSplineGenerator()
{
}

bool FeetCubicSplineGenerator::setStepHeight(double stepHeight)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (stepHeight < 0){
        std::cerr << "[FeetCubicSplineGenerator::setStepHeight] The stepHeight is supposed to be positive." << std::endl;
        return false;
    }
    m_pimpl->stepHeight = stepHeight;
    return true;
}

bool FeetCubicSplineGenerator::setPitchDelta(double pitchAngle)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->pitchDelta = iDynTree::deg2rad(pitchAngle);
    return true;
}

bool FeetCubicSplineGenerator::setFootApexTime(double swingTimeRatio)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if ((swingTimeRatio <= 0)||(swingTimeRatio >= 1)){
        std::cerr << "[FeetCubicSplineGenerator::setFootApexTime] The swingTimeRatio is supposed to be chosen in the interval (0, 1)." << std::endl;
        return false;
    }
    m_pimpl->swingApex = swingTimeRatio;
    return true;
}

bool FeetCubicSplineGenerator::setFootLandingVelocity(double landingVelocity)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->landingVelocity = landingVelocity;
    return true;
}

void FeetCubicSplineGenerator::getFeetTrajectories(std::vector<iDynTree::Transform> &lFootTrajectory, std::vector<iDynTree::Transform> &rFootTrajectory) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    lFootTrajectory = m_pimpl->leftTrajectory;
    rFootTrajectory = m_pimpl->rightTrajectory;
}

void FeetCubicSplineGenerator::getFeetTwistsInMixedRepresentation(std::vector<iDynTree::Twist> &lFootTwistsInMixedRepresentation, std::vector<iDynTree::Twist> &rFootTwistsInMixedRepresentation) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    lFootTwistsInMixedRepresentation = m_pimpl->leftMixedTwist;
    rFootTwistsInMixedRepresentation = m_pimpl->rightMixedTwist;
}

void FeetCubicSplineGenerator::getFeetAccelerationInMixedRepresentation(std::vector<iDynTree::SpatialAcc> &lFootAccelerationInMixedRepresentation, std::vector<iDynTree::SpatialAcc> &rFootAccelerationInMixedRepresentation) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    lFootAccelerationInMixedRepresentation = m_pimpl->leftMixedAcceleration;
    rFootAccelerationInMixedRepresentation = m_pimpl->rightMixedAcceleration;
}
