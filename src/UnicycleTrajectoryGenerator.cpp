/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicycleTrajectoryGenerator.h"

UnicycleTrajectoryGenerator::UnicycleTrajectoryGenerator()
    :m_left(std::make_shared<FootPrint>())
    ,m_right(std::make_shared<FootPrint>())
{
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT,
                                                         const InitialState& weightInLeftAtMergePoint)
{
    m_left = leftFoot;
    m_right = rightFoot;
    return computeNewSteps(leftFoot, rightFoot, initTime) && interpolate(*leftFoot, *rightFoot, initTime, dT, weightInLeftAtMergePoint);
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT)
{
    m_left = leftFoot;
    m_right = rightFoot;
    return computeNewSteps(leftFoot, rightFoot, initTime) && interpolate(*leftFoot, *rightFoot, initTime, dT);
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(double initTime, double dT, double endTime)
{
    m_left->clearSteps();
    m_right->clearSteps();
    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) && interpolate(*m_left, *m_right, initTime, dT);
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT, double endTime)
{
    m_left = leftFoot;
    m_right = rightFoot;
    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) && interpolate(*m_left, *m_right, initTime, dT);
}


bool UnicycleTrajectoryGenerator::reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint)
{
    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
        interpolate(*m_left, *m_right, initTime, dT, weightInLeftAtMergePoint);
}

bool UnicycleTrajectoryGenerator::reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint,
                                             const Step &measuredLeft, const Step &measuredRight)
{
    Step previousL, previousR;

    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    m_left->getLastStep(previousL);
    m_right->getLastStep(previousR);

    m_left->clearSteps();
    m_right->clearSteps();

    if (!m_left->addStep(measuredLeft)){
        std::cerr << "The measuredLeft step is invalid." << std::endl;
        return false;
    }

    if (!m_right->addStep(measuredRight)){
        std::cerr << "The measuredRight step is invalid." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
        interpolate(*m_left, *m_right, initTime, dT, weightInLeftAtMergePoint, previousL, previousR);
}

bool UnicycleTrajectoryGenerator::reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint,
                                             bool correctLeft, const iDynTree::Vector2 &measuredPosition, double measuredAngle)
{
    Step previousL, previousR, correctedStep;

    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    m_left->getLastStep(previousL);
    m_right->getLastStep(previousR);

    std::shared_ptr<FootPrint> toBeCorrected = correctLeft ? m_left : m_right;

    toBeCorrected->clearSteps();

    correctedStep = correctLeft ? previousL : previousR;

    correctedStep.position = measuredPosition;
    iDynTree::Rotation initialRotation = iDynTree::Rotation::RotZ(correctedStep.angle);
    iDynTree::Rotation measuredRotation = iDynTree::Rotation::RotZ(measuredAngle);
    correctedStep.angle = correctedStep.angle + (initialRotation.inverse()*measuredRotation).asRPY()(2);

    toBeCorrected->addStep(correctedStep);

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
        interpolate(*m_left, *m_right, initTime, dT, weightInLeftAtMergePoint, previousL, previousR);
}


// DCM functions

bool UnicycleTrajectoryGenerator::generateAndInterpolateDCM(double initTime, double dT, double endTime)
{
    m_left->clearSteps();
    m_right->clearSteps();
    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) && interpolateDCM(*m_left, *m_right, initTime, dT);
}

bool UnicycleTrajectoryGenerator::generateAndInterpolateDCM(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT, double endTime)
{
    m_left = leftFoot;
    m_right = rightFoot;
    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) && interpolateDCM(*m_left, *m_right, initTime, dT);
}


bool UnicycleTrajectoryGenerator::reGenerateDCM(double initTime, double dT, double endTime, const DCMInitialState &DCMBoundaryConditionAtMergePoint)
{
    if (!m_left->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    if (!m_right->keepOnlyPresentStep(initTime)){
        std::cerr << "The initTime is not compatible with previous runs. Call a method generateAndInterpolate instead." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
        interpolateDCM(*m_left, *m_right, initTime, dT, DCMBoundaryConditionAtMergePoint);
}
