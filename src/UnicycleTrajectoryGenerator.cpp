/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicycleTrajectoryGenerator.h"

bool UnicycleTrajectoryGenerator::clearAndAddMeasuredStep(std::shared_ptr<FootPrint> foot, Step &previousStep, const iDynTree::Vector2 &measuredPosition, double measuredAngle)
{
    foot->clearSteps();

    Step correctedStep = previousStep;

    correctedStep.position = measuredPosition;
    iDynTree::Rotation initialRotation = iDynTree::Rotation::RotZ(previousStep.angle);
    iDynTree::Rotation measuredRotation = iDynTree::Rotation::RotZ(measuredAngle);
    correctedStep.angle = previousStep.angle + (initialRotation.inverse()*measuredRotation).asRPY()(2);

    return foot->addStep(correctedStep);
}

UnicycleTrajectoryGenerator::UnicycleTrajectoryGenerator()
    :m_left(std::make_shared<FootPrint>())
    ,m_right(std::make_shared<FootPrint>())
{
}

bool UnicycleTrajectoryGenerator::generateAndInterpolate(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double dT, const InitialState& weightInLeftAtMergePoint)
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

    correctedStep = correctLeft ? previousL : previousR;

    if (!clearAndAddMeasuredStep(toBeCorrected, correctedStep, measuredPosition, measuredAngle)){
        std::cerr << "Failed to update the steps using the measured value." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
            interpolate(*m_left, *m_right, initTime, dT, weightInLeftAtMergePoint, previousL, previousR);
}

bool UnicycleTrajectoryGenerator::reGenerate(double initTime, double dT, double endTime, const InitialState &weightInLeftAtMergePoint, const iDynTree::Vector2 &measuredLeftPosition, double measuredLeftAngle, const iDynTree::Vector2 &measuredRightPosition, double measuredRightAngle)
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

    if (!clearAndAddMeasuredStep(m_left, previousL, measuredLeftPosition, measuredLeftAngle)){
        std::cerr << "Failed to update the left steps using the measured value." << std::endl;
        return false;
    }

    if (!clearAndAddMeasuredStep(m_right, previousR, measuredRightPosition, measuredRightAngle)){
        std::cerr << "Failed to update the right steps using the measured value." << std::endl;
        return false;
    }

    return setEndTime(endTime) && computeNewSteps(m_left, m_right, initTime) &&
            interpolate(*m_left, *m_right, initTime, dT, weightInLeftAtMergePoint, previousL, previousR);
}
