/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <ZMPTrajectoryGenerator.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/CubicSpline.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Transform.h>
#include <cassert>
#include <iostream>
#include <cmath>

class ZMPTrajectoryGenerator::ZMPTrajectoryGeneratorImplementation {
public:
    WeightInitialState initialState;
    std::vector<WeightInitialState> m_initStates;


    double nominalSwitchTime, maxSwitchTime, maxSwingTime, nominalSwingTime;
    iDynTree::Vector2 m_leftStanceZMP, m_leftSwitchZMP, m_rightStanceZMP, m_rightSwitchZMP;

    Step previousLeft, previousRight;

    std::vector<double> m_weightInLeft, m_weightInRight;
    std::vector<double> m_weightInLeftVelocity, m_weightInRightVelocity;
    std::vector<double> m_weightInLeftAcceleration, m_weightInRightAcceleration;
    std::vector<iDynTree::Vector2> m_leftZMP, m_rightZMP, m_worldZMP;
    std::vector<iDynTree::Vector2> m_leftZMPVelocity, m_rightZMPVelocity, m_worldZMPVelocity;
    std::vector<iDynTree::Vector2> m_leftZMPAcceleration, m_rightZMPAcceleration, m_worldZMPAcceleration;

    bool initialWeightSpecified = false, previousStepsSpecified = false;

private:
    iDynTree::VectorDynSize m_buffer, m_timesBuffer;
    iDynTree::CubicSpline m_spline;


public:

    bool computeFootWeightPortion(double dT, bool pauseActive, const std::vector<StepPhase> &stepPhase, const std::vector<size_t> &mergePoints,
                                  const std::vector<size_t>& phaseShift, std::vector<double> &output,
                                  std::vector<double> &outputVelocity, std::vector<double> &outputAcceleration) {
        //NOTE this must be called after createPhasesTimings

        if (output.size() != stepPhase.size())
            output.resize(stepPhase.size());
        if (outputVelocity.size() != stepPhase.size())
            outputVelocity.resize(stepPhase.size());
        if (outputAcceleration.size() != stepPhase.size())
            outputAcceleration.resize(stepPhase.size());

        if (m_initStates.size() != mergePoints.size())
            m_initStates.resize(mergePoints.size());

        size_t instant = 0, initialSwitchInstant, endOfPhase;
        double switchLength, switchInstant;
        size_t mergePoint = 0;

        for (size_t phase = 1; phase < phaseShift.size(); ++phase){ //the first value is useless

            endOfPhase = phaseShift[phase];

            if ((stepPhase[instant] == StepPhase::Stance)||(stepPhase[instant] == StepPhase::Swing)){
                while (instant < endOfPhase){
                    output[instant] = (stepPhase[instant] == StepPhase::Stance) ? 1.0 : 0.0;
                    outputVelocity[instant] = 0.0;
                    outputAcceleration[instant] = 0.0;

                    if (instant == mergePoints[mergePoint]){
                        m_initStates[mergePoint].initialPosition = output[instant];
                        m_initStates[mergePoint].initialVelocity = outputVelocity[instant];
                        m_initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                        if (mergePoint < (mergePoints.size() - 1))
                            mergePoint++;
                    }

                    ++instant;
                }
            } else if ((stepPhase[instant] == StepPhase::SwitchIn)||(stepPhase[instant] == StepPhase::SwitchOut)){

                switchLength = (endOfPhase-instant) * dT;
                bool pause = pauseActive && (switchLength > maxSwitchTime); //if true, it will pause in the middle

                if (phase == 1){ //first half switch
                    m_buffer(0) = initialState.initialPosition;
                    m_spline.setInitialConditions(initialState.initialVelocity, initialState.initialAcceleration);
                } else {
                    m_buffer(0) = (stepPhase[instant] == StepPhase::SwitchIn) ? 0.0 : 1.0;
                    m_spline.setInitialConditions(0.0, 0.0);
                }
                m_timesBuffer(0) = 0.0;

                if (pause){
                    m_buffer(1) = 0.5; //pause in the middle
                    m_spline.setFinalConditions(0.0, 0.0);
                    m_timesBuffer(1) = nominalSwitchTime/2;

                    if (!m_spline.setData(m_timesBuffer, m_buffer)){
                        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the spline for the weight portion during pause." << std::endl;
                        return false;
                    }
                    initialSwitchInstant = instant;
                    while (instant < (initialSwitchInstant + std::round(nominalSwitchTime/(2.0 * dT)))){
                        switchInstant = (instant - initialSwitchInstant)*dT;
                        output[instant] = m_spline.evaluatePoint(switchInstant, outputVelocity[instant], outputAcceleration[instant]);

                        if (instant == mergePoints[mergePoint]){
                            m_initStates[mergePoint].initialPosition = output[instant];
                            m_initStates[mergePoint].initialVelocity = outputVelocity[instant];
                            m_initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                            if (mergePoint < (mergePoints.size() - 1))
                                mergePoint++;
                        }

                        ++instant;
                    }

                    while (instant < (endOfPhase - std::round(nominalSwitchTime/(2.0 * dT)))){
                        output[instant] = 0.5;
                        outputVelocity[instant] = 0.0;
                        outputAcceleration[instant] = 0.0;

                        if (instant == mergePoints[mergePoint]){
                            m_initStates[mergePoint].initialPosition = output[instant];
                            m_initStates[mergePoint].initialVelocity = outputVelocity[instant];
                            m_initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                            if (mergePoint < (mergePoints.size() - 1))
                                mergePoint++;
                        }

                        ++instant;
                    }
                    //pause is finished
                    m_buffer(0) = 0.5;
                    m_spline.setInitialConditions(0.0, 0.0);
                    m_timesBuffer(0) = 0.0;
                }

                if (phase == (phaseShift.size() - 1)){ //last half step
                    m_buffer(1) = 0.5;
                } else m_buffer(1) = (stepPhase[instant] == StepPhase::SwitchIn) ? 1.0 : 0.0;

                switchLength = (endOfPhase-instant) * dT;
                m_timesBuffer(1) = switchLength;

                m_spline.setFinalConditions(0.0, 0.0);
                if (!m_spline.setData(m_timesBuffer, m_buffer)){
                    std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the spline for the weight portion." << std::endl;
                    return false;
                }

                initialSwitchInstant = instant;
                while (instant < endOfPhase){
                    switchInstant = (instant - initialSwitchInstant)*dT;
                    output[instant] = m_spline.evaluatePoint(switchInstant, outputVelocity[instant], outputAcceleration[instant]);

                    if (instant == mergePoints[mergePoint]){
                        m_initStates[mergePoint].initialPosition = output[instant];
                        m_initStates[mergePoint].initialVelocity = outputVelocity[instant];
                        m_initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                        if (mergePoint < (mergePoints.size() - 1))
                            mergePoint++;
                    }

                    ++instant;
                }

            } else {
                std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Unrecognized step phase." <<std::endl;
                return false;
            }
        }
        return true;
    }


    void mirrorWeightPortion(const std::vector<double> &original, const std::vector<double> &originalVelocity,
                             const std::vector<double> &originalAcceleration,std::vector<double> &mirrored,
                             std::vector<double> &mirroredVelocity, std::vector<double> &mirroredAcceleration) {
        if (mirrored.size() != original.size())
            mirrored.resize(original.size());
        if (mirroredVelocity.size() != originalVelocity.size())
            mirroredVelocity.resize(originalVelocity.size());
        if (mirroredAcceleration.size() != originalAcceleration.size())
            mirroredAcceleration.resize(originalAcceleration.size());

        for (size_t instant = 0; instant < original.size(); ++instant){
            mirrored[instant] = 1.0 - original[instant];
        }

        for (size_t instant = 0; instant < originalVelocity.size(); ++instant){
            mirroredVelocity[instant] = -originalVelocity[instant];
        }

        for (size_t instant = 0; instant < originalAcceleration.size(); ++instant){
            mirroredAcceleration[instant] = -originalAcceleration[instant];
        }
    }


    bool computeLocalZMP(double dT, bool pauseActive,
                         const std::vector<StepPhase> &stepPhase,
                         const std::vector<size_t>& phaseShift,
                         const iDynTree::Vector2 &stanceZmpPosition,
                         const iDynTree::Vector2 &switchZmpInitPosition,
                         std::vector<iDynTree::Vector2> &output,
                         std::vector<iDynTree::Vector2> &outputVelocity,
                         std::vector<iDynTree::Vector2> &outputAcceleration) {
        //NOTE this must be called after createPhasesTimings

        if (output.size() != stepPhase.size())
            output.resize(stepPhase.size());
        if (outputVelocity.size() != stepPhase.size())
            outputVelocity.resize(stepPhase.size());
        if (outputAcceleration.size() != stepPhase.size())
            outputAcceleration.resize(stepPhase.size());

        static iDynTree::VectorDynSize xBuffer(2), yBuffer(2), timeBuffer(2);
        static iDynTree::CubicSpline xSpline(2), ySpline(2);

        size_t instant = 0, initialInstant, endOfPhase;
        double stanceLength, switchLength, elapsedTime;

        for (size_t phase = 1; phase < phaseShift.size(); ++phase){

            endOfPhase = phaseShift[phase];

            if (stepPhase[instant] == StepPhase::Swing){ //in this phase this local ZMP should not be taken into consideration. This is done by setting to zero its portion of weight
                while (instant < endOfPhase){
                    output[instant] = stanceZmpPosition;
                    outputVelocity[instant].zero();
                    outputAcceleration[instant].zero();
                    instant++;
                }
            }else if (stepPhase[instant] == StepPhase::SwitchIn){
                while (instant < endOfPhase){
                    output[instant] = stanceZmpPosition;
                    outputVelocity[instant].zero();
                    outputAcceleration[instant].zero();
                    ++instant;
                }
            } else if (stepPhase[instant] == StepPhase::Stance){

                xBuffer(0) = stanceZmpPosition(0);
                yBuffer(0) = stanceZmpPosition(1);
                timeBuffer(0) = 0.0;
                xSpline.setInitialConditions(0.0, 0.0);
                ySpline.setInitialConditions(0.0, 0.0);

                stanceLength = (endOfPhase-instant) * dT;
                xBuffer(1) = switchZmpInitPosition(0);
                yBuffer(1) = switchZmpInitPosition(1);
                timeBuffer(1) = stanceLength;
                xSpline.setFinalConditions(0.0, 0.0);
                ySpline.setFinalConditions(0.0, 0.0);

                if (!xSpline.setData(timeBuffer, xBuffer)){
                    std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the ZMPx spline in stance phase." << std::endl;
                    return false;
                }
                if (!ySpline.setData(timeBuffer, yBuffer)){
                    std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the ZMPy spline in stance phase." << std::endl;
                    return false;
                }

                initialInstant = instant;
                while (instant < endOfPhase){
                    elapsedTime = (instant - initialInstant)*dT;
                    output[instant](0) = xSpline.evaluatePoint(elapsedTime, outputVelocity[instant](0), outputAcceleration[instant](0));
                    output[instant](1) = ySpline.evaluatePoint(elapsedTime, outputVelocity[instant](1), outputAcceleration[instant](1));
                    ++instant;
                }
            } else if (stepPhase[instant] == StepPhase::SwitchOut){

                if (phase == 1){ //initial half switch
                    while (instant < endOfPhase){
                        output[instant] = stanceZmpPosition;
                        outputVelocity[instant].zero();
                        outputAcceleration[instant].zero();
                        instant++;
                    }
                } else {
                    switchLength = (endOfPhase-instant) * dT;
                    bool pause = pauseActive && (switchLength > maxSwitchTime); //if true, it will pause in the middle

                    xBuffer(0) = switchZmpInitPosition(0);
                    yBuffer(0) = switchZmpInitPosition(1);
                    timeBuffer(0) = 0.0;
                    xSpline.setInitialConditions(0.0, 0.0);
                    ySpline.setInitialConditions(0.0, 0.0);

                    xBuffer(1) = stanceZmpPosition(0); //bring the ZMP back to the stance position
                    yBuffer(1) = stanceZmpPosition(1);
                    if (pause){
                        timeBuffer(1) = nominalSwitchTime/2;
                    } else if (phase == (phaseShift.size() - 1)){
                        timeBuffer(1) = (endOfPhase - instant)*dT;
                    } else {
                        timeBuffer(1) = switchLength/2;
                    }

                    xSpline.setFinalConditions(0.0, 0.0);
                    ySpline.setFinalConditions(0.0, 0.0);

                    if (!xSpline.setData(timeBuffer, xBuffer)){
                        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the ZMPx spline in switch phase." << std::endl;
                        return false;
                    }
                    if (!ySpline.setData(timeBuffer, yBuffer)){
                        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the ZMPy spline in switch phase." << std::endl;
                        return false;
                    }
                    initialInstant = instant;
                    while (instant < (initialInstant + std::ceil(timeBuffer(1)/dT))){
                        elapsedTime = (instant - initialInstant)*dT;
                        output[instant](0) = xSpline.evaluatePoint(elapsedTime, outputVelocity[instant](0), outputAcceleration[instant](0));
                        output[instant](1) = ySpline.evaluatePoint(elapsedTime, outputVelocity[instant](1), outputAcceleration[instant](1));
                        ++instant;
                    }

                    while (instant < endOfPhase) {
                        output[instant](0) = xBuffer(1);
                        output[instant](1) = yBuffer(1);
                        outputVelocity[instant].zero();
                        outputAcceleration[instant].zero();
                        ++instant;
                    }
                }

            } else {
                std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Unrecognized step phase." <<std::endl;
                return false;
            }
        }
        return true;
    }

    inline iDynTree::Position pos3D(const iDynTree::Vector2 &xy) {
        return iDynTree::Position(xy(0), xy(1), 0.0);
    }

    inline iDynTree::Position pos3D(const iDynTree::Transform &H, const iDynTree::Vector2 &xy) {
        return H * pos3D(xy);
    }

//    void computeGlobalZMP(double dT, const std::vector<size_t>& phaseShift, const std::vector<const Step*>& orderedSteps) {
//        //NOTE This must be called after that both the local ZMPs, the weight portions and the feet are computed
//        iDynTree::Position leftWorldZMP, rightWorldZMP;
//        iDynTree::Position leftWorldZMPVelocity, rightWorldZMPVelocity;
//        iDynTree::Position leftWorldZMPAcceleration, rightWorldZMPAcceleration;

//        m_worldZMP.resize(m_leftZMP.size());
//        m_worldZMPVelocity.resize(m_leftZMPVelocity.size());
//        m_worldZMPAcceleration.resize(m_leftZMPAcceleration.size());

//        iDynTree::CubicSpline correctionSpline(2);
//        static iDynTree::VectorDynSize correctionBuffer(2), timeBuffer(2);

//        correctionBuffer(0) = 1.0;
//        timeBuffer(0) = 0.0;
//        correctionBuffer(1) = 0.0;
//        timeBuffer(1) = phaseShift[1] * dT;
//        correctionSpline.setInitialConditions(0.0, 0.0);
//        correctionSpline.setFinalConditions(0.0, 0.0);
//        correctionSpline.setData(timeBuffer, correctionBuffer);

//        iDynTree::Transform oldLeftH, oldRightH;
//        oldLeftH.setPosition(pos3D(previousLeft.position));
//        oldLeftH.setRotation(iDynTree::Rotation::RotZ(previousLeft.angle));
//        oldRightH.setPosition(pos3D(previousRight.position));
//        oldRightH.setRotation(iDynTree::Rotation::RotZ(previousRight.angle));
//        iDynTree::Position deltaL, deltaR;


//        double correction = 0.0, correctionVelocity = 0.0, correctionAcceleration = 0.0;

//        for (size_t instant = 0; instant < m_leftZMP.size(); ++instant){

//            leftWorldZMP = pos3D(m_leftTrajectory[instant], m_leftZMP[instant]);
//            rightWorldZMP = pos3D(m_rightTrajectory[instant], m_rightZMP[instant]);

//            if (instant < phaseShift[1]){
//                correction = correctionSpline.evaluatePoint(instant*m_dT, correctionVelocity, correctionAcceleration);
//            } else {
//                correction = 0.0;
//                correctionVelocity = 0.0;
//                correctionAcceleration = 0.0;
//            }

//            if (instant < phaseShift[1]){ //first half switch
//                deltaL = pos3D(oldLeftH, m_leftZMP[instant]) - leftWorldZMP;
//                deltaR = pos3D(oldRightH, m_rightZMP[instant]) - rightWorldZMP;
//                for (unsigned int i = 0; i < 2; ++i){
//                    m_worldZMP[instant](i) = m_weightInLeft[instant] * (leftWorldZMP(i) + correction * deltaL(i)) +
//                            m_weightInRight[instant] * (rightWorldZMP(i) + correction * deltaR(i));
//                }
//            } else {
//                for (unsigned int i = 0; i < 2; ++i){
//                    m_worldZMP[instant](i) = m_weightInLeft[instant] * leftWorldZMP(i) + m_weightInRight[instant] * rightWorldZMP(i);
//                }
//            }


//            leftWorldZMPVelocity = pos3D(m_leftTrajectory[instant], m_leftZMPVelocity[instant]);
//            rightWorldZMPVelocity = pos3D(m_rightTrajectory[instant], m_rightZMPVelocity[instant]);

//            if (instant < phaseShift[1]){
//                for (unsigned int i = 0; i < 2; ++i){ //NOTE!! HERE WE ARE ASSUMING THAT NEITHER THE FEET, NOR THE LOCAL ZMPs ARE MOVING (only for the first phase)
//                    m_worldZMPVelocity[instant](i) = m_weightInLeftVelocity[instant] * (leftWorldZMP(i) + correction * deltaL(i)) +
//                            m_weightInLeft[instant] * correctionVelocity * deltaL(i) +
//                            m_weightInRightVelocity[instant] * (rightWorldZMP(i) + correction * deltaR(i)) +
//                            m_weightInRight[instant] * correctionVelocity * deltaR(i);
//                }
//            } else {
//                for (unsigned int i = 0; i < 2; ++i){
//                    m_worldZMPVelocity[instant](i) = m_weightInLeftVelocity[instant] * leftWorldZMP(i) +
//                            m_weightInLeft[instant] * leftWorldZMPVelocity(i) +
//                            m_weightInRightVelocity[instant] * rightWorldZMP(i) +
//                            m_weightInRight[instant] * rightWorldZMPVelocity(i);
//                }
//            }

//            leftWorldZMPAcceleration = pos3D(m_leftTrajectory[instant], m_leftZMPAcceleration[instant]);
//            rightWorldZMPAcceleration = pos3D(m_rightTrajectory[instant], m_rightZMPAcceleration[instant]);

//            if (instant < phaseShift[1]){
//                for (unsigned int i = 0; i < 2; ++i){ //NOTE!! HERE WE ARE ASSUMING THAT NEITHER THE FEET, NOR THE LOCAL ZMPs ARE MOVING (only for the first phase)
//                    m_worldZMPAcceleration[instant](i) = m_weightInLeftAcceleration[instant] * (leftWorldZMP(i) + correction * deltaL(i)) +
//                            2 * m_weightInLeftVelocity[instant] * correctionVelocity * deltaL(i) +
//                            m_weightInLeft[instant] * correctionAcceleration * deltaL(i) +
//                            m_weightInRightAcceleration[instant] * (rightWorldZMP(i) + correction * deltaR(i)) +
//                            2 * m_weightInRightVelocity[instant] * correctionVelocity * deltaR(i) +
//                            m_weightInRight[instant] * correctionAcceleration * deltaR(i);
//                }
//            } else {
//                for (unsigned int i = 0; i < 2; ++i){
//                    m_worldZMPAcceleration[instant](i) = m_weightInLeftAcceleration[instant] * leftWorldZMP(i) +
//                            2 * m_weightInLeftVelocity[instant] * leftWorldZMPVelocity(i) +
//                            m_weightInLeft[instant] * leftWorldZMPAcceleration(i) +
//                            m_weightInRightAcceleration[instant] * rightWorldZMP(i) +
//                            2 * m_weightInRightVelocity[instant] * rightWorldZMPVelocity(i) +
//                            m_weightInRight[instant] * rightWorldZMPAcceleration(i);
//                }
//            }
//        }
//    }
};




bool ZMPTrajectoryGenerator::computeNewTrajectories(double dT, double switchPercentage, double maxStepTime,
                                                    double nominalStepTime, bool pauseActive, const std::vector<size_t> &mergePoints,
                                                    const FootPrint &left, const FootPrint &right, const std::vector<const Step *> &orderedSteps,
                                                    const std::vector<StepPhase> &lFootPhases, const std::vector<StepPhase> &rFootPhases, const std::vector<size_t> &phaseShift)
{
    m_pimpl->nominalSwitchTime = switchPercentage * nominalStepTime;
    m_pimpl->maxSwitchTime = switchPercentage * maxStepTime;
    m_pimpl->maxSwingTime = maxStepTime - m_pimpl->maxSwitchTime;
    m_pimpl->nominalSwingTime = nominalStepTime - m_pimpl->nominalSwitchTime;

    if (!(m_pimpl->initialWeightSpecified)) {
        m_pimpl->initialState.initialPosition = 0.5;
        m_pimpl->initialState.initialVelocity = 0.0;
        m_pimpl->initialState.initialAcceleration = 0.0;
    }

    if (!(m_pimpl->previousStepsSpecified)) {
        m_pimpl->previousLeft = left.getSteps().front();
        m_pimpl->previousRight = right.getSteps().front();
    }

    if (!(m_pimpl->computeFootWeightPortion(dT, pauseActive, lFootPhases, mergePoints, phaseShift,
                                            m_pimpl->m_weightInLeft, m_pimpl->m_weightInLeftVelocity, m_pimpl->m_weightInLeftAcceleration))) {
        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed while computing the weight percentage on the left foot." << std::endl;
        return false;
    }

    m_pimpl->mirrorWeightPortion(m_pimpl->m_weightInLeft, m_pimpl->m_weightInLeftVelocity, m_pimpl->m_weightInLeftAcceleration,
                                 m_pimpl->m_weightInRight, m_pimpl->m_weightInRightVelocity, m_pimpl->m_weightInRightAcceleration);

    if (!(m_pimpl->computeLocalZMP(dT, pauseActive, lFootPhases, phaseShift, m_pimpl->m_leftStanceZMP, m_pimpl->m_leftSwitchZMP,
                                   m_pimpl->m_leftZMP, m_pimpl->m_leftZMPVelocity, m_pimpl->m_leftZMPAcceleration))) {
        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed while computing the left local ZMP." << std::endl;
        return false;
    }

    if (!(m_pimpl->computeLocalZMP(dT, pauseActive, rFootPhases, phaseShift, m_pimpl->m_rightStanceZMP, m_pimpl->m_rightSwitchZMP,
                                   m_pimpl->m_rightZMP, m_pimpl->m_rightZMPVelocity, m_pimpl->m_rightZMPAcceleration))) {
        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed while computing the right local ZMP." << std::endl;
        return false;
    }



    m_pimpl->initialWeightSpecified = false;
    m_pimpl->previousStepsSpecified = false;

    return true;
}

ZMPTrajectoryGenerator::ZMPTrajectoryGenerator()
    : m_pimpl(new ZMPTrajectoryGeneratorImplementation)
{
    assert(m_pimpl);
    m_pimpl->m_leftStanceZMP.zero();
    m_pimpl->m_leftSwitchZMP.zero();
    m_pimpl->m_rightStanceZMP.zero();
    m_pimpl->m_rightSwitchZMP.zero();
}

ZMPTrajectoryGenerator::~ZMPTrajectoryGenerator()
{
    if (m_pimpl) {
         delete m_pimpl;
        m_pimpl = nullptr;
    }
}

bool ZMPTrajectoryGenerator::setWeightInitialState(const WeightInitialState &initialState)
{
    m_pimpl->initialState = initialState;
    m_pimpl->initialWeightSpecified = true;
    return true;
}

bool ZMPTrajectoryGenerator::setPreviousSteps(const Step &previousLeft, const Step &previousRight)
{
    m_pimpl->previousLeft = previousLeft;
    m_pimpl->previousRight = previousRight;

    m_pimpl->previousStepsSpecified = true;

    return true;
}

bool ZMPTrajectoryGenerator::setStanceZMPDelta(const iDynTree::Vector2 &offsetInLeftFootFrame, const iDynTree::Vector2 &offsetInRightFootFrame)
{
    m_pimpl->m_leftStanceZMP = offsetInLeftFootFrame;
    m_pimpl->m_rightStanceZMP = offsetInRightFootFrame;
    return true;
}

bool ZMPTrajectoryGenerator::setInitialSwitchZMPDelta(const iDynTree::Vector2 &offsetInLeftFootFrame, const iDynTree::Vector2 &offsetInRightFootFrame)
{
    m_pimpl->m_leftSwitchZMP = offsetInLeftFootFrame;
    m_pimpl->m_rightSwitchZMP = offsetInRightFootFrame;
    return true;
}
