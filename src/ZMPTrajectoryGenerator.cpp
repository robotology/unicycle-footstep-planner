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
#include <mutex>

class ZMPTrajectoryGenerator::ZMPTrajectoryGeneratorImplementation {
public:
    InitialState initialState;
    std::vector<InitialState> initStates;


    double nominalSwitchTime, maxSwitchTime, maxSwingTime, nominalSwingTime, endSwitchTime;
    iDynTree::Vector2 leftStanceZMP, leftSwitchZMP, rightStanceZMP, rightSwitchZMP;

    Step previousLeft, previousRight;

    std::vector<double> weightInLeft, weightInRight;
    std::vector<double> weightInLeftVelocity, weightInRightVelocity;
    std::vector<double> weightInLeftAcceleration, weightInRightAcceleration;
    std::vector<iDynTree::Vector2> leftZMP, rightZMP, worldZMP;
    std::vector<iDynTree::Vector2> leftZMPVelocity, rightZMPVelocity, worldZMPVelocity;
    std::vector<iDynTree::Vector2> leftZMPAcceleration, rightZMPAcceleration, worldZMPAcceleration;

    bool initialWeightSpecified = false, previousStepsSpecified = false;
    bool timingWarningPrinted = false;

    std::mutex mutex;

private:
    iDynTree::VectorDynSize m_buffer, m_timesBuffer, m_correctionBuffer, m_xBuffer, m_yBuffer;
    iDynTree::CubicSpline m_spline, m_correctionSpline, m_xSpline, m_ySpline;

public:

    ZMPTrajectoryGeneratorImplementation()
        : m_buffer(2)
        , m_timesBuffer(2)
        , m_correctionBuffer(2)
        , m_xBuffer(2)
        , m_yBuffer(2)
        , m_spline(2)
        , m_correctionSpline(2)
        , m_xSpline(2)
        , m_ySpline(2)
    { }

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

        if (initStates.size() != mergePoints.size())
            initStates.resize(mergePoints.size());

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
                        initStates[mergePoint].initialPosition = output[instant];
                        initStates[mergePoint].initialVelocity = outputVelocity[instant];
                        initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                        if (mergePoint < (mergePoints.size() - 1))
                            mergePoint++;
                    }

                    ++instant;
                }
            } else if ((stepPhase[instant] == StepPhase::SwitchIn)||(stepPhase[instant] == StepPhase::SwitchOut)){

                switchLength = (endOfPhase-instant) * dT;
                bool pause = pauseActive && (switchLength > std::max(maxSwitchTime, endSwitchTime + nominalSwitchTime)); //if true, it will pause in the middle
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
                    m_timesBuffer(1) = endSwitchTime;

                    if (!m_spline.setData(m_timesBuffer, m_buffer)){
                        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the spline for the weight portion during pause." << std::endl;
                        return false;
                    }
                    initialSwitchInstant = instant;
                    while (instant < (initialSwitchInstant + std::round(endSwitchTime / dT))){
                        switchInstant = (instant - initialSwitchInstant)*dT;
                        output[instant] = m_spline.evaluatePoint(switchInstant, outputVelocity[instant], outputAcceleration[instant]);

                        if (instant == mergePoints[mergePoint]){
                            initStates[mergePoint].initialPosition = output[instant];
                            initStates[mergePoint].initialVelocity = outputVelocity[instant];
                            initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                            if (mergePoint < (mergePoints.size() - 1))
                                mergePoint++;
                        }

                        ++instant;
                    }

                    while (instant < (endOfPhase - std::round(nominalSwitchTime/ dT))){
                        output[instant] = 0.5;
                        outputVelocity[instant] = 0.0;
                        outputAcceleration[instant] = 0.0;

                        if (instant == mergePoints[mergePoint]){
                            initStates[mergePoint].initialPosition = output[instant];
                            initStates[mergePoint].initialVelocity = outputVelocity[instant];
                            initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

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
                        initStates[mergePoint].initialPosition = output[instant];
                        initStates[mergePoint].initialVelocity = outputVelocity[instant];
                        initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

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

                m_xBuffer(0) = stanceZmpPosition(0);
                m_yBuffer(0) = stanceZmpPosition(1);
                m_timesBuffer(0) = 0.0;
                m_xSpline.setInitialConditions(0.0, 0.0);
                m_ySpline.setInitialConditions(0.0, 0.0);

                stanceLength = (endOfPhase-instant) * dT;
                m_xBuffer(1) = switchZmpInitPosition(0);
                m_yBuffer(1) = switchZmpInitPosition(1);
                m_timesBuffer(1) = stanceLength;
                m_xSpline.setFinalConditions(0.0, 0.0);
                m_ySpline.setFinalConditions(0.0, 0.0);

                if (!m_xSpline.setData(m_timesBuffer, m_xBuffer)){
                    std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the ZMPx spline in stance phase." << std::endl;
                    return false;
                }
                if (!m_ySpline.setData(m_timesBuffer, m_yBuffer)){
                    std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the ZMPy spline in stance phase." << std::endl;
                    return false;
                }

                initialInstant = instant;
                while (instant < endOfPhase){
                    elapsedTime = (instant - initialInstant)*dT;
                    output[instant](0) = m_xSpline.evaluatePoint(elapsedTime, outputVelocity[instant](0), outputAcceleration[instant](0));
                    output[instant](1) = m_ySpline.evaluatePoint(elapsedTime, outputVelocity[instant](1), outputAcceleration[instant](1));
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
                    bool pause = pauseActive && (switchLength > std::max(maxSwitchTime,endSwitchTime + nominalSwitchTime)); //if true, it will pause in the middle

                    m_xBuffer(0) = switchZmpInitPosition(0);
                    m_yBuffer(0) = switchZmpInitPosition(1);
                    m_timesBuffer(0) = 0.0;
                    m_xSpline.setInitialConditions(0.0, 0.0);
                    m_ySpline.setInitialConditions(0.0, 0.0);

                    m_xBuffer(1) = stanceZmpPosition(0); //bring the ZMP back to the stance position
                    m_yBuffer(1) = stanceZmpPosition(1);
                    if (pause){
                        m_timesBuffer(1) = endSwitchTime;
                    } else if (phase == (phaseShift.size() - 1)){
                        m_timesBuffer(1) = (endOfPhase - instant)*dT;
                    } else {
                        m_timesBuffer(1) = switchLength/2;
                    }

                    m_xSpline.setFinalConditions(0.0, 0.0);
                    m_ySpline.setFinalConditions(0.0, 0.0);

                    if (!m_xSpline.setData(m_timesBuffer, m_xBuffer)){
                        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the ZMPx spline in switch phase." << std::endl;
                        return false;
                    }
                    if (!m_ySpline.setData(m_timesBuffer, m_yBuffer)){
                        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed to initialize the ZMPy spline in switch phase." << std::endl;
                        return false;
                    }
                    initialInstant = instant;
                    while (instant < (initialInstant + std::ceil(m_timesBuffer(1)/dT))){
                        elapsedTime = (instant - initialInstant)*dT;
                        output[instant](0) = m_xSpline.evaluatePoint(elapsedTime, outputVelocity[instant](0), outputAcceleration[instant](0));
                        output[instant](1) = m_ySpline.evaluatePoint(elapsedTime, outputVelocity[instant](1), outputAcceleration[instant](1));
                        ++instant;
                    }

                    while (instant < endOfPhase) {
                        output[instant](0) = m_xBuffer(1);
                        output[instant](1) = m_yBuffer(1);
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

    void computeGlobalZMP(double initTime, double dT, const std::vector<size_t>& phaseShift, const std::vector<const Step*>& orderedSteps, const FootPrint &left, const FootPrint &right) {
        //NOTE This must be called after that both the local ZMPs, the weight portions and the feet are computed
        iDynTree::Position leftWorldZMP, rightWorldZMP;
        iDynTree::Position leftWorldZMPVelocity, rightWorldZMPVelocity;
        iDynTree::Position leftWorldZMPAcceleration, rightWorldZMPAcceleration;

        worldZMP.resize(leftZMP.size());
        worldZMPVelocity.resize(leftZMPVelocity.size());
        worldZMPAcceleration.resize(leftZMPAcceleration.size());


        m_correctionBuffer(0) = 1.0;
        m_timesBuffer(0) = 0.0;
        m_correctionBuffer(1) = 0.0;
        m_timesBuffer(1) = phaseShift[1] * dT;
        m_correctionSpline.setInitialConditions(0.0, 0.0);
        m_correctionSpline.setFinalConditions(0.0, 0.0);
        m_correctionSpline.setData(m_timesBuffer, m_correctionBuffer);

        iDynTree::Transform oldLeftH, oldRightH;
        oldLeftH.setPosition(pos3D(previousLeft.position));
        oldLeftH.setRotation(iDynTree::Rotation::RotZ(previousLeft.angle));
        oldRightH.setPosition(pos3D(previousRight.position));
        oldRightH.setRotation(iDynTree::Rotation::RotZ(previousRight.angle));
        iDynTree::Position deltaL, deltaR;

        std::string leftName = left.getFootName();
        std::string rightName = right.getFootName();
        assert(leftName != rightName);


        double correction = 0.0, correctionVelocity = 0.0, correctionAcceleration = 0.0;
        size_t currentLeftIndex = orderedSteps.size() - 1, currentRightIndex = orderedSteps.size() - 1;

        while (orderedSteps[currentLeftIndex]->footName != leftName) { //find last left step (there should be at least one left step)
            currentLeftIndex--;
        }

        while (orderedSteps[currentRightIndex]->footName != rightName) { //find last left step (there should be at least one left step)
            currentRightIndex--;
        }

        iDynTree::Transform leftTransform, rightTransform;

        size_t instant;
        double currentTime;
        for (size_t reverseInstant = 0; reverseInstant < leftZMP.size(); ++reverseInstant){

            instant = leftZMP.size() - 1 - reverseInstant; //spanning backward, so that it is easier to undestand which is the foot position

            currentTime = instant * dT + initTime;

            if (orderedSteps[currentLeftIndex]->impactTime > currentTime) {
                while ((orderedSteps[currentLeftIndex]->footName != leftName) || (orderedSteps[currentLeftIndex]->impactTime > currentTime)) {
                    currentLeftIndex--;
                }
            }

            leftTransform.setPosition(pos3D(orderedSteps[currentLeftIndex]->position));
            leftTransform.setRotation(iDynTree::Rotation::RPY(0.0, 0.0, orderedSteps[currentLeftIndex]->angle));

            if (orderedSteps[currentRightIndex]->impactTime > currentTime) {
                while ((orderedSteps[currentRightIndex]->footName != rightName) || (orderedSteps[currentRightIndex]->impactTime > currentTime)) {
                    currentRightIndex--;
                }
            }

            rightTransform.setPosition(pos3D(orderedSteps[currentRightIndex]->position));
            rightTransform.setRotation(iDynTree::Rotation::RPY(0.0, 0.0, orderedSteps[currentRightIndex]->angle));


            leftWorldZMP = pos3D(leftTransform, leftZMP[instant]);
            rightWorldZMP = pos3D(rightTransform, rightZMP[instant]);

            if (instant < phaseShift[1]){
                correction = m_correctionSpline.evaluatePoint(instant*dT, correctionVelocity, correctionAcceleration);
            } else {
                correction = 0.0;
                correctionVelocity = 0.0;
                correctionAcceleration = 0.0;
            }

            if (instant < phaseShift[1]){ //first half switch
                deltaL = pos3D(oldLeftH, leftZMP[instant]) - leftWorldZMP;
                deltaR = pos3D(oldRightH, rightZMP[instant]) - rightWorldZMP;
                for (unsigned int i = 0; i < 2; ++i){
                    worldZMP[instant](i) = weightInLeft[instant] * (leftWorldZMP(i) + correction * deltaL(i)) +
                            weightInRight[instant] * (rightWorldZMP(i) + correction * deltaR(i));
                }
            } else {
                for (unsigned int i = 0; i < 2; ++i){
                    worldZMP[instant](i) = weightInLeft[instant] * leftWorldZMP(i) + weightInRight[instant] * rightWorldZMP(i);
                }
            }


            leftWorldZMPVelocity = pos3D(leftTransform, leftZMPVelocity[instant]);
            rightWorldZMPVelocity = pos3D(rightTransform, rightZMPVelocity[instant]);

            if (instant < phaseShift[1]){
                for (unsigned int i = 0; i < 2; ++i){ //NOTE!! HERE WE ARE ASSUMING THAT NEITHER THE FEET, NOR THE LOCAL ZMPs ARE MOVING (only for the first phase)
                    worldZMPVelocity[instant](i) = weightInLeftVelocity[instant] * (leftWorldZMP(i) + correction * deltaL(i)) +
                            weightInLeft[instant] * correctionVelocity * deltaL(i) +
                            weightInRightVelocity[instant] * (rightWorldZMP(i) + correction * deltaR(i)) +
                            weightInRight[instant] * correctionVelocity * deltaR(i);
                }
            } else {
                for (unsigned int i = 0; i < 2; ++i){
                    worldZMPVelocity[instant](i) = weightInLeftVelocity[instant] * leftWorldZMP(i) +
                            weightInLeft[instant] * leftWorldZMPVelocity(i) +
                            weightInRightVelocity[instant] * rightWorldZMP(i) +
                            weightInRight[instant] * rightWorldZMPVelocity(i);
                }
            }

            leftWorldZMPAcceleration = pos3D(leftTransform, leftZMPAcceleration[instant]);
            rightWorldZMPAcceleration = pos3D(rightTransform, rightZMPAcceleration[instant]);

            if (instant < phaseShift[1]){
                for (unsigned int i = 0; i < 2; ++i){ //NOTE!! HERE WE ARE ASSUMING THAT NEITHER THE FEET, NOR THE LOCAL ZMPs ARE MOVING (only for the first phase)
                    worldZMPAcceleration[instant](i) = weightInLeftAcceleration[instant] * (leftWorldZMP(i) + correction * deltaL(i)) +
                            2 * weightInLeftVelocity[instant] * correctionVelocity * deltaL(i) +
                            weightInLeft[instant] * correctionAcceleration * deltaL(i) +
                            weightInRightAcceleration[instant] * (rightWorldZMP(i) + correction * deltaR(i)) +
                            2 * weightInRightVelocity[instant] * correctionVelocity * deltaR(i) +
                            weightInRight[instant] * correctionAcceleration * deltaR(i);
                }
            } else {
                for (unsigned int i = 0; i < 2; ++i){
                    worldZMPAcceleration[instant](i) = weightInLeftAcceleration[instant] * leftWorldZMP(i) +
                            2 * weightInLeftVelocity[instant] * leftWorldZMPVelocity(i) +
                            weightInLeft[instant] * leftWorldZMPAcceleration(i) +
                            weightInRightAcceleration[instant] * rightWorldZMP(i) +
                            2 * weightInRightVelocity[instant] * rightWorldZMPVelocity(i) +
                            weightInRight[instant] * rightWorldZMPAcceleration(i);
                }
            }
        }
    }
};




bool ZMPTrajectoryGenerator::computeNewTrajectories(double initTime, double dT, double switchPercentage, double maxStepTime, double endSwitchTime,
                                                    double nominalStepTime, bool pauseActive, const std::vector<size_t> &mergePoints,
                                                    const FootPrint &left, const FootPrint &right, const std::vector<const Step *> &orderedSteps,
                                                    const std::vector<StepPhase> &lFootPhases, const std::vector<StepPhase> &rFootPhases, const std::vector<size_t> &phaseShift)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->nominalSwitchTime = switchPercentage * nominalStepTime;
    m_pimpl->maxSwitchTime = switchPercentage * maxStepTime;
    m_pimpl->maxSwingTime = maxStepTime - m_pimpl->maxSwitchTime;
    m_pimpl->nominalSwingTime = nominalStepTime - m_pimpl->nominalSwitchTime;
    m_pimpl->endSwitchTime = endSwitchTime;

    if (m_pimpl->nominalSwitchTime + endSwitchTime > m_pimpl->maxSwitchTime) {
        if (!m_pimpl->timingWarningPrinted) {
            std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Warning: the sum of nominalSwitchTime and endSwitchTime is greater than maxSwitchTime. ";
            std::cerr << "The robot might not be able to pause in the middle of the stance phase." << std::endl;
            m_pimpl->timingWarningPrinted = true;
        }
    }
    else {
        m_pimpl->timingWarningPrinted = false;
    }

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
                                            m_pimpl->weightInLeft, m_pimpl->weightInLeftVelocity, m_pimpl->weightInLeftAcceleration))) {
        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed while computing the weight percentage on the left foot." << std::endl;
        return false;
    }

    m_pimpl->mirrorWeightPortion(m_pimpl->weightInLeft, m_pimpl->weightInLeftVelocity, m_pimpl->weightInLeftAcceleration,
                                 m_pimpl->weightInRight, m_pimpl->weightInRightVelocity, m_pimpl->weightInRightAcceleration);

    if (!(m_pimpl->computeLocalZMP(dT, pauseActive, lFootPhases, phaseShift, m_pimpl->leftStanceZMP, m_pimpl->leftSwitchZMP,
                                   m_pimpl->leftZMP, m_pimpl->leftZMPVelocity, m_pimpl->leftZMPAcceleration))) {
        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed while computing the left local ZMP." << std::endl;
        return false;
    }

    if (!(m_pimpl->computeLocalZMP(dT, pauseActive, rFootPhases, phaseShift, m_pimpl->rightStanceZMP, m_pimpl->rightSwitchZMP,
                                   m_pimpl->rightZMP, m_pimpl->rightZMPVelocity, m_pimpl->rightZMPAcceleration))) {
        std::cerr << "[ZMPTrajectoryGenerator::computeNewTrajectories] Failed while computing the right local ZMP." << std::endl;
        return false;
    }

    m_pimpl->computeGlobalZMP(initTime, dT, phaseShift, orderedSteps, left, right);


    m_pimpl->initialWeightSpecified = false;
    m_pimpl->previousStepsSpecified = false;

    return true;
}

ZMPTrajectoryGenerator::ZMPTrajectoryGenerator()
    : m_pimpl(std::make_unique<ZMPTrajectoryGeneratorImplementation>())
{
    assert(m_pimpl);
    m_pimpl->leftStanceZMP.zero();
    m_pimpl->leftSwitchZMP.zero();
    m_pimpl->rightStanceZMP.zero();
    m_pimpl->rightSwitchZMP.zero();
}

ZMPTrajectoryGenerator::~ZMPTrajectoryGenerator()
{
}

bool ZMPTrajectoryGenerator::setWeightInitialState(const InitialState &initialState)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->initialState = initialState;
    m_pimpl->initialWeightSpecified = true;
    return true;
}

bool ZMPTrajectoryGenerator::setPreviousSteps(const Step &previousLeft, const Step &previousRight)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->previousLeft = previousLeft;
    m_pimpl->previousRight = previousRight;

    m_pimpl->previousStepsSpecified = true;

    return true;
}

bool ZMPTrajectoryGenerator::setStanceZMPDelta(const iDynTree::Vector2 &offsetInLeftFootFrame, const iDynTree::Vector2 &offsetInRightFootFrame)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->leftStanceZMP = offsetInLeftFootFrame;
    m_pimpl->rightStanceZMP = offsetInRightFootFrame;
    return true;
}

bool ZMPTrajectoryGenerator::setInitialSwitchZMPDelta(const iDynTree::Vector2 &offsetInLeftFootFrame, const iDynTree::Vector2 &offsetInRightFootFrame)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->leftSwitchZMP = offsetInLeftFootFrame;
    m_pimpl->rightSwitchZMP = offsetInRightFootFrame;
    return true;
}

void ZMPTrajectoryGenerator::getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInRight) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    weightInLeft = m_pimpl->weightInLeft;
    weightInRight = m_pimpl->weightInRight;
}

void ZMPTrajectoryGenerator::getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInLeftFirstDerivative, std::vector<double> &weightInLeftSecondDerivative, std::vector<double> &weightInRight, std::vector<double> &weightInRightFirstDerivative, std::vector<double> &weightInRightSecondDerivative) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    weightInLeft = m_pimpl->weightInLeft;
    weightInLeftFirstDerivative = m_pimpl->weightInLeftVelocity;
    weightInLeftSecondDerivative = m_pimpl->weightInLeftAcceleration;
    weightInRight = m_pimpl->weightInRight;
    weightInRightFirstDerivative = m_pimpl->weightInRightVelocity;
    weightInRightSecondDerivative = m_pimpl->weightInRightAcceleration;
}

void ZMPTrajectoryGenerator::getZMPTrajectory(std::vector<iDynTree::Vector2> &ZMPTrajectory) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    ZMPTrajectory = m_pimpl->worldZMP;
}

void ZMPTrajectoryGenerator::getZMPTrajectory(std::vector<iDynTree::Vector2> &ZMPTrajectory, std::vector<iDynTree::Vector2> &ZMPVelocity, std::vector<iDynTree::Vector2> &ZMPAcceleration) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    ZMPTrajectory = m_pimpl->worldZMP;
    ZMPVelocity = m_pimpl->worldZMPVelocity;
    ZMPAcceleration = m_pimpl->worldZMPAcceleration;
}

void ZMPTrajectoryGenerator::getLocalZMPTrajectories(std::vector<iDynTree::Vector2> &leftZMPTrajectory, std::vector<iDynTree::Vector2> &rightZMPTrajectory) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    leftZMPTrajectory = m_pimpl->leftZMP;
    rightZMPTrajectory = m_pimpl->rightZMP;
}

void ZMPTrajectoryGenerator::getLocalZMPTrajectories(std::vector<iDynTree::Vector2> &leftZMPTrajectory, std::vector<iDynTree::Vector2> &leftZMPVelocity, std::vector<iDynTree::Vector2> &leftZMPAcceleration, std::vector<iDynTree::Vector2> &rightZMPTrajectory, std::vector<iDynTree::Vector2> &rightZMPVelocity, std::vector<iDynTree::Vector2> &rightZMPAcceleration) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    leftZMPTrajectory = m_pimpl->leftZMP;
    rightZMPTrajectory = m_pimpl->rightZMP;

    leftZMPVelocity = m_pimpl->leftZMPVelocity;
    rightZMPVelocity = m_pimpl->rightZMPVelocity;

    leftZMPAcceleration = m_pimpl->leftZMPAcceleration;
    rightZMPAcceleration = m_pimpl->rightZMPAcceleration;
}

void ZMPTrajectoryGenerator::getInitialStatesAtMergePoints(std::vector<InitialState> &initialStates) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);
    initialStates = m_pimpl->initStates;
}

