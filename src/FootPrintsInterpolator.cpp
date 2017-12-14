/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "FootPrintsInterpolator.h"
#include <iostream>
#include <algorithm>
#include <cmath>

bool FeetInterpolator::orderSteps()
{
    m_orderedSteps.clear();
    m_orderedSteps.reserve(m_left.numberOfSteps() + m_right.numberOfSteps() - 2);
    if (m_orderedSteps.capacity() > 0){
        for (StepsIndex footL = m_left.getSteps().cbegin() + 1; footL != m_left.getSteps().cend(); ++footL)
            m_orderedSteps.push_back(footL);
        for (StepsIndex footR = m_right.getSteps().cbegin() + 1; footR != m_right.getSteps().cend(); ++footR)
            m_orderedSteps.push_back(footR);

        std::sort(m_orderedSteps.begin(), m_orderedSteps.end(),
                  [](const StepsIndex&a, const StepsIndex&b) { return a->impactTime < b->impactTime;});
        auto duplicate = std::adjacent_find(m_orderedSteps.begin(), m_orderedSteps.end(),
                                            [](const StepsIndex&a, const StepsIndex&b) { return a->impactTime == b->impactTime;});

        if (duplicate != m_orderedSteps.end()){
            std::cerr << "[FEETINTERPOLATOR] Two entries of the FootPrints pointers have the same impactTime. (The head is not considered)"
                      << std::endl;
            return false;
        }
    }

    return true;
}

bool FeetInterpolator::createPhasesTimings()
{
    //NOTE this method must be called after orderSteps to work properly
    if (m_switchPercentage < 0){
        std::cerr << "[FEETINTERPOLATOR] First you have to define the ratio between switch and swing phases." << std::endl;
        return false;
    }

    m_lFootPhases.reset(new std::vector<StepPhase>());
    m_rFootPhases.reset(new std::vector<StepPhase>());

    m_phaseShift.clear();
    m_phaseShift.push_back(0); //necessary, otherwise I cannot call m_phaseShift.back() later

    m_mergePoints.clear();
    m_mergePoints.push_back(0); //attach a completely new trajectory

    std::shared_ptr<std::vector<StepPhase> > swing, stance;

    if (m_orderedSteps.size() == 0){
        int endSwitchSamples = std::round(m_endSwitch/m_dT); //last shift to the center

        m_lFootPhases->reserve(endSwitchSamples);
        m_rFootPhases->reserve(endSwitchSamples);

        swing = (m_left.getSteps().front().impactTime > m_right.getSteps().front().impactTime) ? m_lFootPhases : m_rFootPhases;
        stance = (m_left.getSteps().front().impactTime > m_right.getSteps().front().impactTime) ? m_rFootPhases : m_lFootPhases;

        swing->insert(swing->end(), endSwitchSamples, StepPhase::SwitchIn);
        stance->insert(stance->end(), endSwitchSamples, StepPhase::SwitchOut);
        m_phaseShift.push_back(endSwitchSamples);
        m_mergePoints.push_back(endSwitchSamples - 1);

        return true;
    }

    double totalTime = m_orderedSteps.back()->impactTime - m_initTime + m_endSwitch;
    int trajectoryDimension = std::ceil(totalTime/m_dT);

    m_lFootPhases->reserve(trajectoryDimension); // Notice that this dimension may not be the final dimension, due to rounding errors!!!
    m_rFootPhases->reserve(trajectoryDimension);

    double stepTime, switchTime, pauseTime;
    int stepSamples, switchSamples, swingSamples;

    StepsIndex leftIndex = m_left.getSteps().cbegin() + 1;
    StepsIndex rightIndex = m_right.getSteps().cbegin() + 1;
    size_t orderedStepIndex = 0;
    StepsIndex nextStepindex;
    double previouStepTime = m_initTime;

    while (orderedStepIndex < m_orderedSteps.size()){
        nextStepindex = m_orderedSteps[orderedStepIndex];
        stepTime = nextStepindex->impactTime - previouStepTime;

        if (stepTime < 0){
            std::cerr <<"Something went wrong. The stepTime appears to be negative." << std::endl;
            return false;
        }

        if (nextStepindex == m_orderedSteps.front()){ //first half step
            //Timings
            switchTime = (m_switchPercentage/(1 - (m_switchPercentage/2.0)) * stepTime)/2.0; //half switch
        } else { //general case
            switchTime = m_switchPercentage * stepTime; //full switch
        }

        bool pause = m_pauseActive && (stepTime > m_maxStepTime); //if true, it will pause in the middle
        if (pause){
            pauseTime = stepTime - m_nominalStepTime;
            switchTime = m_nominalSwitchTime + pauseTime;
        } else pauseTime = 0;

        //Samples
        stepSamples = std::round(stepTime/m_dT);
        switchSamples = std::round(switchTime/m_dT);
        swingSamples = stepSamples - switchSamples;

        if (leftIndex == nextStepindex){
            swing = m_lFootPhases;
            stance = m_rFootPhases;
            leftIndex++;
        } else if (rightIndex == nextStepindex){
            swing = m_rFootPhases;
            stance = m_lFootPhases;
            rightIndex++;
        } else {
            std::cerr << "[FEETINTERPOLATOR] Something went wrong." << std::endl;
            return false;
        }

        swing->insert(swing->end(), switchSamples, StepPhase::SwitchOut); //insert the value "StepPhase::SwitchOut" switchSamples times
        stance->insert(stance->end(), switchSamples, StepPhase::SwitchIn);
        m_phaseShift.push_back(m_phaseShift.back() + switchSamples); //it stores the indeces when a change of phase occurs

        if (nextStepindex != m_orderedSteps.front()){ //add no merge point in the first half switch
            //bool pause = m_pauseActive && (switchTime > m_maxSwitchTime); //if true, it will pause in the middle
            size_t mergePoint;
            if (pause){
                mergePoint = m_phaseShift.back() - std::round(m_nominalSwitchTime/(2*m_dT));
                m_mergePoints.push_back(mergePoint);
            } else {
                mergePoint = m_phaseShift.back() - std::round(switchTime/(2*m_dT));
                m_mergePoints.push_back(mergePoint);
            }
        }

        swing->insert(swing->end(), swingSamples, StepPhase::Swing); //first step
        stance->insert(stance->end(), swingSamples, StepPhase::Stance);
        m_phaseShift.push_back(m_phaseShift.back() + swingSamples);

        previouStepTime += stepSamples*m_dT; //to take into account samples lost by numeric errors
        orderedStepIndex++;
    }
    switchSamples = std::round(m_endSwitch/m_dT); //last shift to the center
    swing->insert(swing->end(), switchSamples, StepPhase::SwitchIn);
    stance->insert(stance->end(), switchSamples, StepPhase::SwitchOut);
    m_phaseShift.push_back(m_phaseShift.back() + switchSamples);

    m_mergePoints.push_back(m_phaseShift.back() - 1); //merge on the last

    m_lFootPhases->shrink_to_fit();
    m_rFootPhases->shrink_to_fit();

    return true;
}

void FeetInterpolator::fillFeetStandingPeriodsVectors()
{
    //NOTE this must be called after createPhasesTimings
    m_lFootContact.resize(m_lFootPhases->size());
    m_rFootContact.resize(m_rFootPhases->size());

    for (int instant = 0; instant < m_lFootContact.size(); ++instant){
        if (m_lFootPhases->at(instant) == StepPhase::Swing)
            m_lFootContact[instant] = false;
        else m_lFootContact[instant] = true;
    }

    for (int instant = 0; instant < m_rFootContact.size(); ++instant){
        if (m_rFootPhases->at(instant) == StepPhase::Swing)
            m_rFootContact[instant] = false;
        else m_rFootContact[instant] = true;
    }

}

void FeetInterpolator::fillLeftFixedVector()
{
    //NOTE this must be called after createPhasesTimings
    m_leftFixed.resize(m_lFootPhases->size());

    for (int instant = 0; instant < m_leftFixed.size(); ++instant){
        m_leftFixed[instant] = (m_lFootPhases->at(instant) == StepPhase::Stance)||(m_lFootPhases->at(instant) == StepPhase::SwitchOut);
    }
}

bool FeetInterpolator::interpolateFoot(const std::vector<StepPhase> &stepPhase, const FootPrint &foot, std::vector<iDynTree::Transform> &output)
{
    //NOTE this must be called after createPhasesTimings

    if (m_stepHeight < 0){
        std::cerr << "[FEETINTERPOLATOR] First you have to set the step height." << std::endl;
        return false;
    }

    if (output.size() != stepPhase.size())
        output.resize(stepPhase.size());

    static iDynTree::VectorDynSize xPositionsBuffer(2), yPositionsBuffer(2), zPositionsBuffer(3), yawsBuffer(2), timesBuffer(2), zTimesBuffer(3);
    static iDynTree::CubicSpline xSpline(2), ySpline(2), zSpline(3), yawSpline(2);

    const StepList& steps = foot.getSteps();
    StepList::const_iterator footState = steps.begin();
    size_t instant = 0;
    iDynTree::Transform newTransform;
    iDynTree::Position newPosition;
    double swingLength;
    size_t endOfPhase;

    for (size_t phase = 1; phase < m_phaseShift.size(); ++phase){ //the first value is useless

        endOfPhase = m_phaseShift[phase];

        if (stepPhase[instant] == StepPhase::Swing){
            //create the interpolation points
            //increase footState
            //prepare splines
            //iterate for every instant asking the interpolator

            swingLength = (endOfPhase-instant) * m_dT;

            xPositionsBuffer(0) = footState->position(0);
            yPositionsBuffer(0) = footState->position(1);
            yawsBuffer(0) = footState->angle;
            timesBuffer(0) = 0.0;
            zPositionsBuffer(0) = 0.0;
             zTimesBuffer(0) = 0.0;
            zPositionsBuffer(1) = m_stepHeight; //the z has an additional midpoint
             zTimesBuffer(1) = m_swingApex*swingLength;

            if (footState + 1 == steps.cend()){
                std::cerr << "[FEETINTERPOLATOR] Something went wrong. stepPhase and foot don't seem to be coherent." << std::endl;
                return false;
            }
            ++footState;

            xPositionsBuffer(1) = footState->position(0);
            yPositionsBuffer(1) = footState->position(1);
            yawsBuffer(1) = footState->angle;
            timesBuffer(1) = swingLength;
            zPositionsBuffer(2) = 0.0;
             zTimesBuffer(2) = swingLength;

            //preparation for splines

            xSpline.setInitialConditions(0.0, 0.0);
            ySpline.setInitialConditions(0.0, 0.0);
            zSpline.setInitialConditions(0.0, 0.0);
            yawSpline.setInitialConditions(0.0, 0.0);

            xSpline.setFinalConditions(0.0, 0.0);
            ySpline.setFinalConditions(0.0, 0.0);
            zSpline.setFinalConditions(0.0, 0.0); //we may think of non-null final acceleration for the z
            yawSpline.setFinalConditions(0.0, 0.0);

            if (!xSpline.setData(timesBuffer, xPositionsBuffer)){
                std::cerr << "[FEETINTERPOLATOR] Failed to initialize the x-dimension spline." << std::endl;
                return false;
            }
            if (!ySpline.setData(timesBuffer, yPositionsBuffer)){
                std::cerr << "[FEETINTERPOLATOR] Failed to initialize the y-dimension spline." << std::endl;
                return false;
            }
            if (!zSpline.setData(zTimesBuffer, zPositionsBuffer)){
                std::cerr << "[FEETINTERPOLATOR] Failed to initialize the z-dimension spline." << std::endl;
                return false;
            }
            if (!yawSpline.setData(timesBuffer, yawsBuffer)){
                std::cerr << "[FEETINTERPOLATOR] Failed to initialize the yaw-dimension spline." << std::endl;
                return false;
            }

            size_t startSwingInstant = instant;
            double interpolationTime;
            while (instant < endOfPhase){
                interpolationTime = (instant - startSwingInstant)*m_dT;
                newPosition(0) = xSpline.evaluatePoint(interpolationTime);
                newPosition(1) = ySpline.evaluatePoint(interpolationTime);
                newPosition(2) = zSpline.evaluatePoint(interpolationTime);
                newTransform.setPosition(newPosition);
                newTransform.setRotation(iDynTree::Rotation::RPY(0.0, 0.0, yawSpline.evaluatePoint(interpolationTime)));

                if (newPosition(2) < 0){
                    std::cerr << "[FEETINTERPOLATOR] The z of the foot is negative at time " << instant*m_dT + m_initTime;
                    std::cerr<<". Continuing anyway." <<std::endl;
                }
                output[instant] = newTransform;

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
                ++instant;
            }
        }
    }

    return true;
}

bool FeetInterpolator::computeFootWeightPortion(const std::vector<StepPhase> &stepPhase, const InitialState &alpha0, std::vector<double> &output, std::vector<double> &outputVelocity, std::vector<double> &outputAcceleration)
{
    //NOTE this must be called after createPhasesTimings

    if (output.size() != stepPhase.size())
        output.resize(stepPhase.size());
    if (outputVelocity.size() != stepPhase.size())
        outputVelocity.resize(stepPhase.size());
    if (outputAcceleration.size() != stepPhase.size())
        outputAcceleration.resize(stepPhase.size());

    if (m_initStates.size() != m_mergePoints.size())
        m_initStates.resize(m_mergePoints.size());

    static iDynTree::VectorDynSize buffer(2), timesBuffer(2);
    static iDynTree::CubicSpline spline(2);

    size_t instant = 0, initialSwitchInstant, endOfPhase;
    double switchLength, switchInstant;
    size_t mergePoint = 0;

    for (size_t phase = 1; phase < m_phaseShift.size(); ++phase){ //the first value is useless

        endOfPhase = m_phaseShift[phase];

        if ((stepPhase[instant] == StepPhase::Stance)||(stepPhase[instant] == StepPhase::Swing)){
            while (instant < endOfPhase){
                output[instant] = (stepPhase[instant] == StepPhase::Stance) ? 1.0 : 0.0;
                outputVelocity[instant] = 0.0;
                outputAcceleration[instant] = 0.0;

                if (instant == m_mergePoints[mergePoint]){
                    m_initStates[mergePoint].initialPosition = output[instant];
                    m_initStates[mergePoint].initialVelocity = outputVelocity[instant];
                    m_initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                    if (mergePoint < (m_mergePoints.size() - 1))
                        mergePoint++;
                }

                ++instant;
            }
        } else if ((stepPhase[instant] == StepPhase::SwitchIn)||(stepPhase[instant] == StepPhase::SwitchOut)){

            switchLength = (endOfPhase-instant) * m_dT;
            bool pause = m_pauseActive && (switchLength > m_maxSwitchTime); //if true, it will pause in the middle

            if (phase == 1){ //first half switch
                buffer(0) = alpha0.initialPosition;
                spline.setInitialConditions(alpha0.initialVelocity, alpha0.initialAcceleration);
            } else {
                buffer(0) = (stepPhase[instant] == StepPhase::SwitchIn) ? 0.0 : 1.0;
                spline.setInitialConditions(0.0, 0.0);
            }
            timesBuffer(0) = 0.0;

            if (pause){
                buffer(1) = 0.5; //pause in the middle
                spline.setFinalConditions(0.0, 0.0);
                timesBuffer(1) = m_nominalSwitchTime/2;

                if (!spline.setData(timesBuffer, buffer)){
                    std::cerr << "[FEETINTERPOLATOR] Failed to initialize the spline for the weight portion during pause." << std::endl;
                    return false;
                }
                initialSwitchInstant = instant;
                while (instant < (initialSwitchInstant + std::round(m_nominalSwitchTime/(2.0 * m_dT)))){
                    switchInstant = (instant - initialSwitchInstant)*m_dT;
                    output[instant] = spline.evaluatePoint(switchInstant, outputVelocity[instant], outputAcceleration[instant]);

                    if (instant == m_mergePoints[mergePoint]){
                        m_initStates[mergePoint].initialPosition = output[instant];
                        m_initStates[mergePoint].initialVelocity = outputVelocity[instant];
                        m_initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                        if (mergePoint < (m_mergePoints.size() - 1))
                            mergePoint++;
                    }

                    ++instant;
                }

                while (instant < (endOfPhase - std::round(m_nominalSwitchTime/(2.0 * m_dT)))){
                    output[instant] = 0.5;
                    outputVelocity[instant] = 0.0;
                    outputAcceleration[instant] = 0.0;

                    if (instant == m_mergePoints[mergePoint]){
                        m_initStates[mergePoint].initialPosition = output[instant];
                        m_initStates[mergePoint].initialVelocity = outputVelocity[instant];
                        m_initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                        if (mergePoint < (m_mergePoints.size() - 1))
                            mergePoint++;
                    }

                    ++instant;
                }
                //pause is finished
                buffer(0) = 0.5;
                spline.setInitialConditions(0.0, 0.0);
                timesBuffer(0) = 0.0;
            }

            if (phase == (m_phaseShift.size() - 1)){ //last half step
                buffer(1) = 0.5;
            } else buffer(1) = (stepPhase[instant] == StepPhase::SwitchIn) ? 1.0 : 0.0;

            switchLength = (endOfPhase-instant) * m_dT;
            timesBuffer(1) = switchLength;

            spline.setFinalConditions(0.0, 0.0);
            if (!spline.setData(timesBuffer, buffer)){
                std::cerr << "[FEETINTERPOLATOR] Failed to initialize the spline for the weight portion." << std::endl;
                return false;
            }

            initialSwitchInstant = instant;
            while (instant < endOfPhase){
                switchInstant = (instant - initialSwitchInstant)*m_dT;
                output[instant] = spline.evaluatePoint(switchInstant, outputVelocity[instant], outputAcceleration[instant]);

                if (instant == m_mergePoints[mergePoint]){
                    m_initStates[mergePoint].initialPosition = output[instant];
                    m_initStates[mergePoint].initialVelocity = outputVelocity[instant];
                    m_initStates[mergePoint].initialAcceleration = outputAcceleration[instant];

                    if (mergePoint < (m_mergePoints.size() - 1))
                        mergePoint++;
                }

                ++instant;
            }

        } else {
            std::cerr << "[FEETINTERPOLATOR] Unrecognized step phase." <<std::endl;
            return false;
        }
    }
    return true;
}

void FeetInterpolator::mirrorWeightPortion(const std::vector<double> &original, const std::vector<double> &originalVelocity,
                                           const std::vector<double> &originalAcceleration, std::vector<double> &mirrored,
                                           std::vector<double> &mirroredVelocity, std::vector<double> &mirroredAcceleration)
{
    if (mirrored.size() != original.size())
        mirrored.resize(original.size());
    if (mirroredVelocity.size() != originalVelocity.size())
        mirroredVelocity.resize(originalVelocity.size());
    if (mirroredAcceleration.size() != originalAcceleration.size())
        mirroredAcceleration.resize(originalAcceleration.size());

    for (int instant = 0; instant < original.size(); ++instant){
        mirrored[instant] = 1.0 - original[instant];
    }

    for (int instant = 0; instant < originalVelocity.size(); ++instant){
        mirroredVelocity[instant] = -originalVelocity[instant];
    }

    for (int instant = 0; instant < originalAcceleration.size(); ++instant){
        mirroredAcceleration[instant] = -originalAcceleration[instant];
    }
}

bool FeetInterpolator::computeLocalZMP(const std::vector<StepPhase> &stepPhase,
                                       const iDynTree::Vector2 &stanceZmpPosition,
                                       const iDynTree::Vector2 &switchZmpInitPosition,
                                       std::vector<iDynTree::Vector2>& output,
                                       std::vector<iDynTree::Vector2> &outputVelocity,
                                       std::vector<iDynTree::Vector2> &outputAcceleration)
{
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

    for (size_t phase = 1; phase < m_phaseShift.size(); ++phase){

        endOfPhase = m_phaseShift[phase];

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

            stanceLength = (endOfPhase-instant) * m_dT;
            xBuffer(1) = switchZmpInitPosition(0);
            yBuffer(1) = switchZmpInitPosition(1);
            timeBuffer(1) = stanceLength;
            xSpline.setFinalConditions(0.0, 0.0);
            ySpline.setFinalConditions(0.0, 0.0);

            if (!xSpline.setData(timeBuffer, xBuffer)){
                std::cerr << "[FEETINTERPOLATOR] Failed to initialize the ZMPx spline in stance phase." << std::endl;
                return false;
            }
            if (!ySpline.setData(timeBuffer, yBuffer)){
                std::cerr << "[FEETINTERPOLATOR] Failed to initialize the ZMPy spline in stance phase." << std::endl;
                return false;
            }

            initialInstant = instant;
            while (instant < endOfPhase){
                elapsedTime = (instant - initialInstant)*m_dT;
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
                switchLength = (endOfPhase-instant) * m_dT;
                bool pause = m_pauseActive && (switchLength > m_maxSwitchTime); //if true, it will pause in the middle

                xBuffer(0) = switchZmpInitPosition(0);
                yBuffer(0) = switchZmpInitPosition(1);
                timeBuffer(0) = 0.0;
                xSpline.setInitialConditions(0.0, 0.0);
                ySpline.setInitialConditions(0.0, 0.0);

                xBuffer(1) = stanceZmpPosition(0); //bring the ZMP back to the stance position
                yBuffer(1) = stanceZmpPosition(1);
                if (pause){
                    timeBuffer(1) = m_nominalSwitchTime/2;
                } else if (phase == (m_phaseShift.size() - 1)){
                    timeBuffer(1) = (endOfPhase - instant)*m_dT;
                } else {
                    timeBuffer(1) = switchLength/2;
                }

                xSpline.setFinalConditions(0.0, 0.0);
                ySpline.setFinalConditions(0.0, 0.0);

                if (!xSpline.setData(timeBuffer, xBuffer)){
                    std::cerr << "[FEETINTERPOLATOR] Failed to initialize the ZMPx spline in switch phase." << std::endl;
                    return false;
                }
                if (!ySpline.setData(timeBuffer, yBuffer)){
                    std::cerr << "[FEETINTERPOLATOR] Failed to initialize the ZMPy spline in switch phase." << std::endl;
                    return false;
                }
                initialInstant = instant;
                while (instant < (initialInstant + std::ceil(timeBuffer(1)/m_dT))){
                    elapsedTime = (instant - initialInstant)*m_dT;
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
            std::cerr << "[FEETINTERPOLATOR] Unrecognized step phase." <<std::endl;
            return false;
        }
    }
    return true;
}

void FeetInterpolator::computeGlobalZMP()
{
    //NOTE This must be called after that both the local ZMPs, the weight portions and the feet are computed
    iDynTree::Position leftLocalZMP, rightLocalZMP, leftWorldZMP, rightWorldZMP;
    iDynTree::Position leftWorldZMPVelocity, rightWorldZMPVelocity, leftLocalZMPVelocity, rightLocalZMPVelocity;
    iDynTree::Position leftWorldZMPAcceleration, rightWorldZMPAcceleration, leftLocalZMPAcceleration, rightLocalZMPAcceleration;

    leftLocalZMP.zero();
    rightLocalZMP.zero();
    leftWorldZMP.zero();
    rightWorldZMP.zero();

    leftWorldZMPVelocity.zero();
    rightWorldZMPVelocity.zero();
    leftLocalZMPVelocity.zero();
    rightLocalZMPVelocity.zero();

    leftWorldZMPAcceleration.zero();
    rightWorldZMPAcceleration.zero();
    leftLocalZMPAcceleration.zero();
    rightLocalZMPAcceleration.zero();

    m_worldZMP.resize(m_leftZMP.size());
    m_worldZMPVelocity.resize(m_leftZMPVelocity.size());
    m_worldZMPAcceleration.resize(m_leftZMPAcceleration.size());

    for (size_t instant = 0; instant < m_leftZMP.size(); ++instant){

        leftLocalZMP(0) = m_leftZMP[instant](0);
        leftLocalZMP(1) = m_leftZMP[instant](1);
        rightLocalZMP(0) = m_rightZMP[instant](0);
        rightLocalZMP(1) = m_rightZMP[instant](1);

        leftWorldZMP = m_leftTrajectory[instant] * leftLocalZMP;
        rightWorldZMP = m_rightTrajectory[instant] * rightLocalZMP;

        m_worldZMP[instant](0) = m_weightInLeft[instant] * leftWorldZMP(0) + m_weightInRight[instant] * rightWorldZMP(0);
        m_worldZMP[instant](1) = m_weightInLeft[instant] * leftWorldZMP(1) + m_weightInRight[instant] * rightWorldZMP(1);

        leftLocalZMPVelocity(0) = m_leftZMPVelocity[instant](0);
        leftLocalZMPVelocity(1) = m_leftZMPVelocity[instant](1);
        rightLocalZMPVelocity(0) = m_rightZMPVelocity[instant](0);
        rightLocalZMPVelocity(1) = m_rightZMPVelocity[instant](1);

        leftWorldZMPVelocity = m_leftTrajectory[instant] * leftLocalZMPVelocity;
        rightWorldZMPVelocity = m_rightTrajectory[instant] * rightLocalZMPVelocity;

        m_worldZMPVelocity[instant](0) = m_weightInLeftVelocity[instant] * leftWorldZMP(0) +
                m_weightInLeft[instant] * leftWorldZMPVelocity(0) +
                m_weightInRightVelocity[instant] * rightWorldZMP(0) +
                m_weightInRight[instant] * rightWorldZMPVelocity(0);
        m_worldZMPVelocity[instant](1) = m_weightInLeftVelocity[instant] * leftWorldZMP(1) +
                m_weightInLeft[instant] * leftWorldZMPVelocity(1) +
                m_weightInRightVelocity[instant] * rightWorldZMP(1) +
                m_weightInRight[instant] * rightWorldZMPVelocity(1);

        leftLocalZMPAcceleration(0) = m_leftZMPAcceleration[instant](0);
        leftLocalZMPAcceleration(1) = m_leftZMPAcceleration[instant](1);
        rightLocalZMPAcceleration(0) = m_rightZMPAcceleration[instant](0);
        rightLocalZMPAcceleration(1) = m_rightZMPAcceleration[instant](1);

        leftWorldZMPAcceleration = m_leftTrajectory[instant] * leftLocalZMPAcceleration;
        rightWorldZMPAcceleration = m_rightTrajectory[instant] * rightLocalZMPAcceleration;

        m_worldZMPAcceleration[instant](0) = m_weightInLeftAcceleration[instant] * leftWorldZMP(0) +
                2*m_weightInLeftVelocity[instant]*leftWorldZMPVelocity(0) +
                m_weightInLeft[instant] * leftWorldZMPAcceleration(0) +
                m_weightInRightAcceleration[instant] * rightWorldZMP(0) +
                2*m_weightInRightVelocity[instant]*rightWorldZMPVelocity(0) +
                m_weightInRight[instant] * rightWorldZMPAcceleration(0);
        m_worldZMPAcceleration[instant](1) = m_weightInLeftAcceleration[instant] * leftWorldZMP(1) +
                2*m_weightInLeftVelocity[instant]*leftWorldZMPVelocity(1) +
                m_weightInLeft[instant] * leftWorldZMPAcceleration(1) +
                m_weightInRightAcceleration[instant] * rightWorldZMP(1) +
                2*m_weightInRightVelocity[instant]*rightWorldZMPVelocity(1) +
                m_weightInRight[instant] * rightWorldZMPAcceleration(1);
    }
}

bool FeetInterpolator::computeCoMHeightTrajectory()
{
    //NOTE this must be called after createPhasesTimings
    if (m_CoMHeight < 0){
        std::cerr << "[FEETINTERPOLATOR] First you have to set the nominal CoM height." << std::endl;
        return false;
    }

    static iDynTree::VectorDynSize hBuffer(3), timesBuffer(3);
    static iDynTree::CubicSpline heightSpline(3);

    const std::vector<StepPhase>& leftPhases = *m_lFootPhases;

    if (m_CoMHeightTrajectory.size() != leftPhases.size())
        m_CoMHeightTrajectory.resize(leftPhases.size());

    if (m_CoMHeightAcceleration.size() != leftPhases.size())
        m_CoMHeightAcceleration.resize(leftPhases.size());

    if (m_CoMHeight < 0){
        std::cerr << "[FEETINTERPOLATOR] First you have to set the nominal CoM height." << std::endl;
        return false;
    }

    size_t endOfPhase, initialInstant;
    double interpolationTime, dummy, stanceLength;
    size_t instant = 0;
    for (size_t phase = 1; phase < m_phaseShift.size(); ++phase){ //the first value is useless
        endOfPhase = m_phaseShift[phase];

        if ((leftPhases[instant] == StepPhase::SwitchIn)||(leftPhases[instant] == StepPhase::SwitchOut)){
            while (instant < endOfPhase){
                m_CoMHeightTrajectory[instant] = m_CoMHeight;
                m_CoMHeightAcceleration[instant] = 0.0;
                instant++;
            }
        } else if ((leftPhases[instant] == StepPhase::Stance)||(leftPhases[instant] == StepPhase::Swing)){

            stanceLength = (endOfPhase - instant) * m_dT;
            hBuffer(0) = m_CoMHeight;
            timesBuffer(0) = 0.0;
            heightSpline.setInitialConditions(0.0, 0.0);

            hBuffer(1) = m_CoMHeight + m_CoMHeightDelta;
            timesBuffer(1) = stanceLength/2;

            hBuffer(2) = m_CoMHeight;
            timesBuffer(2) = stanceLength;
            heightSpline.setFinalConditions(0.0, 0.0);

            if (!heightSpline.setData(timesBuffer, hBuffer)){
                std::cerr << "[FEETINTERPOLATOR] Failed to initialize the height spline." << std::endl;
                return false;
            }

            initialInstant = instant;
            while (instant < endOfPhase){
                interpolationTime = (instant - initialInstant) * m_dT;
                m_CoMHeightTrajectory[instant] = heightSpline.evaluatePoint(interpolationTime, dummy, m_CoMHeightAcceleration[instant]);
                instant++;
            }
        } else {
            std::cerr << "[FEETINTERPOLATOR] Unrecognized step phase." <<std::endl;
            return false;
        }
    }

    return true;
}

FeetInterpolator::FeetInterpolator()
    :m_switchPercentage(-1.0)
    ,m_endSwitch(0.0)
    ,m_initTime(0.0)
    ,m_stepHeight(-1)
    ,m_swingApex(0.5)
    ,m_pauseActive(false)
    ,m_CoMHeight(-1.0)
    ,m_CoMHeightDelta(0.0)
{
    m_leftStanceZMP.zero();
    m_leftSwitchZMP.zero();
    m_rightStanceZMP.zero();
    m_rightSwitchZMP.zero();
}

bool FeetInterpolator::interpolate(const FootPrint &left, const FootPrint &right,
                                   double initTime, double dT, const InitialState& weightInLeftAtMergePoint)
{
    if (left.numberOfSteps() < 1){
        std::cerr << "[FEETINTERPOLATOR] No steps in the left pointer." << std::endl;
        return false;
    }

    if (right.numberOfSteps() < 1){
        std::cerr << "[FEETINTERPOLATOR] No steps in the right pointer." << std::endl;
        return false;
    }

    if (dT <= 0){
        std::cerr << "[FEETINTERPOLATOR] The dT is supposed to be positive." << std::endl;
        return false;
    }

    if ((weightInLeftAtMergePoint.initialPosition < 0) || (weightInLeftAtMergePoint.initialPosition > 1)){
        std::cerr << "[FEETINTERPOLATOR] The initial position for the weight in left is supposed to be in the [0, 1] range." << std::endl;
        return false;
    }

    double startLeft = left.getSteps().front().impactTime;
    double startRight = right.getSteps().front().impactTime;

    if (initTime < std::max(startLeft, startRight)){
        std::cerr << "[FEETINTERPOLATOR] The initTime must be greater or equal than the maximum of the first impactTime of the two feet."
                  << std::endl;
        return false;
    }

    m_maxSwitchTime = m_switchPercentage * m_maxStepTime;
    m_maxSwingTime = m_maxStepTime - m_maxSwitchTime;

    m_nominalSwitchTime = m_switchPercentage * m_nominalStepTime;
    m_nominalSwingTime = m_nominalStepTime - m_nominalSwitchTime;

    m_left = left;
    m_right = right;
    m_dT = dT;
    m_initTime = initTime;

    if (!orderSteps()){
        std::cerr << "[FEETINTERPOLATOR] Failed while ordering the steps." << std::endl;
        return false;
    }

    if (!createPhasesTimings()){
        std::cerr << "[FEETINTERPOLATOR] Failed while creating the standing periods." << std::endl;
        return false;
    }

    fillFeetStandingPeriodsVectors();
    fillLeftFixedVector();

    if (!interpolateFoot(*m_lFootPhases, left, m_leftTrajectory)){
        std::cerr << "[FEETINTERPOLATOR] Failed while interpolating left foot trajectory." << std::endl;
        return false;
    }

    if (!interpolateFoot(*m_rFootPhases, right, m_rightTrajectory)){
        std::cerr << "[FEETINTERPOLATOR] Failed while interpolating left foot trajectory." << std::endl;
        return false;
    }

    if (!computeFootWeightPortion(*m_lFootPhases, weightInLeftAtMergePoint,
                                  m_weightInLeft, m_weightInLeftVelocity, m_weightInLeftAcceleration)){
        std::cerr << "[FEETINTERPOLATOR] Failed while computing the weight percentage on the left foot." << std::endl;
        return false;
    }

    mirrorWeightPortion(m_weightInLeft, m_weightInLeftVelocity, m_weightInLeftAcceleration,
                        m_weightInRight, m_weightInRightVelocity, m_weightInRightAcceleration);

    if (!computeLocalZMP(*m_lFootPhases, m_leftStanceZMP, m_leftSwitchZMP, m_leftZMP, m_leftZMPVelocity, m_leftZMPAcceleration)){
        std::cerr << "[FEETINTERPOLATOR] Failed while computing the left local ZMP." << std::endl;
        return false;
    }

    if (!computeLocalZMP(*m_rFootPhases, m_rightStanceZMP, m_rightSwitchZMP, m_rightZMP, m_rightZMPVelocity, m_rightZMPAcceleration)){
        std::cerr << "[FEETINTERPOLATOR] Failed while computing the left local ZMP." << std::endl;
        return false;
    }

    computeGlobalZMP();


    if (!computeCoMHeightTrajectory()){
        std::cerr << "[FEETINTERPOLATOR] Failed while computing the CoM height trajectories." << std::endl;
        return false;
    }

    return true;
}

bool FeetInterpolator::interpolate(const FootPrint &left, const FootPrint &right, double initTime, double dT)
{
    InitialState alpha0;
    alpha0.initialPosition = 0.5;
    alpha0.initialVelocity = 0.0;
    alpha0.initialAcceleration = 0.0;

    return interpolate(left, right, initTime, dT, alpha0);
}

bool FeetInterpolator::setSwitchOverSwingRatio(double ratio)
{
    if (ratio <= 0){
        std::cerr << "[FEETINTERPOLATOR] The ratio is supposed to be positive." << std::endl;
        return false;
    }

    m_switchPercentage = ratio/(1.0 + ratio);
    return true;
}

bool FeetInterpolator::setTerminalHalfSwitchTime(double lastHalfSwitchTime)
{
    if (lastHalfSwitchTime < 0){
        std::cerr << "[FEETINTERPOLATOR] The lastHalfSwitchTime cannot be negative." << std::endl;
        return false;
    }

    m_endSwitch = lastHalfSwitchTime;
    return true;
}

bool FeetInterpolator::setStepHeight(double stepHeight)
{
    if (stepHeight < 0){
        std::cerr << "[FEETINTERPOLATOR] The stepHeight is supposed to be positive." << std::endl;
        return false;
    }
    m_stepHeight = stepHeight;
    return true;
}

bool FeetInterpolator::setFootApexTime(double swingTimeRatio)
{
    if ((swingTimeRatio <= 0)||(swingTimeRatio >= 1)){
        std::cerr << "[FEETINTERPOLATOR] The swingTimeRatio is supposed to be chosen in the interval (0, 1)." << std::endl;
        return false;
    }
    m_swingApex = swingTimeRatio;
    return true;
}

bool FeetInterpolator::setPauseConditions(double maxStepTime, double nominalStepTime)
{
    if (maxStepTime < 0){
        std::cerr << "[FEETINTERPOLATOR] If the maxStepTime is negative, the robot won't pause in middle stance." << std::endl;
        m_pauseActive = false;
    }

    m_pauseActive = true;
    m_maxStepTime = maxStepTime;

    if (m_pauseActive){
        if (nominalStepTime <= 0){
            std::cerr << "[FEETINTERPOLATOR] The nominalStepTime is supposed to be positive." << std::endl;
            m_pauseActive = false;
            return false;
        }

        if ((nominalStepTime) > maxStepTime){
            std::cerr << "[FEETINTERPOLATOR] The nominalSwitchTime cannot be greater than maxSwitchTime." << std::endl;
            m_pauseActive = false;
            return false;
        }
    }
    m_nominalStepTime = nominalStepTime;

    return true;
}

bool FeetInterpolator::setStanceZMPDelta(const iDynTree::Vector2 &offsetInLeftFootFrame, const iDynTree::Vector2 &offsetInRightFootFrame)
{
    m_leftStanceZMP = offsetInLeftFootFrame;
    m_rightStanceZMP = offsetInRightFootFrame;
    return true;
}

bool FeetInterpolator::setInitialSwitchZMPDelta(const iDynTree::Vector2 &offsetInLeftFootFrame, const iDynTree::Vector2 &offsetInRightFootFrame)
{
    m_leftSwitchZMP = offsetInLeftFootFrame;
    m_rightSwitchZMP = offsetInRightFootFrame;
    return true;
}

bool FeetInterpolator::setCoMHeightSettings(double comHeight, double comHeightStanceDelta)
{
    if (comHeight < 0){
        std::cerr << "[FEETINTERPOLATOR] The comHeight is supposed to be positive." << std::endl;
        return false;
    }

    if ((comHeight + comHeightStanceDelta) < 0.0){
        std::cerr << "[FEETINTERPOLATOR] The comHeightStanceDelta cannot be greater than the nominal comHeight." << std::endl;
        return false;
    }

    m_CoMHeight = comHeight;
    m_CoMHeightDelta = comHeightStanceDelta;

    return true;
}

void FeetInterpolator::getFeetTrajectories(std::vector<iDynTree::Transform> &lFootTrajectory, std::vector<iDynTree::Transform> &rFootTrajectory) const
{
    lFootTrajectory = m_leftTrajectory;
    rFootTrajectory = m_rightTrajectory;
}

void FeetInterpolator::getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInRight) const
{
    weightInLeft = m_weightInLeft;
    weightInRight = m_weightInRight;
}

void FeetInterpolator::getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInLeftFirstDerivative,
                                           std::vector<double> &weightInLeftSecondDerivative, std::vector<double> &weightInRight,
                                           std::vector<double> &weightInRightFirstDerivative,
                                           std::vector<double> &weightInRightSecondDerivative) const
{
    weightInLeft = m_weightInLeft;
    weightInRight = m_weightInRight;

    weightInLeftFirstDerivative = m_weightInLeftVelocity;
    weightInRightFirstDerivative = m_weightInRightVelocity;

    weightInLeftSecondDerivative = m_weightInLeftAcceleration;
    weightInRightSecondDerivative = m_weightInRightAcceleration;
}

void FeetInterpolator::getZMPTrajectory(std::vector<iDynTree::Vector2> &ZMPTrajectory) const
{
    ZMPTrajectory = m_worldZMP;
}

void FeetInterpolator::getZMPTrajectory(std::vector<iDynTree::Vector2> &ZMPTrajectory, std::vector<iDynTree::Vector2> &ZMPVelocity,
                                        std::vector<iDynTree::Vector2> &ZMPAcceleration) const
{
    ZMPTrajectory = m_worldZMP;
    ZMPVelocity = m_worldZMPVelocity;
    ZMPAcceleration = m_worldZMPAcceleration;
}

void FeetInterpolator::getLocalZMPTrajectories(std::vector<iDynTree::Vector2> &leftZMPTrajectory,
                                               std::vector<iDynTree::Vector2> &rightZMPTrajectory) const
{
    leftZMPTrajectory = m_leftZMP;
    rightZMPTrajectory = m_rightZMP;
}

void FeetInterpolator::getLocalZMPTrajectories(std::vector<iDynTree::Vector2> &leftZMPTrajectory,
                                               std::vector<iDynTree::Vector2> &leftZMPVelocity,
                                               std::vector<iDynTree::Vector2> &leftZMPAcceleration,
                                               std::vector<iDynTree::Vector2> &rightZMPTrajectory,
                                               std::vector<iDynTree::Vector2> &rightZMPVelocity,
                                               std::vector<iDynTree::Vector2> &rightZMPAcceleration) const
{
    leftZMPTrajectory = m_leftZMP;
    rightZMPTrajectory = m_rightZMP;

    leftZMPVelocity = m_leftZMPVelocity;
    rightZMPVelocity = m_rightZMPVelocity;

    leftZMPAcceleration = m_leftZMPAcceleration;
    rightZMPAcceleration = m_rightZMPAcceleration;
}

void FeetInterpolator::getCoMHeightTrajectory(std::vector<double> &CoMHeightTrajectory) const
{
    CoMHeightTrajectory = m_CoMHeightTrajectory;
}

void FeetInterpolator::getCoMHeightAccelerationProfile(std::vector<double> &CoMHeightAccelerationProfile) const
{
    CoMHeightAccelerationProfile = m_CoMHeightAcceleration;
}

void FeetInterpolator::getFeetStandingPeriods(std::vector<bool> &lFootContacts, std::vector<bool> &rFootContacts) const
{
    lFootContacts = m_lFootContact;
    rFootContacts = m_rFootContact;
}

void FeetInterpolator::getWhenUseLeftAsFixed(std::vector<bool> &leftIsFixed) const
{
    leftIsFixed = m_leftFixed;
}

void FeetInterpolator::getInitialStatesAtMergePoints(std::vector<InitialState> &initialStates) const
{
    initialStates = m_initStates;
}

void FeetInterpolator::getMergePoints(std::vector<size_t> &mergePoints) const
{
    mergePoints = m_mergePoints;
}


