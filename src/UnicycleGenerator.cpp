/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <UnicycleGenerator.h>
#include <StepPhase.h>

#include <vector>
#include <cassert>
#include <algorithm>
#include <cstddef>

typedef StepList::const_iterator StepsIndex;

class UnicycleGenerator::UnicycleGeneratorImplementation {
public:
    std::shared_ptr<UnicyclePlanner> planner;
    std::shared_ptr<FootPrint> leftFootPrint, rightFootPrint;

    std::vector<const Step*> orderedSteps;
    std::shared_ptr<std::vector<StepPhase>> lFootPhases, rFootPhases;
    std::vector<size_t> phaseShift; //it stores the indeces when a change of phase occurs. The last element is the dimension of m_lFootPhases. It is common to both feet.
    std::vector<size_t> mergePoints; //it stores the indeces from which is convenient to merge a new trajectory. The last element is the dimension of m_lFootPhases, i.e. merge after the end
    std::vector<bool> lFootContact, rFootContact, leftFixed;

    double switchPercentage = -1.0, dT = 0.01, endSwitch = 0.0, initTime = 0.0;
    double nominalSwitchTime = 1.0;
    double maxStepTime = 10.0, nominalStepTime = 2.0;
    bool pauseActive = false;


    std::shared_ptr<FeetCubicSplineGenerator> feetSplineGenerator = nullptr;
    std::shared_ptr<ZMPTrajectoryGenerator> zmpGenerator = nullptr;
    std::shared_ptr<CoMHeightTrajectoryGenerator> comHeightGenerator = nullptr;


    bool orderSteps(const FootPrint &leftFootPrint, const FootPrint &rightFootPrint) {
        orderedSteps.clear();
        orderedSteps.reserve(leftFootPrint.numberOfSteps() + rightFootPrint.numberOfSteps());
        if (orderedSteps.capacity() > 0){
            for (StepsIndex footL = leftFootPrint.getSteps().cbegin(); footL != leftFootPrint.getSteps().cend(); ++footL)
                orderedSteps.push_back(&*footL);
            for (StepsIndex footR = rightFootPrint.getSteps().cbegin(); footR != rightFootPrint.getSteps().cend(); ++footR)
                orderedSteps.push_back(&*footR);

            std::sort(orderedSteps.begin(), orderedSteps.end(),
                      [](const Step *a, const Step *b) { return a->impactTime < b->impactTime;});
            auto duplicate = std::adjacent_find(orderedSteps.begin() + 2, orderedSteps.end(),
                                                [](const Step *a, const Step *b) { return a->impactTime == b->impactTime;});

            if (duplicate != orderedSteps.end()){
                std::cerr << "[FEETINTERPOLATOR] Two entries of the FootPrints pointers have the same impactTime. (The head is not considered)"
                          << std::endl;
                return false;
            }
        }
        return true;
    }

    bool createPhasesTimings(const FootPrint &leftFootPrint, const FootPrint &rightFootPrint) {
        //NOTE this method must be called after orderSteps to work properly

        lFootPhases.reset(new std::vector<StepPhase>());
        rFootPhases.reset(new std::vector<StepPhase>());

        phaseShift.clear();
        phaseShift.push_back(0); //necessary, otherwise I cannot call m_phaseShift.back() later

        mergePoints.clear();
        mergePoints.push_back(0); //attach a completely new trajectory

        std::shared_ptr<std::vector<StepPhase> > swing, stance;

        if (orderedSteps.size() == 2){
            size_t endSwitchSamples = static_cast<size_t>(std::round(endSwitch/dT)); //last shift to the center

            lFootPhases->reserve(endSwitchSamples);
            rFootPhases->reserve(endSwitchSamples);

            swing = (leftFootPrint.getSteps().front().impactTime > rightFootPrint.getSteps().front().impactTime) ? lFootPhases : rFootPhases;
            stance = (leftFootPrint.getSteps().front().impactTime > rightFootPrint.getSteps().front().impactTime) ? rFootPhases : lFootPhases;

            swing->insert(swing->end(), endSwitchSamples, StepPhase::SwitchIn);
            stance->insert(stance->end(), endSwitchSamples, StepPhase::SwitchOut);
            phaseShift.push_back(endSwitchSamples);
            mergePoints.push_back(endSwitchSamples - 1);

            return true;
        }

        double totalTime = orderedSteps.back()->impactTime - initTime + endSwitch;
        size_t trajectoryDimension = static_cast<size_t>(std::ceil(totalTime/dT));

        lFootPhases->reserve(trajectoryDimension); // Notice that this dimension may not be the final dimension, due to rounding errors!!!
        rFootPhases->reserve(trajectoryDimension);

        double stepTime, switchTime, pauseTime;
        size_t stepSamples, switchSamples, swingSamples;

        const Step* leftIndex = &*(leftFootPrint.getSteps().cbegin() + 1);
        const Step* rightIndex = &*(rightFootPrint.getSteps().cbegin() + 1);
        size_t orderedStepIndex = 2;
        const Step* nextStepindex;
        double previouStepTime = initTime;

        while (orderedStepIndex < orderedSteps.size()){
            nextStepindex = orderedSteps[orderedStepIndex];
            stepTime = nextStepindex->impactTime - previouStepTime;

            if (stepTime < 0){
                std::cerr <<"Something went wrong. The stepTime appears to be negative." << std::endl;
                return false;
            }

            if ((nextStepindex == orderedSteps.front()) && (leftFootPrint.getSteps().front().impactTime != rightFootPrint.getSteps().front().impactTime)) { //first half step
                //Timings
                switchTime = (switchPercentage/(1 - (switchPercentage/2.0)) * stepTime)/2.0; //half switch
            } else { //general case
                switchTime = switchPercentage * stepTime; //full switch
            }

            bool pause = pauseActive && (stepTime > maxStepTime); //if true, it will pause in the middle
            if (pause){
                pauseTime = stepTime - nominalStepTime;
                switchTime = nominalSwitchTime + pauseTime;
            } else pauseTime = 0;

            //Samples
            stepSamples = static_cast<size_t>(std::round(stepTime/dT));
            switchSamples = static_cast<size_t>(std::round(switchTime/dT));
            swingSamples = stepSamples - switchSamples;

            if (leftIndex == nextStepindex){
                swing = lFootPhases;
                stance = rFootPhases;
                leftIndex++;
            } else if (rightIndex == nextStepindex){
                swing = rFootPhases;
                stance = lFootPhases;
                rightIndex++;
            } else {
                std::cerr << "[FEETINTERPOLATOR] Something went wrong." << std::endl;
                return false;
            }

            swing->insert(swing->end(), switchSamples, StepPhase::SwitchOut); //insert the value "StepPhase::SwitchOut" switchSamples times
            stance->insert(stance->end(), switchSamples, StepPhase::SwitchIn);
            phaseShift.push_back(phaseShift.back() + switchSamples); //it stores the indeces when a change of phase occurs

            if (nextStepindex != orderedSteps.front()){ //add no merge point in the first half switch
                //bool pause = m_pauseActive && (switchTime > m_maxSwitchTime); //if true, it will pause in the middle
                size_t mergePoint;
                if (pause){
                    mergePoint = phaseShift.back() - static_cast<size_t>(std::round(nominalSwitchTime/(2*dT)));
                    mergePoints.push_back(mergePoint);
                } else {
                    mergePoint = phaseShift.back() - static_cast<size_t>(std::round(switchTime/(2*dT)));
                    mergePoints.push_back(mergePoint);
                }
            }

            swing->insert(swing->end(), swingSamples, StepPhase::Swing); //first step
            stance->insert(stance->end(), swingSamples, StepPhase::Stance);
            phaseShift.push_back(phaseShift.back() + swingSamples);

            previouStepTime += stepSamples*dT; //to take into account samples lost by numeric errors
            orderedStepIndex++;
        }
        switchSamples = static_cast<size_t>(std::round(endSwitch/dT)); //last shift to the center
        swing->insert(swing->end(), switchSamples, StepPhase::SwitchIn);
        stance->insert(stance->end(), switchSamples, StepPhase::SwitchOut);
        phaseShift.push_back(phaseShift.back() + switchSamples);

        mergePoints.push_back(phaseShift.back() - 1); //merge on the last

        lFootPhases->shrink_to_fit();
        rFootPhases->shrink_to_fit();

        return true;
    }

    void fillFeetStandingPeriodsVectors() {
        //NOTE this must be called after createPhasesTimings
        lFootContact.resize(lFootPhases->size());
        rFootContact.resize(rFootPhases->size());

        for (size_t instant = 0; instant < lFootContact.size(); ++instant){
            if (lFootPhases->at(instant) == StepPhase::Swing)
                lFootContact[instant] = false;
            else lFootContact[instant] = true;
        }

        for (size_t instant = 0; instant < rFootContact.size(); ++instant){
            if (rFootPhases->at(instant) == StepPhase::Swing)
                rFootContact[instant] = false;
            else rFootContact[instant] = true;
        }
    }


    void fillLeftFixedVector() {
        //NOTE this must be called after createPhasesTimings
        leftFixed.resize(lFootPhases->size());

        for (size_t instant = 0; instant < leftFixed.size(); ++instant){
            leftFixed[instant] = (lFootPhases->at(instant) == StepPhase::Stance)||(lFootPhases->at(instant) == StepPhase::SwitchOut);
        }
    }


};

UnicycleGenerator::UnicycleGenerator()
    : m_pimpl(new UnicycleGeneratorImplementation)
{
    assert(m_pimpl);
    m_pimpl->planner = std::make_shared<UnicyclePlanner>();

    m_pimpl->leftFootPrint = std::make_shared<FootPrint>();
    m_pimpl->leftFootPrint->setFootName("left");

    m_pimpl->rightFootPrint = std::make_shared<FootPrint>();
    m_pimpl->rightFootPrint->setFootName("right");
}

UnicycleGenerator::~UnicycleGenerator()
{
    if (m_pimpl) {
        delete m_pimpl;
        m_pimpl = nullptr;
    }
}

std::shared_ptr<UnicyclePlanner> UnicycleGenerator::unicyclePlanner()
{
    return m_pimpl->planner;
}

bool UnicycleGenerator::generate(const FootPrint &left, const FootPrint &right, double initTime, double dT)
{
    if (left.numberOfSteps() < 1){
        std::cerr << "[UnicycleGenerator::generate] No steps in the left pointer." << std::endl;
        return false;
    }

    if (right.numberOfSteps() < 1){
        std::cerr << "[UnicycleGenerator::generate] No steps in the right pointer." << std::endl;
        return false;
    }

    if (left.getFootName() == right.getFootName()) {
        std::cerr << "[UnicycleGenerator::generate] The left and right footprints are expected to have different name." << std::endl;
        return false;
    }

    if (dT <= 0){
        std::cerr << "[UnicycleGenerator::generate] The dT is supposed to be positive." << std::endl;
        return false;
    }

    double startLeft = left.getSteps().front().impactTime;
    double startRight = right.getSteps().front().impactTime;

    if (initTime < std::max(startLeft, startRight)){
        std::cerr << "[UnicycleGenerator::generate] The initTime must be greater or equal than the maximum of the first impactTime of the two feet."
                  << std::endl;
        return false;
    }

    if (m_pimpl->switchPercentage < 0){
        std::cerr << "[UnicycleGenerator::generate] First you have to define the ratio between switch and swing phases." << std::endl;
        return false;
    }


    m_pimpl->nominalSwitchTime = m_pimpl->switchPercentage * m_pimpl->nominalStepTime;

    m_pimpl->dT = dT;
    m_pimpl->initTime = initTime;

    if (!(m_pimpl->orderSteps(left, right))){
        std::cerr << "[UnicycleGenerator::generate] Failed while ordering the steps." << std::endl;
        return false;
    }

    if (!(m_pimpl->createPhasesTimings(left, right))){
        std::cerr << "[UnicycleGenerator::generate] Failed while creating the standing periods." << std::endl;
        return false;
    }

    m_pimpl->fillFeetStandingPeriodsVectors();

    m_pimpl->fillLeftFixedVector();

    if (m_pimpl->feetSplineGenerator) {
        if (!(m_pimpl->feetSplineGenerator->computeNewTrajectories(dT, left, right, m_pimpl->orderedSteps,
                                                                   *(m_pimpl->lFootPhases), *(m_pimpl->rFootPhases),
                                                                   m_pimpl->phaseShift))) {
            std::cerr << "[UnicycleGenerator::generate] Failed while computing new feet trajectories." << std::endl;
            return false;
        }
    }

    if (m_pimpl->zmpGenerator) {
        if (!(m_pimpl->zmpGenerator->computeNewTrajectories(initTime, dT, m_pimpl->switchPercentage, m_pimpl->maxStepTime,
                                                            m_pimpl->nominalStepTime, m_pimpl->pauseActive, m_pimpl->mergePoints,
                                                            left, right, m_pimpl->orderedSteps, *(m_pimpl->lFootPhases),
                                                            *(m_pimpl->rFootPhases), m_pimpl->phaseShift))) {
            std::cerr << "[UnicycleGenerator::generate] Failed while computing new ZMP trajectories." << std::endl;
            return false;
        }
    }

    if (m_pimpl->comHeightGenerator) {
        if (!(m_pimpl->comHeightGenerator->computeNewTrajectories(dT, *(m_pimpl->lFootPhases), m_pimpl->phaseShift))) {
            std::cerr << "[UnicycleGenerator::generate] Failed while computing new CoM height trajectory." << std::endl;
            return false;
        }
    }

    return true;
}

bool UnicycleGenerator::generate(double initTime, double dT, double endTime)
{
    m_pimpl->leftFootPrint->clearSteps();
    m_pimpl->rightFootPrint->clearSteps();

    if (!(m_pimpl->planner->setEndTime(endTime))) {
        std::cerr << "[UnicycleGenerator::generate] Failed while setting endTime." << std::endl;
        return false;
    }

    if (!(m_pimpl->planner->computeNewSteps(m_pimpl->leftFootPrint, m_pimpl->rightFootPrint, initTime))) {
        std::cerr << "[UnicycleGenerator::generate] Failed to compute new steps." << std::endl;
        return false;
    }
    return generate(*(m_pimpl->leftFootPrint), *(m_pimpl->rightFootPrint), initTime, dT);
}

bool UnicycleGenerator::setSwitchOverSwingRatio(double ratio)
{
    if (ratio <= 0){
        std::cerr << "[UnicycleGenerator::setSwitchOverSwingRatio] The ratio is supposed to be positive." << std::endl;
        return false;
    }

    m_pimpl->switchPercentage = ratio/(1.0 + ratio);
    return true;
}

bool UnicycleGenerator::setTerminalHalfSwitchTime(double lastHalfSwitchTime)
{
    if (lastHalfSwitchTime < 0){
        std::cerr << "[UnicycleGenerator::setTerminalHalfSwitchTime] The lastHalfSwitchTime cannot be negative." << std::endl;
        return false;
    }

    m_pimpl->endSwitch = lastHalfSwitchTime;
    return true;
}

bool UnicycleGenerator::setPauseConditions(double maxStepTime, double nominalStepTime)
{
    if (maxStepTime < 0){
        std::cerr << "[FEETINTERPOLATOR] If the maxStepTime is negative, the robot won't pause in middle stance." << std::endl;
        m_pimpl->pauseActive = false;
    }

    m_pimpl->pauseActive = true;
    m_pimpl->maxStepTime = maxStepTime;

    if (m_pimpl->pauseActive){
        if (nominalStepTime <= 0){
            std::cerr << "[FEETINTERPOLATOR] The nominalStepTime is supposed to be positive." << std::endl;
            m_pimpl->pauseActive = false;
            return false;
        }

        if ((nominalStepTime) > maxStepTime){
            std::cerr << "[FEETINTERPOLATOR] The nominalSwitchTime cannot be greater than maxSwitchTime." << std::endl;
            m_pimpl->pauseActive = false;
            return false;
        }
    }
    m_pimpl->nominalStepTime = nominalStepTime;

    return true;
}

void UnicycleGenerator::getFeetStandingPeriods(std::vector<bool> &lFootContacts, std::vector<bool> &rFootContacts) const
{
    lFootContacts = m_pimpl->lFootContact;
    rFootContacts = m_pimpl->rFootContact;
}

void UnicycleGenerator::getWhenUseLeftAsFixed(std::vector<bool> &leftIsFixed) const
{
    leftIsFixed = m_pimpl->leftFixed;

}

void UnicycleGenerator::getMergePoints(std::vector<size_t> &mergePoints) const
{
    mergePoints = m_pimpl->mergePoints;
}

std::shared_ptr<FeetCubicSplineGenerator> UnicycleGenerator::addFeetCubicSplineGenerator()
{
    if (m_pimpl->feetSplineGenerator == nullptr) {
        m_pimpl->feetSplineGenerator.reset(new FeetCubicSplineGenerator());
    }

    return m_pimpl->feetSplineGenerator;

}

std::shared_ptr<ZMPTrajectoryGenerator> UnicycleGenerator::addZMPTrajectoryGenerator()
{
    if (m_pimpl->zmpGenerator == nullptr) {
        m_pimpl->zmpGenerator.reset(new ZMPTrajectoryGenerator());
    }

    return m_pimpl->zmpGenerator;
}

std::shared_ptr<CoMHeightTrajectoryGenerator> UnicycleGenerator::addCoMHeightTrajectoryGenerator()
{
    if (m_pimpl->comHeightGenerator == nullptr) {
        m_pimpl->comHeightGenerator.reset(new CoMHeightTrajectoryGenerator());
    }

    return m_pimpl->comHeightGenerator;
}


