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
#include <mutex>

#include <iDynTree/Core/EigenHelpers.h>

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

    double switchPercentage = 0.5, dT = 0.01, endSwitch = 0.0, initTime = 0.0;
    double nominalSwitchTime = 1.0;
    double maxStepTime = 10.0, nominalStepTime = 2.0;
    bool pauseActive = true;
    double mergePointRatioBegin = 0.5;
    double mergePointRatioEnd = 0.5;


    std::shared_ptr<FeetCubicSplineGenerator> feetSplineGenerator = nullptr;
    std::shared_ptr<FeetMinimumJerkGenerator> feetMinimumJerkGenerator = nullptr;
    std::shared_ptr<ZMPTrajectoryGenerator> zmpGenerator = nullptr;
    std::shared_ptr<CoMHeightTrajectoryGenerator> comHeightGenerator = nullptr;
    std::shared_ptr<DCMTrajectoryGenerator> dcmTrajectoryGenerator = nullptr;

    std::mutex mutex;


    bool orderSteps(const FootPrint &lFootPrint, const FootPrint &rFootPrint) {
        orderedSteps.clear();
        orderedSteps.reserve(lFootPrint.numberOfSteps() + rFootPrint.numberOfSteps());
        if (orderedSteps.capacity() > 0){

            auto& lSteps = lFootPrint.getSteps();
            auto& rSteps = rFootPrint.getSteps();

            for (StepsIndex footL = lSteps.cbegin(); footL != lSteps.cend(); ++footL)
                orderedSteps.push_back(&*footL);
            for (StepsIndex footR = rSteps.cbegin(); footR != rSteps.cend(); ++footR)
                orderedSteps.push_back(&*footR);

            std::sort(orderedSteps.begin(), orderedSteps.end(),
                      [](const Step *a, const Step *b) { return a->impactTime < b->impactTime;});
            auto duplicate = std::adjacent_find(orderedSteps.begin() + 2, orderedSteps.end(),
                                                [](const Step *a, const Step *b) { return a->impactTime == b->impactTime;});

            if (duplicate != orderedSteps.end()){
                std::cerr << "[ERROR][UnicycleGenerator::orderSteps] Two entries of the FootPrints pointers have the same impactTime. (The head is not considered)"
                          << std::endl;
                return false;
            }

            if ((orderedSteps.size() > 2) && (orderedSteps[0]->impactTime == orderedSteps[1]->impactTime) && (orderedSteps[1]->footName == orderedSteps[2]->footName)){ //preserve the alternation of ordered steps when the initial steps of the two feet have the same impact time
                const Step* buffer = orderedSteps[0];
                orderedSteps[0] = orderedSteps[1];
                orderedSteps[1] = buffer;
            }
        }
        return true;
    }

    bool createPhasesTimings(const FootPrint &lFootPrint, const FootPrint &rFootPrint) {
        //NOTE this method must be called after orderSteps to work properly

        lFootPhases = std::make_shared<std::vector<StepPhase>>();
        rFootPhases = std::make_shared<std::vector<StepPhase>>();

        phaseShift.clear();
        phaseShift.push_back(0); //necessary, otherwise I cannot call m_phaseShift.back() later

        mergePoints.clear();
        mergePoints.push_back(0); //attach a completely new trajectory

        std::shared_ptr<std::vector<StepPhase> > swing, stance;

        if (orderedSteps.size() == 2){
            size_t endSwitchSamples = static_cast<size_t>(std::round(endSwitch/dT)); //last shift to the center

            lFootPhases->reserve(endSwitchSamples);
            rFootPhases->reserve(endSwitchSamples);

            swing = (lFootPrint.getSteps().front().impactTime > rFootPrint.getSteps().front().impactTime) ? lFootPhases : rFootPhases;
            stance = (lFootPrint.getSteps().front().impactTime > rFootPrint.getSteps().front().impactTime) ? rFootPhases : lFootPhases;

            swing->insert(swing->end(), endSwitchSamples, StepPhase::SwitchIn);
            stance->insert(stance->end(), endSwitchSamples, StepPhase::SwitchOut);
            phaseShift.push_back(endSwitchSamples);

            for (size_t m = 1; m < endSwitchSamples; ++m) //Starting from 1 because 0 has been already added
                mergePoints.push_back(m);

            return true;
        }

        double totalTime = orderedSteps.back()->impactTime - initTime + endSwitch;
        size_t trajectoryDimension = static_cast<size_t>(std::ceil(totalTime/dT));

        lFootPhases->reserve(trajectoryDimension); // Notice that this dimension may not be the final dimension, due to rounding errors!!!
        rFootPhases->reserve(trajectoryDimension);

        double stepTime, switchTime, pauseTime;
        size_t stepSamples, switchSamples, swingSamples;

        const StepList& leftList = lFootPrint.getSteps();
        const StepList& rightList = rFootPrint.getSteps();
        size_t orderedStepIndex = 2, leftIndex = 1, rightIndex = 1;
        const Step* nextStep;
        double previouStepTime = initTime;

        while (orderedStepIndex < orderedSteps.size()){
            nextStep = orderedSteps[orderedStepIndex];
            stepTime = nextStep->impactTime - previouStepTime;

            if (stepTime < 0){
                std::cerr <<"[ERROR][UnicycleGenerator::createPhasesTimings] Something went wrong. The stepTime appears to be negative." << std::endl;
                return false;
            }

            if ((orderedStepIndex == 2) && (lFootPrint.getSteps().front().impactTime != rFootPrint.getSteps().front().impactTime)) { //first half step
                //Timings
                switchTime = (switchPercentage/(1 - (switchPercentage/2.0)) * stepTime)/2.0; //half switch
            } else { //general case
                switchTime = switchPercentage * stepTime; //full switch
            }

            bool pause = pauseActive && (stepTime > maxStepTime); //if true, it will pause in the middle
            if (pause){
                pauseTime = stepTime - nominalStepTime;
                switchTime = nominalSwitchTime + pauseTime;
            }

            //Samples
            stepSamples = static_cast<size_t>(std::round(stepTime/dT));
            switchSamples = static_cast<size_t>(std::round(switchTime/dT));
            swingSamples = stepSamples - switchSamples;

            if (&(leftList[leftIndex]) == nextStep){
                swing = lFootPhases;
                stance = rFootPhases;
                leftIndex++;
            } else if (&(rightList[rightIndex]) == nextStep){
                swing = rFootPhases;
                stance = lFootPhases;
                rightIndex++;
            } else {
                std::cerr << "[ERROR][UnicycleGenerator::createPhasesTimings] Something went wrong." << std::endl;
                return false;
            }

            swing->insert(swing->end(), switchSamples, StepPhase::SwitchOut); //insert the value "StepPhase::SwitchOut" switchSamples times
            stance->insert(stance->end(), switchSamples, StepPhase::SwitchIn);
            phaseShift.push_back(phaseShift.back() + switchSamples); //it stores the indeces when a change of phase occurs

            if (orderedStepIndex != 2){ //add no merge point in the first half switch
                size_t initialMergePoint, finalMergePoint;
                if (pause){
                    initialMergePoint = phaseShift.back() - static_cast<size_t>(std::round(nominalSwitchTime * (1 - mergePointRatioBegin)/(dT)));
                    finalMergePoint = phaseShift.back() - static_cast<size_t>(std::round(nominalSwitchTime * (1 - mergePointRatioEnd)/(dT)));
                } else {
                    initialMergePoint = phaseShift.back() - static_cast<size_t>(std::round(switchTime * (1 - mergePointRatioBegin)/(dT)));
                    finalMergePoint = phaseShift.back() - static_cast<size_t>(std::round(switchTime * (1 - mergePointRatioEnd)/(dT)));
                }

                for (size_t m = initialMergePoint; m <= finalMergePoint; ++m)
                    mergePoints.push_back(m);
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

    Step editStepFromMeasured(const Step& input, const Step& measured)
    {
        Step output = input;
        output.impactTime = measured.impactTime;
        output.position = measured.position;
        iDynTree::Rotation initialRotation = iDynTree::Rotation::RotZ(input.angle);
        iDynTree::Rotation measuredRotation = iDynTree::Rotation::RotZ(measured.angle);
        output.angle = input.angle + (initialRotation.inverse() * measuredRotation).asRPY()(2);
        return output;
    }

    enum class CorrectionType {
        Left,
        Right,
        Both,
        None
    };

    bool computeNewStepsFromMeasuredSteps(double initTime, double dT, double endTime, const Step& measuredLeft, const Step& measuredRight, CorrectionType correction)
    {
        Step previousL, previousR;

        if (!(leftFootPrint->keepOnlyPresentStep(initTime))) {
            std::cerr << "[UnicycleGenerator::reGenerate] The initTime is not compatible with previous runs. Call a method generate instead." << std::endl;
            return false;
        }

        if (!(rightFootPrint->keepOnlyPresentStep(initTime))) {
            std::cerr << "[UnicycleGenerator::reGenerate] The initTime is not compatible with previous runs. Call a method generate instead." << std::endl;
            return false;
        }

        bool shouldResetTime = dcmTrajectoryGenerator && iDynTree::toEigen(dcmTrajectoryGenerator->getDCMInitialState().initialVelocity).norm() < 1e-5;
        bool correctSteps = correction != CorrectionType::None || shouldResetTime;
        if (correctSteps)
        {
            leftFootPrint->getLastStep(previousL);
            rightFootPrint->getLastStep(previousR);
            bool editLeft = correction == CorrectionType::Left || correction == CorrectionType::Both ;
            bool editRight = correction == CorrectionType::Right || correction == CorrectionType::Both;
            Step editedStepLeft = editLeft? editStepFromMeasured(previousL, measuredLeft) : previousL;
            Step editedStepRight = editRight? editStepFromMeasured(previousR, measuredRight) : previousR;

            // If the input impact times are greater than the init time, it means that they are not valid.
            // At the same time, if the correction type is None, they are already equal to the previous steps.
            if (editedStepLeft.impactTime > initTime || editedStepRight.impactTime > initTime)
            {
                editedStepLeft.impactTime = previousL.impactTime;
                editedStepRight.impactTime = previousR.impactTime;
            }

            if (shouldResetTime)
            {
                double newTime = std::max(previousL.impactTime, previousR.impactTime);
                editedStepLeft.impactTime = newTime;
                editedStepRight.impactTime = newTime;
                editLeft = true;
                editRight = true;
            }

            if (editLeft) {
                leftFootPrint->clearSteps();
                if (!leftFootPrint->addStep(editedStepLeft)) {
                    std::cerr << "[UnicycleGenerator::reGenerate] The measuredLeft step is invalid." << std::endl;
                    return false;
                }
            }

            if (editRight) {
                rightFootPrint->clearSteps();
                if (!rightFootPrint->addStep(editedStepRight)) {
                    std::cerr << "[UnicycleGenerator::reGenerate] The measuredRight step is invalid." << std::endl;
                    return false;
                }
            }
        }

        if (!(planner->computeNewSteps(leftFootPrint, rightFootPrint, initTime, endTime))) {
            std::cerr << "[UnicycleGenerator::reGenerate] Unicycle planner failed to compute new steps." << std::endl;
            return false;
        }

        if (correctSteps) {
            if (zmpGenerator) {
                if (!(zmpGenerator->setPreviousSteps(previousL, previousR))) {
                    std::cerr << "[UnicycleGenerator::reGenerate] Failed to set the previous steps to the ZMP trajectory generator." << std::endl;
                    return false;
                }
            }
        }

        return true;
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
}

std::shared_ptr<UnicyclePlanner> UnicycleGenerator::unicyclePlanner()
{
    return m_pimpl->planner;
}

std::shared_ptr<FootPrint> UnicycleGenerator::getLeftFootPrint()
{
    return m_pimpl->leftFootPrint;
}

std::shared_ptr<FootPrint> UnicycleGenerator::getRightFootPrint()
{
    return m_pimpl->rightFootPrint;
}

bool UnicycleGenerator::generateFromFootPrints(std::shared_ptr<FootPrint> left, std::shared_ptr<FootPrint> right, double initTime, double dT)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    m_pimpl->leftFootPrint = left;

    m_pimpl->rightFootPrint = right;

    if (left->numberOfSteps() < 1){
        std::cerr << "[UnicycleGenerator::generate] No steps in the left pointer." << std::endl;
        return false;
    }

    if (right->numberOfSteps() < 1){
        std::cerr << "[UnicycleGenerator::generate] No steps in the right pointer." << std::endl;
        return false;
    }

    if (left->getFootName() == right->getFootName()) {
        std::cerr << "[UnicycleGenerator::generate] The left and right footprints are expected to have different name." << std::endl;
        return false;
    }

    if (dT <= 0){
        std::cerr << "[UnicycleGenerator::generate] The dT is supposed to be positive." << std::endl;
        return false;
    }

    double startLeft = left->getSteps().front().impactTime;
    double startRight = right->getSteps().front().impactTime;

    if (initTime < std::max(startLeft, startRight)){
        std::cerr << "[UnicycleGenerator::generate] The initTime must be greater or equal than the maximum of the first impactTime of the two feet."
                  << std::endl;
        return false;
    }

    m_pimpl->nominalSwitchTime = m_pimpl->switchPercentage * m_pimpl->nominalStepTime;

    m_pimpl->dT = dT;
    m_pimpl->initTime = initTime;

    if (!(m_pimpl->orderSteps(*left, *right))){
        std::cerr << "[UnicycleGenerator::generate] Failed while ordering the steps." << std::endl;
        return false;
    }

    if (!(m_pimpl->createPhasesTimings(*left, *right))){
        std::cerr << "[UnicycleGenerator::generate] Failed while creating the standing periods." << std::endl;
        return false;
    }

    m_pimpl->fillFeetStandingPeriodsVectors();

    m_pimpl->fillLeftFixedVector();

    if (m_pimpl->feetSplineGenerator) {
        if (!(m_pimpl->feetSplineGenerator->computeNewTrajectories(dT, *left, *right,*(m_pimpl->lFootPhases), *(m_pimpl->rFootPhases),
                                                                   m_pimpl->phaseShift))) {
            std::cerr << "[UnicycleGenerator::generate] Failed while computing new feet trajectories." << std::endl;
            return false;
        }
    }

    if (m_pimpl->feetMinimumJerkGenerator) {
        if (!(m_pimpl->feetMinimumJerkGenerator->computeNewTrajectories(dT, *left, *right,*(m_pimpl->lFootPhases), *(m_pimpl->rFootPhases),
                                                                        m_pimpl->phaseShift))) {
            std::cerr << "[UnicycleGenerator::generate] Failed while computing new feet trajectories (minimum jerk)." << std::endl;
            return false;
        }
    }

    if (m_pimpl->zmpGenerator) {
        if (!(m_pimpl->zmpGenerator->computeNewTrajectories(initTime, dT, m_pimpl->switchPercentage, m_pimpl->maxStepTime,
                                                            m_pimpl->nominalStepTime, m_pimpl->pauseActive, m_pimpl->mergePoints,
                                                            *left, *right, m_pimpl->orderedSteps, *(m_pimpl->lFootPhases),
                                                            *(m_pimpl->rFootPhases), m_pimpl->phaseShift))) {
            std::cerr << "[UnicycleGenerator::generate] Failed while computing new ZMP trajectories." << std::endl;
            return false;
        }
    }

    if (m_pimpl->dcmTrajectoryGenerator) {
        if (!(m_pimpl->dcmTrajectoryGenerator->computeNewTrajectories(initTime, dT, m_pimpl->switchPercentage, m_pimpl->maxStepTime, m_pimpl->nominalStepTime,
                                                                      m_pimpl->pauseActive, m_pimpl->orderedSteps, m_pimpl->phaseShift, *(m_pimpl->lFootPhases),
                                                                      *left, *right))) {
            std::cerr << "[UnicycleGenerator::generate] Failed while computing new DCM trajectories." << std::endl;
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
    {
        std::lock_guard<std::mutex> guard(m_pimpl->mutex);

        if (!(m_pimpl->planner->computeNewSteps(m_pimpl->leftFootPrint, m_pimpl->rightFootPrint, initTime, endTime))) {
            std::cerr << "[UnicycleGenerator::generate] Failed to compute new steps." << std::endl;
            return false;
        }
    }
    return generateFromFootPrints(m_pimpl->leftFootPrint, m_pimpl->rightFootPrint, initTime, dT);
}

bool UnicycleGenerator::reGenerate(double initTime, double dT, double endTime)
{
    {
        std::lock_guard<std::mutex> guard(m_pimpl->mutex);

        if (!m_pimpl->computeNewStepsFromMeasuredSteps(initTime, dT, endTime, Step(), Step(), UnicycleGeneratorImplementation::CorrectionType::None))
        {
            return false;
        }
    }

    return generateFromFootPrints(m_pimpl->leftFootPrint, m_pimpl->rightFootPrint, initTime, dT);
}

bool UnicycleGenerator::reGenerate(double initTime, double dT, double endTime, const Step &measuredLeft, const Step &measuredRight)
{
    {
        std::lock_guard<std::mutex> guard(m_pimpl->mutex);

        if (!m_pimpl->computeNewStepsFromMeasuredSteps(initTime, dT, endTime, measuredLeft, measuredRight, UnicycleGeneratorImplementation::CorrectionType::Both))
        {
            return false;
        }
    }

    return generateFromFootPrints(m_pimpl->leftFootPrint, m_pimpl->rightFootPrint, initTime, dT);
}

bool UnicycleGenerator::reGenerate(double initTime, double dT, double endTime, bool correctLeft, const iDynTree::Vector2 &measuredPosition, double measuredAngle)
{
    {
        std::lock_guard<std::mutex> guard(m_pimpl->mutex);

        Step correctedLeft, correctedRight;
        correctedLeft.angle = measuredAngle;
        correctedLeft.position = measuredPosition;
        correctedLeft.impactTime = initTime + 1.0; //to notify that the impact time is not valid
        correctedLeft.footName = m_pimpl->leftFootPrint->getFootName();

        correctedRight.angle = measuredAngle;
        correctedRight.position = measuredPosition;
        correctedRight.impactTime = initTime + 1.0; //to notify that the impact time is not valid
        correctedRight.footName = m_pimpl->rightFootPrint->getFootName();

        UnicycleGeneratorImplementation::CorrectionType correction = correctLeft ? UnicycleGeneratorImplementation::CorrectionType::Left : UnicycleGeneratorImplementation::CorrectionType::Right;

        if (!m_pimpl->computeNewStepsFromMeasuredSteps(initTime, dT, endTime, correctedLeft, correctedRight, correction))
        {
            return false;
        }
    }

    return generateFromFootPrints(m_pimpl->leftFootPrint, m_pimpl->rightFootPrint, initTime, dT);
}

bool UnicycleGenerator::reGenerate(double initTime, double dT, double endTime, const iDynTree::Vector2 &measuredLeftPosition, double measuredLeftAngle, const iDynTree::Vector2 &measuredRightPosition, double measuredRightAngle)
{
    {
        std::lock_guard<std::mutex> guard(m_pimpl->mutex);

        Step correctedLeft, correctedRight;
        correctedLeft.angle = measuredLeftAngle;
        correctedLeft.position = measuredLeftPosition;
        correctedLeft.impactTime = initTime + 1.0; //to notify that the impact time is not valid
        correctedLeft.footName = m_pimpl->leftFootPrint->getFootName();
        correctedRight.angle = measuredRightAngle;
        correctedRight.position = measuredRightPosition;
        correctedRight.impactTime = initTime + 1.0; //to notify that the impact time is not valid
        correctedRight.footName = m_pimpl->rightFootPrint->getFootName();

        if (!m_pimpl->computeNewStepsFromMeasuredSteps(initTime, dT, endTime, correctedLeft, correctedRight, UnicycleGeneratorImplementation::CorrectionType::Both))
        {
            return false;
        }
    }

    return generateFromFootPrints(m_pimpl->leftFootPrint, m_pimpl->rightFootPrint, initTime, dT);
}

bool UnicycleGenerator::setSwitchOverSwingRatio(double ratio)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (ratio <= 0){
        std::cerr << "[UnicycleGenerator::setSwitchOverSwingRatio] The ratio is supposed to be positive." << std::endl;
        return false;
    }

    m_pimpl->switchPercentage = ratio/(1.0 + ratio);
    return true;
}

bool UnicycleGenerator::setTerminalHalfSwitchTime(double lastHalfSwitchTime)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (lastHalfSwitchTime < 0){
        std::cerr << "[UnicycleGenerator::setTerminalHalfSwitchTime] The lastHalfSwitchTime cannot be negative." << std::endl;
        return false;
    }

    m_pimpl->endSwitch = lastHalfSwitchTime;
    return true;
}

void UnicycleGenerator::setPauseActive(bool isPauseActive)
{
    m_pimpl->pauseActive = isPauseActive;
}

bool UnicycleGenerator::setPauseConditions(double maxStepTime, double nominalStepTime)
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (maxStepTime < 0){
        std::cerr << "[UnicycleGenerator::setPauseConditions] If the maxStepTime is negative, the robot won't pause in middle stance." << std::endl;
        return false;
    }

    if (nominalStepTime <= 0){
        std::cerr << "[UnicycleGenerator::setPauseConditions] The nominalStepTime is supposed to be positive." << std::endl;
        return false;
    }

    if ((nominalStepTime) > maxStepTime){
        std::cerr << "[UnicycleGenerator::setPauseConditions] The nominalSwitchTime cannot be greater than maxSwitchTime." << std::endl;
        return false;
    }

    m_pimpl->maxStepTime = maxStepTime;
    m_pimpl->nominalStepTime = nominalStepTime;

    return true;
}

bool UnicycleGenerator::setMergePointRatio(double mergePointRatio)
{
    return setMergePointRatio(mergePointRatio, mergePointRatio);
}

bool UnicycleGenerator::setMergePointRatio(double mergePointRatioBegin, double mergePointRatioEnd)
{
    if(mergePointRatioBegin < 0 || mergePointRatioBegin > 1){
        std::cerr << "[UnicycleGenerator::setMergePointRatio] The mergePointRatioBegin has to be in the range [0, 1].";
        return false;
    }
    if(mergePointRatioEnd < 0 || mergePointRatioEnd > 1){
        std::cerr << "[UnicycleGenerator::setMergePointRatio] The mergePointRatioEnd has to be in the range [0, 1].";
        return false;
    }

    if (mergePointRatioBegin > mergePointRatioEnd)
    {
        std::cerr << "[UnicycleGenerator::setMergePointRatio] The mergePointRatioEnd has to be greater or equal than mergePointRatioBegin.";
        return false;
    }

    m_pimpl->mergePointRatioBegin = mergePointRatioBegin;
    m_pimpl->mergePointRatioEnd = mergePointRatioEnd;

    return true;
}

void UnicycleGenerator::getStepPhases(std::vector<StepPhase> &leftPhases, std::vector<StepPhase> &rightPhases) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    leftPhases = *(m_pimpl->lFootPhases);
    rightPhases = *(m_pimpl->rFootPhases);
}

void UnicycleGenerator::getFeetStandingPeriods(std::vector<bool> &lFootContacts, std::vector<bool> &rFootContacts) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    lFootContacts = m_pimpl->lFootContact;
    rFootContacts = m_pimpl->rFootContact;
}

void UnicycleGenerator::getWhenUseLeftAsFixed(std::vector<bool> &leftIsFixed) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    leftIsFixed = m_pimpl->leftFixed;

}

void UnicycleGenerator::getMergePoints(std::vector<size_t> &mergePoints) const
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    mergePoints = m_pimpl->mergePoints;
}

void UnicycleGenerator::disablePauseConditions()
{
    m_pimpl->pauseActive = false;
}

std::shared_ptr<FeetCubicSplineGenerator> UnicycleGenerator::addFeetCubicSplineGenerator()
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (m_pimpl->feetSplineGenerator == nullptr) {
        m_pimpl->feetSplineGenerator.reset(new FeetCubicSplineGenerator());
    }

    return m_pimpl->feetSplineGenerator;

}

std::shared_ptr<FeetMinimumJerkGenerator> UnicycleGenerator::addFeetMinimumJerkGenerator()
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (m_pimpl->feetMinimumJerkGenerator == nullptr) {
        m_pimpl->feetMinimumJerkGenerator.reset(new FeetMinimumJerkGenerator());
    }

    return m_pimpl->feetMinimumJerkGenerator;
}

std::shared_ptr<ZMPTrajectoryGenerator> UnicycleGenerator::addZMPTrajectoryGenerator()
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (m_pimpl->zmpGenerator == nullptr) {
        m_pimpl->zmpGenerator.reset(new ZMPTrajectoryGenerator());
    }

    return m_pimpl->zmpGenerator;
}

std::shared_ptr<CoMHeightTrajectoryGenerator> UnicycleGenerator::addCoMHeightTrajectoryGenerator()
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (m_pimpl->comHeightGenerator == nullptr) {
        m_pimpl->comHeightGenerator.reset(new CoMHeightTrajectoryGenerator());
    }

    return m_pimpl->comHeightGenerator;
}

std::shared_ptr<DCMTrajectoryGenerator> UnicycleGenerator::addDCMTrajectoryGenerator()
{
    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (m_pimpl->dcmTrajectoryGenerator == nullptr) {
        m_pimpl->dcmTrajectoryGenerator.reset(new DCMTrajectoryGenerator());
    }

    return m_pimpl->dcmTrajectoryGenerator;
}

bool UnicycleGenerator::setNavigationPath(const std::vector<UnicycleState> &path)
{
    if (!m_pimpl->planner->setNavigationPath(path))
    {
        std::cerr << "[UnicycleGenerator::setNavigationPath] Unable to set navigation path." << std::endl;
        return false;
    }
    return true;
}
