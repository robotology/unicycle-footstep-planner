/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicyclePlanner.h"
#include "iDynTree/Core/Utils.h"
#include "Eigen/Core"
#include "iDynTree/Core/EigenHelpers.h"
#include <iostream>

bool UnicyclePlanner::getInitialStateFromFeet(double initTime)
{

    UnicycleState unicycleState;
    iDynTree::Vector2 dummyVector;
    dummyVector.zero();
    unicycleState.position.zero();
    unicycleState.angle = 0;
    double initTimeFromFeet;

    if ((m_left->numberOfSteps() == 0) && (m_right->numberOfSteps() == 0)){

        if (m_currentController == UnicycleController::PERSON_FOLLOWING)
        {
            if (!m_personFollowingController->getDesiredPoint(initTime, unicycleState.position, dummyVector))
                return false;

            unicycleState.position(0) = unicycleState.position(0) - m_personFollowingController->getPersonDistance()(0);
            unicycleState.position(1) = unicycleState.position(1) - m_personFollowingController->getPersonDistance()(1);
            unicycleState.angle = 0;
        }
        else
        {
            unicycleState.position.zero();
            unicycleState.angle = 0.0;
        }

        m_left->addStepFromUnicycle(unicycleState, initTime);
        m_right->addStepFromUnicycle(unicycleState, initTime);

        m_firstStep = true;
        m_swingLeft = m_startLeft;

    } else {
        if (m_left->numberOfSteps() == 0){
            Step lastFoot;

            if (!(m_right->getLastStep(lastFoot)))
                return false;

            if (!(m_right->getUnicycleStateFromStep(lastFoot, unicycleState)))
                return false;

            initTimeFromFeet = lastFoot.impactTime;
            m_swingLeft = true;
            m_firstStep = false;

        } else if (m_right->numberOfSteps() == 0){
            Step lastFoot;

            if (!(m_left->getLastStep(lastFoot)))
                return false;

            if (!(m_left->getUnicycleStateFromStep(lastFoot, unicycleState)))
                return false;

            initTimeFromFeet = lastFoot.impactTime;
            m_swingLeft = false;
            m_firstStep = false;

        } else {
            Step lastFootL, lastFootR;

            if (!(m_left->getLastStep(lastFootL)) || !(m_right->getLastStep(lastFootR)))
                return false;

            if(lastFootL.impactTime == lastFootR.impactTime){
                if (m_startLeft){
                    if (!(m_right->getUnicycleStateFromStep(lastFootR, unicycleState)))
                        return false;

                    m_swingLeft = true;
                } else {
                    if (!(m_left->getUnicycleStateFromStep(lastFootL, unicycleState)))
                        return false;

                    m_swingLeft = false;
                }
                initTimeFromFeet = lastFootL.impactTime;
                m_firstStep = true;
            } else if (lastFootL.impactTime < lastFootR.impactTime) {
                if (!(m_right->getUnicycleStateFromStep(lastFootR, unicycleState)))
                    return false;

                m_swingLeft = true;
                m_firstStep = false;
                initTimeFromFeet = lastFootR.impactTime;
            } else {
                if (!(m_left->getUnicycleStateFromStep(lastFootL, unicycleState)))
                    return false;

                m_swingLeft = false;
                m_firstStep = false;
                initTimeFromFeet = lastFootL.impactTime;
            }
        }
        if (initTime < initTimeFromFeet){
            std::cerr << "The initTime cannot be greater then the last impactTime." << std::endl;
            return false;
        }

        TrajectoryPoint initialPoint;
        initialPoint.initTime = initTime;
        initialPoint.yDesired = m_personFollowingController->getPersonPosition(unicycleState.position, unicycleState.angle);
        dummyVector.zero();
        initialPoint.yDotDesired = dummyVector;

        if (!(m_personFollowingController->setDesiredPoint(initialPoint))){
            std::cerr <<"Failed to set a the intial reference given the provided FootPrints." <<std::endl;
            return false;
        }

        if (m_firstStep) {
            initialPoint.initTime = initTime + m_nominalTime - m_minTime;

            if (!(m_personFollowingController->setDesiredPoint(initialPoint))){
                std::cerr <<"Failed to set a the intial dummy reference to have a slower first step." <<std::endl;
                return false;
            }

            m_directController->setInactiveUntil(initTime + m_nominalTime - m_minTime);
        }
    }

    if(!(m_unicycle->setInitialState(unicycleState.position, unicycleState.angle))){
        std::cerr << "Error while setting the initial state for the integrator." << std::endl;
        return false;
    }

    //Change first stepping foot according to the desired lateral velocity
    if (m_firstStep && m_currentController == UnicycleController::DIRECT &&
            std::abs(m_directController->desiredLateralVelocity()) > 0)
    {
        //If we want to move right, use the right as first stepping foot
        m_swingLeft = m_directController->desiredLateralVelocity() > 0;
    }

    return true;
}

bool UnicyclePlanner::initializePlanner(double initTime)
{

    iDynTree::Vector2 widths;
    widths.zero();
    widths(1) = m_nominalWidth/2;
    m_left->setDistanceFromUnicycle(widths);
    widths(1) = -m_nominalWidth/2;
    m_right->setDistanceFromUnicycle(widths);

    if (!m_left->setYawOffsetInRadians(m_leftYawOffset))
        return false;

    if (!m_right->setYawOffsetInRadians(m_rightYawOffset))
        return false;

    if (!(m_left->setTinyStepLength(m_minLength)))
        return false;
    if (!(m_right->setTinyStepLength(m_minLength)))
        return false;

    if (!(m_left->setTinyStepAngle(m_minAngle)))
        return false;
    if (!(m_right->setTinyStepAngle(m_minAngle)))
        return false;

    if(!getInitialStateFromFeet(initTime)){
        std::cerr << "Error while computing the initial state." << std::endl;
        return false;
    }

    if (m_endTime <= initTime){
        std::cerr << "The endTime should be strictly greater than the specified initTime." << std::endl;
        return false;
    }

    m_integrator.clearSolution();

    if(!m_integrator.integrate(initTime, m_endTime)){
        std::cerr << "Error while integrating the unicycle dynamics." << std::endl;
        return false;
    }

    return true;

}

bool UnicyclePlanner::get_rPl(const UnicycleState &unicycleState, iDynTree::Vector2& rPl)
{
    Eigen::Matrix<double, 2, 2, Eigen::RowMajor> rTranspose(2,2);
    iDynTree::Vector2 newPosition;

    Step prevStep;
    if (m_swingLeft){
        if (!(m_right->getLastStep(prevStep)))
            return false;
        if(!(m_left->getFootPositionFromUnicycle(unicycleState, newPosition)))
            return false;

        double c_theta = std::cos(prevStep.angle);
        double s_theta = std::sin(prevStep.angle);

        rTranspose(0,0) = c_theta;
        rTranspose(1,0) = -s_theta;
        rTranspose(0,1) = s_theta;
        rTranspose(1,1) = c_theta;

        iDynTree::toEigen(rPl) =
                rTranspose*(iDynTree::toEigen(newPosition) - iDynTree::toEigen(prevStep.position));
    } else {
        if (!(m_left->getLastStep(prevStep)))
            return false;
        if(!(m_right->getFootPositionFromUnicycle(unicycleState, newPosition)))
            return false;

        double c_theta = std::cos(unicycleState.angle);
        double s_theta = std::sin(unicycleState.angle);

        rTranspose(0,0) = c_theta;
        rTranspose(1,0) = -s_theta;
        rTranspose(0,1) = s_theta;
        rTranspose(1,1) = c_theta;

        iDynTree::toEigen(rPl) =
                rTranspose*(iDynTree::toEigen(prevStep.position) - iDynTree::toEigen(newPosition));
    }
    return true;
}

bool UnicyclePlanner::get_newStepRelativePosition(const UnicycleState &unicycleState, iDynTree::Vector2 &newStepRelativePosition)
{
    Eigen::Matrix<double, 2, 2, Eigen::RowMajor> rTranspose(2,2);
    iDynTree::Vector2 newPosition;
    Step prevStep;

    if (m_swingLeft){
        if (!(m_right->getLastStep(prevStep)))
            return false;
        if(!(m_left->getFootPositionFromUnicycle(unicycleState, newPosition)))
            return false;
    } else {
        if (!(m_left->getLastStep(prevStep)))
            return false;
        if(!(m_right->getFootPositionFromUnicycle(unicycleState, newPosition)))
            return false;
    }

    double c_theta = std::cos(prevStep.angle);
    double s_theta = std::sin(prevStep.angle);

    rTranspose(0,0) = c_theta;
    rTranspose(1,0) = -s_theta;
    rTranspose(0,1) = s_theta;
    rTranspose(1,1) = c_theta;

    iDynTree::toEigen(newStepRelativePosition) =
            rTranspose*(iDynTree::toEigen(newPosition) - iDynTree::toEigen(prevStep.position));

    return true;
}

bool UnicyclePlanner::getIntegratorSolution(double time, UnicycleState &unicycleState) const
{
    auto& fullSolution = m_integrator.getFullSolution();

    assert(fullSolution.size() != 0 && "Error while retrieving the integrator solution. No solution available.");

    double initialTime = fullSolution.front().time;

    assert(time >= initialTime && time <= fullSolution.back().time && "Error while retrieving the integrator solution. Time out of bounds.");

    size_t index = static_cast<size_t>(std::round((time - initialTime)/m_integrator.maximumStepSize()));
    index = std::min(index, fullSolution.size() - 1);
    auto& state = fullSolution[index].stateAtT;

    unicycleState.position(0) = state(0);
    unicycleState.position(1) = state(1);
    unicycleState.angle = state(2);
    return true;
}

bool UnicyclePlanner::addTerminalStep(const UnicycleState &lastUnicycleState)
{
    std::shared_ptr<UnicycleFoot> stanceFoot, swingFoot;

    stanceFoot = m_swingLeft ? m_right : m_left;
    swingFoot = m_swingLeft ? m_left : m_right;

    Step prevStanceStep;
    if (!(stanceFoot->getLastStep(prevStanceStep)))
        return false;

    double prevUnicycleAngleFromStance = stanceFoot->getUnicycleAngleFromStep(prevStanceStep);

    iDynTree::Vector2 rPl, plannedPosition, relativePlannedPosition;

    if (!get_rPl(lastUnicycleState, rPl))
        return false;

    if (!get_newStepRelativePosition(lastUnicycleState, relativePlannedPosition))
        return false;

    swingFoot->getFootPositionFromUnicycle(lastUnicycleState, plannedPosition);

    double deltaAngleStance = std::abs(lastUnicycleState.angle - prevUnicycleAngleFromStance);
    double stepTime = std::max(m_initTime + m_nominalTime, prevStanceStep.impactTime + m_nominalTime);
    double deltaTime = stepTime - prevStanceStep.impactTime;

    bool isTinyForStance = stanceFoot->isTinyStep(lastUnicycleState); //two steps will be taken, first by the swing leg and then by the stance leg to make the two feet parallel.
                                                                      //Since the constraints on this second step are implicitly satisfied by the geometry of the unicycle,
                                                                      //we focus in avoiding that this second step break the "tiny step" constraint. The constraints are checked for the first step.
    bool constraintsSatisfied = m_unicycleProblem.areConstraintsSatisfied(rPl, deltaAngleStance, deltaTime, plannedPosition, relativePlannedPosition);
    bool isTinyForSwing = swingFoot->isTinyStep(lastUnicycleState);

    if (constraintsSatisfied && !isTinyForStance && !isTinyForSwing){
        if(!(swingFoot->addStepFromUnicycle(lastUnicycleState, stepTime))){
            std::cerr << "Error while adding terminal step." << std::endl;
            return false;
        }

        if(!(stanceFoot->addStepFromUnicycle(lastUnicycleState, stepTime + m_nominalTime))){
            std::cerr << "Error while adding terminal step." << std::endl;
            return false;
        }
    } else {
        if (!constraintsSatisfied){
            std::cerr << "Unable to satisfy constraints on the last step. The following constraints are not satisfied:" << std::endl;
            m_unicycleProblem.printViolatedConstraints(rPl, deltaAngleStance, deltaTime, plannedPosition, relativePlannedPosition);
        }

        UnicycleState previousUnicycleStateFromStance;

        if(!stanceFoot->getUnicycleStateFromStep(prevStanceStep, previousUnicycleStateFromStance)){
            return false;
        }
        isTinyForSwing = swingFoot->isTinyStep(previousUnicycleStateFromStance);

        if (!isTinyForSwing){
            if(!(swingFoot->addParallelStep(*(stanceFoot), stepTime))){
                std::cerr << "Error while adding terminal step." << std::endl;
                return false;
            }
        }
    }

    return true;

}

UnicyclePlanner::UnicyclePlanner()
    :m_personFollowingController(std::make_shared<PersonFollowingController>())
    ,m_directController(std::make_shared<UnicycleDirectController>())
    ,m_currentController(UnicycleController::PERSON_FOLLOWING)
    ,m_unicycle(std::make_shared<ControlledUnicycle>())
    ,m_integrator(m_unicycle)
    ,m_initTime(0.0)
    ,m_endTime(0.01)
    ,m_minTime(3)
    ,m_maxTime(10)
    ,m_nominalTime(4)
    ,m_dT(0.01)
    ,m_minAngle(iDynTree::deg2rad(5))
    ,m_nominalWidth(0.14)
    ,m_maxLength(0.2)
    ,m_maxLengthBackward(0.2)
    ,m_minLength(0.05)
    ,m_maxAngle(iDynTree::deg2rad(45))
    ,m_addTerminalStep(true)
    ,m_startLeft(true)
    ,m_resetStartingFoot(false)
    ,m_firstStep(false)
    ,m_freeSpaceMethod(FreeSpaceEllipseMethod::REFERENCE_ONLY)
    ,m_leftYawOffset(0.0)
    ,m_rightYawOffset(0.0)
    ,m_left(nullptr)
    ,m_right(nullptr)
    ,m_swingLeft(true)
    , m_linearVelocityConservativeFactor(0.9)
    , m_angularVelocityConservativeFactor(0.7)
    , m_lateralVelocityConservaiveFactor(0.2)
{
    m_unicycle->setController(m_personFollowingController);
    m_integrator.setMaximumStepSize(0.01);
    m_unicycleProblem.setMaxLength(0.20);
    m_unicycleProblem.setMinWidth(0.08);
    m_unicycleProblem.setMaxAngleVariation(iDynTree::deg2rad(45));
    m_unicycleProblem.setCostWeights(1.0, 2.5);
}

bool UnicyclePlanner::setDesiredPersonDistance(double xPosition, double yPosition)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setPersonDistance(xPosition, yPosition);
}

bool UnicyclePlanner::setControllerGain(double controllerGain)
{
    return setPersonFollowingControllerGain(controllerGain);
}

bool UnicyclePlanner::setPersonFollowingControllerGain(double controllerGain)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setGain(controllerGain);
}

bool UnicyclePlanner::setSlowWhenTurnGain(double slowWhenTurnGain)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setSlowWhenTurnGain(slowWhenTurnGain) &&
            m_directController->setSlowWhenTurnGain(slowWhenTurnGain);
}

bool UnicyclePlanner::setSlowWhenBackwardFactor(double slowWhenBackwardFactor)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setSlowWhenBackwardFactor(slowWhenBackwardFactor) &&
            m_directController->setSlowWhenBackwardFactor(slowWhenBackwardFactor);
}

bool UnicyclePlanner::setSlowWhenSidewaysFactor(double slowWhenSidewaysFactor)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setSlowWhenSidewaysFactor(slowWhenSidewaysFactor) && m_directController->setSlowWhenSidewaysFactor(slowWhenSidewaysFactor);
}

bool UnicyclePlanner::addDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2 &yDesired)
{
    return addPersonFollowingDesiredTrajectoryPoint(initTime, yDesired);
}

bool UnicyclePlanner::addPersonFollowingDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2 &yDesired)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    TrajectoryPoint newPoint;
    newPoint.yDesired = yDesired;
    newPoint.yDotDesired.zero();
    newPoint.initTime = initTime;
    return m_personFollowingController->setDesiredPoint(newPoint);
}

bool UnicyclePlanner::addDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2 &yDesired, const iDynTree::Vector2 &yDotDesired)
{
    return addPersonFollowingDesiredTrajectoryPoint(initTime, yDesired, yDotDesired);
}

bool UnicyclePlanner::addPersonFollowingDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2 &yDesired, const iDynTree::Vector2 &yDotDesired)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    TrajectoryPoint newPoint;
    newPoint.yDesired = yDesired;
    newPoint.yDotDesired = yDotDesired;
    newPoint.initTime = initTime;
    return m_personFollowingController->setDesiredPoint(newPoint);
}

void UnicyclePlanner::clearDesiredTrajectory()
{
    return clearPersonFollowingDesiredTrajectory();
}

void UnicyclePlanner::clearPersonFollowingDesiredTrajectory()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_personFollowingController->clearDesiredTrajectory();
}

bool UnicyclePlanner::clearDesiredTrajectoryUpTo(double time)
{
    return clearPersonFollowingDesiredTrajectoryUpTo(time);
}

bool UnicyclePlanner::clearPersonFollowingDesiredTrajectoryUpTo(double time)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->clearDesiredTrajectoryUpTo(time);
}

void UnicyclePlanner::setDesiredDirectControl(double forwardVelocity, double angularVelocity, double lateralVelocity)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_directController->setConstantControl(forwardVelocity, angularVelocity, lateralVelocity);
}

bool UnicyclePlanner::setSaturationsConservativeFactors(double linearVelocityConservativeFactor, double angularVelocityConservativeFactor)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (linearVelocityConservativeFactor < 0.0 || linearVelocityConservativeFactor > 1.0)
    {
        std::cerr << " The parameter linearVelocityConservativeFactor is supposed to be between 0 and 1" << std::endl;
        return false;
    }

    if (angularVelocityConservativeFactor < 0.0 || angularVelocityConservativeFactor > 1.0)
    {
        std::cerr << " The parameter angularVelocityConservativeFactor is supposed to be between 0 and 1" << std::endl;
        return false;
    }

    m_linearVelocityConservativeFactor = linearVelocityConservativeFactor;
    m_angularVelocityConservativeFactor = angularVelocityConservativeFactor;

    return true;
}

bool UnicyclePlanner::setMaximumIntegratorStepSize(double dT)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_integrator.setMaximumStepSize(dT);
}

bool UnicyclePlanner::setMaxStepLength(double maxLength)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(!m_unicycleProblem.setMaxLength(maxLength)){
        return false;
    }
    m_maxLength = maxLength;


    if(!m_unicycleProblem.setMaxLengthBackward(maxLength)){
        return false;
    }
    m_maxLengthBackward = maxLength;


    return true;
}

bool UnicyclePlanner::setMaxStepLength(double maxLength, double backwardMultiplier)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (backwardMultiplier < 0)
    {
        std::cerr << "backwardMultiplier is expected to be non-negative." << std::endl;
        return false;
    }

    if(!m_unicycleProblem.setMaxLength(maxLength)){
        return false;
    }
    m_maxLength = maxLength;

    double maxLengthBack = maxLength * backwardMultiplier;
    if(!m_unicycleProblem.setMaxLengthBackward(maxLengthBack)){
        return false;
    }
    m_maxLengthBackward = maxLengthBack;

    return true;
}


bool UnicyclePlanner::setMaxAngleVariation(double maxAngleInRad)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_unicycleProblem.setMaxAngleVariation(maxAngleInRad)){
        m_maxAngle = maxAngleInRad;
        return true;
    }
    return false;
}

bool UnicyclePlanner::setCostWeights(double positionWeight, double timeWeight)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_unicycleProblem.setCostWeights(positionWeight, timeWeight);
}

bool UnicyclePlanner::setStepTimings(double minTime, double maxTime, double nominalTime)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if ((minTime < 0) || (maxTime < 0) || (nominalTime <0)){
        std::cerr << "Timings are expected to be non-negative." << std::endl;
        return false;
    }

    if (minTime > maxTime){
        std::cerr << "The minTime needs to be greater than the maximum time." << std::endl;
        return false;
    }

    if ((nominalTime < minTime)||(maxTime < nominalTime)){
        std::cerr << "The nominal time is expected to be between minTime and maxTime." << std::endl;
        return false;
    }

    m_minTime = minTime;
    m_maxTime = maxTime;
    m_nominalTime = nominalTime;

    return true;
}

bool UnicyclePlanner::setPlannerPeriod(double dT)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (dT < 0){
        std::cerr << "The planner period is supposed to be non-negative." << std::endl;
        return false;
    }

    m_dT = dT;

    return true;
}

bool UnicyclePlanner::setMinimumAngleForNewSteps(double minAngleInRad)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (minAngleInRad < 0){
        std::cerr << "The minimum angle is supposed to be non-negative." << std::endl;
        return false;
    }

    m_minAngle = minAngleInRad;

    return true;
}

bool UnicyclePlanner::setMinimumStepLength(double minLength)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (minLength < 0){
        std::cerr << "The minimum length is supposed to be non-negative." << std::endl;
        return false;
    }
    m_minLength = minLength;
    return true;
}

bool UnicyclePlanner::setWidthSetting(double minWidth, double nominalWidth)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (nominalWidth < 0){
        std::cerr << "The nominal width is supposed to be non-negative." << std::endl;
        return false;
    }

    if (nominalWidth < minWidth){
        std::cerr << "The nominal width is expected to be grater than the minimum width." << std::endl;
        return false;
    }

    if (!m_unicycleProblem.setMinWidth(minWidth)){
        return false;
    }
    m_minWidth = minWidth;
    m_nominalWidth = nominalWidth;

    return true;
}

void UnicyclePlanner::addTerminalStep(bool addStep)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_addTerminalStep = addStep;
}

void UnicyclePlanner::startWithLeft(bool startLeft)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_startLeft = startLeft;
}

void UnicyclePlanner::resetStartingFootIfStill(bool resetStartingFoot)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_resetStartingFoot = resetStartingFoot;
}

void UnicyclePlanner::setLeftFootYawOffsetInRadians(double leftYawOffsetInRadians)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_leftYawOffset = leftYawOffsetInRadians;
}

void UnicyclePlanner::setRightFootYawOffsetInRadians(double rightYawOffsetInRadians)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_rightYawOffset = rightYawOffsetInRadians;
}

bool UnicyclePlanner::computeNewSteps(std::shared_ptr< FootPrint > leftFoot, std::shared_ptr< FootPrint > rightFoot, double initTime, double endTime)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!leftFoot || !rightFoot){
        std::cerr <<"Empty feet pointers."<<std::endl;
        return false;
    }

    if (m_nominalWidth > m_maxLength){
        std::cerr << "Error: the maxLength parameter seems to be too restrictive. Notice that it represents the cartesian distance between the two feet (width included)." <<std::endl;
        return false;
    }

    if (m_nominalWidth > m_maxLengthBackward){
        std::cerr << "Error: the backwardMultiplier parameter seems to be too restrictive. Notice that it multiplies the cartesian distance between the two feet (width included)." <<std::endl;
        return false;
    }

    if (m_minAngle >= m_maxAngle) {
        std::cerr << "Error: the minAngle parameter is supposed to be lower than the maxAngle. Otherwise, when the feet start parallel, it would be impossible to rotate the swing foot." << std::endl;
        return false;
    }

    m_initTime = initTime;
    m_endTime = endTime;

    m_left.reset(new UnicycleFoot(leftFoot));
    m_right.reset(new UnicycleFoot(rightFoot));

    double maxVelocity = std::sqrt(std::pow(m_maxLength,2) - std::pow(m_nominalWidth,2))/m_minTime * m_linearVelocityConservativeFactor;
    double maxAngVelocity = m_maxAngle/m_minTime * m_angularVelocityConservativeFactor;

    if (!m_personFollowingController->setSaturations(maxVelocity, maxAngVelocity))
        return false;

    if (!m_directController->setSaturations(maxVelocity, maxAngVelocity))
        return false;

    if (!initializePlanner(m_initTime)){
        std::cerr << "Error during planner initialization." <<std::endl;
        return false;
    }

    double cost, minCost = 1e10; //this big number is actually not necessary since there is a check on tOptim also
    double deltaTime = 0, pauseTime = 0;
    double deltaAngle = 0;
    iDynTree::Vector2 rPl, newFootPosition, relativePlannedPosition;
    rPl.zero();
    newFootPosition.zero();
    relativePlannedPosition.zero();

    UnicycleState unicycleState;
    std::shared_ptr<UnicycleFoot> stanceFoot, swingFoot;

    size_t numberOfStepsLeft = leftFoot->numberOfSteps();
    size_t numberOfStepsRight = rightFoot->numberOfSteps();

    stanceFoot = m_swingLeft ? m_right : m_left;
    swingFoot = m_swingLeft ? m_left : m_right;

    Step prevStep;
    if (!(stanceFoot->getLastStep(prevStep)))
        return false;

    double t = m_initTime, tOptim = -1.0;
    pauseTime = t - prevStep.impactTime;

    double timeOffset;

    if (m_firstStep){
        timeOffset = m_nominalTime;
    } else {
        timeOffset = 0;
    }

    while (t <= m_endTime){
        deltaTime = t - prevStep.impactTime;

        if(!getIntegratorSolution(t, unicycleState)){
            return false;
        }

        deltaAngle = std::abs(stanceFoot->getUnicycleAngleFromStep(prevStep) - unicycleState.angle);

        if ((deltaTime >= m_minTime) && (t > (m_initTime + timeOffset))){ //The step is not too fast
            if (!(swingFoot->isTinyStep(unicycleState))){ //the step is not tiny
                deltaTime -= pauseTime; //deltaTime is the duration of a step. We remove the time in which the robot is simply standing still, otherwise the following condition could be triggered.
                if ((deltaTime > m_maxTime) || (t == m_endTime)){ //If this condition is true, it means we just exited the feasible region for what concerns the time. Hence, we check if we found a feasible solutions
                    if (tOptim > 0){  //a feasible point has been found

                        if(!getIntegratorSolution(tOptim, unicycleState)){
                            return false;
                        }

                        if(!swingFoot->addStepFromUnicycle(unicycleState, tOptim)){
                            std::cerr << "Error while inserting new step." << std::endl;
                            return false;
                        }

                        t = tOptim;
                        pauseTime = 0;

                        //reset
                        tOptim = -1.0;
                        m_firstStep = false;
                        timeOffset = 0;

                        m_swingLeft = !m_swingLeft;

                    } else { //No feasible solution has been found

                        std::cerr << "Unable to satisfy constraints given the specified maxStepTime. In the last evaluation of constraints, the following were not satisfied:" << std::endl;

                        m_unicycleProblem.printViolatedConstraints(rPl, deltaAngle, deltaTime, newFootPosition, relativePlannedPosition);

                        if(!stanceFoot->getUnicycleStateFromStep(prevStep, unicycleState)){
                            return false;
                        }

                        bool isTiny = swingFoot->isTinyStep(unicycleState);

                        if(!isTiny){
                            if(!swingFoot->addParallelStep(*(stanceFoot), t)){
                                std::cerr << "Error while inserting new step." << std::endl;
                                return false;
                            }

                            pauseTime = 0;
                            m_firstStep = false;
                            timeOffset = 0;
                            m_swingLeft = !m_swingLeft;

                        } else {

                            pauseTime = t - prevStep.impactTime; //We set as pause time the time since the last step up to now, since we did not manage to find any solution
                        }
                    }

                    stanceFoot = m_swingLeft ? m_right : m_left;
                    swingFoot = m_swingLeft ? m_left : m_right;

                    if (!(stanceFoot->getLastStep(prevStep))){
                        std::cerr << "Error updating last step" << std::endl;
                        return false;
                    }

                } else { //if ((deltaTime > m_maxTime) || (t == m_endTime)) //here I need to find the best solution

                    if(!get_rPl(unicycleState, rPl)){
                        std::cerr << "Error while retrieving the relative position from right to left." << std::endl;
                        return false;
                    }

                    if(!get_newStepRelativePosition(unicycleState, relativePlannedPosition)){
                        std::cerr << "Error while retrieving the relative position" << std::endl;
                        return false;
                    }

                    swingFoot->getFootPositionFromUnicycle(unicycleState, newFootPosition);


                    if((deltaTime > 0) && (m_unicycleProblem.areConstraintsSatisfied(rPl, deltaAngle, deltaTime, newFootPosition, relativePlannedPosition))){ //constraints are satisfied

                        if(!m_unicycleProblem.getCostValue(rPl, deltaAngle, deltaTime, newFootPosition, relativePlannedPosition, cost)){
                            std::cerr << "Error while evaluating the cost function." << std::endl;
                            return false;
                        }

                        if ((tOptim < 0) || (cost <= minCost)){ //no other feasible point has been found yet
                            tOptim = t;
                            minCost = cost;
                        }
                    }
                }
            } else { //Arrive here if the step is tiny
                pauseTime += m_dT;
            }
        }

        if (((t+m_dT) > m_endTime) && (t < m_endTime)){
            t = m_endTime;
        } else {
            t += m_dT;
        }
    }

    if (m_addTerminalStep){

        if(!getIntegratorSolution(m_endTime, unicycleState)){
            return false;
        }

        if(!addTerminalStep(unicycleState)){
            std::cerr << "Error while adding the terminal step." << std::endl;
            return false;
        }

    }

    if ((numberOfStepsLeft == leftFoot->numberOfSteps()) && (numberOfStepsRight == rightFoot->numberOfSteps())){ //no steps have been added
        Step lastLeft, lastRight;

        leftFoot->getLastStep(lastLeft);
        rightFoot->getLastStep(lastRight);

        if (lastLeft.impactTime > lastRight.impactTime){
            lastRight.impactTime = lastLeft.impactTime;
            rightFoot->clearLastStep();
            rightFoot->addStep(lastRight);

            if (!m_resetStartingFoot) {
                m_startLeft = false;
            }

        } else if (lastLeft.impactTime < lastRight.impactTime){
            lastLeft.impactTime = lastRight.impactTime;
            leftFoot->clearLastStep();
            leftFoot->addStep(lastLeft);

            if (!m_resetStartingFoot) {
                m_startLeft = true;
            }
        }

    }
    return true;
}

bool UnicyclePlanner::startWithLeft() const
{
    return m_swingLeft;
}

bool UnicyclePlanner::getPersonPosition(double time, iDynTree::Vector2& personPosition)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    UnicycleState unicycleState;

    if(!getIntegratorSolution(time, unicycleState)){
        std::cerr << "Time is out of range." << std::endl;
        return false;
    }
    personPosition = m_personFollowingController->getPersonPosition(unicycleState.position, unicycleState.angle);
    return true;
}

void UnicyclePlanner::setFreeSpaceEllipseMethod(FreeSpaceEllipseMethod method)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_freeSpaceMethod = method;
    m_personFollowingController->setFreeSpaceEllipse(FreeSpaceEllipse()); //Reset the ellipse
    m_unicycleProblem.setFreeSpaceEllipse(FreeSpaceEllipse()); //Reset the ellipse
}

bool UnicyclePlanner::setFreeSpaceEllipse(const FreeSpaceEllipse &freeSpaceEllipse)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_freeSpaceMethod == FreeSpaceEllipseMethod::REFERENCE_ONLY ||
            m_freeSpaceMethod == FreeSpaceEllipseMethod::REFERENCE_AND_FOOTSTEPS)
    {
        if (!m_personFollowingController->setFreeSpaceEllipse(freeSpaceEllipse))
        {
            return false;
        }
    }

    if (m_freeSpaceMethod == FreeSpaceEllipseMethod::FOOTSTEPS_ONLY ||
            m_freeSpaceMethod == FreeSpaceEllipseMethod::REFERENCE_AND_FOOTSTEPS)
    {
        if (!m_unicycleProblem.setFreeSpaceEllipse(freeSpaceEllipse))
        {
            return false;
        }
    }

    return true;
}

bool UnicyclePlanner::setFreeSpaceEllipseConservativeFactor(double conservativeFactor)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setFreeSpaceEllipseConservativeFactor(conservativeFactor);
}

bool UnicyclePlanner::setInnerFreeSpaceEllipseOffset(double offset)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setInnerFreeSpaceEllipseOffset(offset);
}

bool UnicyclePlanner::setInnerFreeSpaceEllipseOffsets(double semiMajorAxisOffset, double semiMinorAxisOffset)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setInnerFreeSpaceEllipseOffsets(semiMajorAxisOffset, semiMinorAxisOffset);
}

bool UnicyclePlanner::setUnicycleController(UnicycleController controller)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (controller == UnicycleController::PERSON_FOLLOWING)
    {
        m_currentController = controller;
        return m_unicycle->setController(m_personFollowingController);
    }

    if (controller == UnicycleController::DIRECT)
    {
        m_currentController = controller;
        return m_unicycle->setController(m_directController);
    }

    return false;
}

bool UnicyclePlanner::setLateralVelocityConservaiveFactor(double lateralVelocityConservaiveFactor)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (lateralVelocityConservaiveFactor < 0.0 || lateralVelocityConservaiveFactor > 1.0)
    {
        std::cerr << " The parameter lateralVelocityConservaiveFactor is supposed to be between 0 and 1" << std::endl;
        return false;
    }
    else
    {
        m_lateralVelocityConservaiveFactor = lateralVelocityConservaiveFactor;
        return true;
    }
}

bool UnicyclePlanner::interpolateNewStepsFromPath(std::shared_ptr< FootPrint > leftFoot, 
                                                  std::shared_ptr< FootPrint > rightFoot, 
                                                  double initTime, 
                                                  double endTime
                                                  )
{
    std::lock_guard<std::mutex> guard(m_mutex);
    
    // First consistency cheks
    if (!leftFoot || !rightFoot){
        std::cerr <<"Empty feet pointers."<<std::endl;
        return false;
    }

    if (m_nominalWidth > m_maxLength){
        std::cerr << "Error: the maxLength parameter seems to be too restrictive. Notice that it represents the cartesian distance between the two feet (width included)." <<std::endl;
        return false;
    }

    if (m_nominalWidth > m_maxLengthBackward){
        std::cerr << "Error: the backwardMultiplier parameter seems to be too restrictive. Notice that it multiplies the cartesian distance between the two feet (width included)." <<std::endl;
        return false;
    }

    if (m_minAngle >= m_maxAngle) {
        std::cerr << "Error: the minAngle parameter is supposed to be lower than the maxAngle. Otherwise, when the feet start parallel, it would be impossible to rotate the swing foot." << std::endl;
        return false;
    }

    std::vector<UnicycleState> navigationPath = m_inputPath;    //input set externally fromm another method to m_inputPath
    std::vector<PosesPairInterpolator::PoseStamped> interpolatedPath;   //output of the integration

    m_initTime = initTime;
    m_endTime = endTime;

    m_left.reset(new UnicycleFoot(leftFoot));
    m_right.reset(new UnicycleFoot(rightFoot));

    double maxVelocity = std::sqrt(std::pow(m_maxLength,2) - std::pow(m_nominalWidth,2))/m_minTime * m_linearVelocityConservativeFactor;
    double maxLateralVelocity = maxVelocity * m_lateralVelocityConservaiveFactor;    //corrective factor equal to slowWhenSidewaysFactor - TODO: parameterize 0.2
    double maxAngVelocity = m_maxAngle/m_minTime * m_angularVelocityConservativeFactor;

    // TODO are these saturations necessary?
    if (!m_personFollowingController->setSaturations(maxVelocity, maxAngVelocity))
        return false;

    if (!m_directController->setSaturations(maxVelocity, maxAngVelocity))
        return false;

    if (!initializePlanner(m_initTime)){
        std::cerr << "Error during planner initialization." <<std::endl;
        return false;
    }

    double cost, minCost = 1e10; //this big number is actually not necessary since there is a check on tOptim also
    double deltaTime = 0, pauseTime = 0;
    double deltaAngle = 0;
    iDynTree::Vector2 rPl, newFootPosition, relativePlannedPosition;
    rPl.zero();
    newFootPosition.zero();
    relativePlannedPosition.zero();

    UnicycleState unicycleState;
    std::shared_ptr<UnicycleFoot> stanceFoot, swingFoot;

    size_t numberOfStepsLeft = leftFoot->numberOfSteps();
    size_t numberOfStepsRight = rightFoot->numberOfSteps();

    stanceFoot = m_swingLeft ? m_right : m_left;
    swingFoot = m_swingLeft ? m_left : m_right;

    Step prevStep;
    if (!(stanceFoot->getLastStep(prevStep)))
        return false;

    double t = m_initTime, tOptim = -1.0;
    pauseTime = t - prevStep.impactTime;

    double timeOffset;

    if (m_firstStep){
        //timeOffset = m_nominalTime;
        timeOffset = m_minTime; //we want to start analyzing poses from the minimum time instead of the nominal one
    } else {
        timeOffset = 0;
    }

    //The robot has to stay still at startup or when we have an invalid command
    if (navigationPath.size()<2)
    {
        std::cerr <<"The navigation path has less than 2 poses (at least 2 points are needed) - Size: " << navigationPath.size() << " Staying still." <<std::endl;
        
        UnicycleState zeroUnicycleState;
        //Get the unicycle state from the current step:
        if(!stanceFoot->getUnicycleStateFromStep(prevStep, zeroUnicycleState)){
                            return false;
                        }
        //Resetting starting swing foot
        m_startLeft = true;
        return true;
    }
    std::cout << "interpolatedPath = interpolatePath(navigationPath, maxVelocity, maxLateralVelocity, maxAngVelocity);" << std::endl;
    //Interpolation of the given path
    interpolatedPath = interpolatePath(navigationPath, maxVelocity, maxLateralVelocity, maxAngVelocity);
    //now we have everything on the interpolatedPath. Both time and poses for the whole time horizon
    std::cout << "interpolatedPath.size() = " << interpolatedPath.size() << std::endl;
    //Reset variables
    t = m_initTime;
    tOptim = -1.0;
    //EVALUATION LOOP
    for (auto it = interpolatedPath.begin(); it != interpolatedPath.end(); ++it){
        std::cout << "EVALUATION LOOP: Pose time: "<< it->time << " X: " << it->pose.position(0) 
        << " Y: " << it->pose.position(1) << " Angle: " << it->pose.angle << std::endl;
        t = it->time;
        deltaTime = t - prevStep.impactTime;

        unicycleState = it->pose;

        deltaAngle = std::abs(stanceFoot->getUnicycleAngleFromStep(prevStep) - unicycleState.angle);

        //we have to take into account angle periodicity
        if (deltaAngle >= M_PI)
        {
            deltaAngle = std::abs(deltaAngle - 2* M_PI);
        }

        if ((deltaTime >= m_minTime) && (t > (m_initTime + timeOffset))){ //The step is not too fast
            if (!(swingFoot->isTinyStep(unicycleState))){ //the step is not tiny

                deltaTime -= pauseTime; //deltaTime is the duration of a step. We remove the time in which the robot is simply standing still, otherwise the following condition could be triggered.
                if ((deltaTime > m_maxTime) || (t == m_endTime)){ //If this condition is true, it means we just exited the feasible region for what concerns the time. Hence, we check if we found a feasible solutions
                    if (tOptim > 0){  //a feasible point has been found

                        auto bestMatch = std::find_if(interpolatedPath.begin(),
                                                      interpolatedPath.end(),
                                                      [&tOptim] (PosesPairInterpolator::PoseStamped pS) {return (pS.time == tOptim);});

                        if(!swingFoot->addStepFromUnicycle(bestMatch->pose, tOptim)){
                            std::cout << "Error while inserting new step." << std::endl;
                            return false;
                        }

                        t = tOptim;

                        it = bestMatch; //properly set the iterator
                        pauseTime = 0;

                        //reset
                        tOptim = -1.0;
                        m_firstStep = false;
                        timeOffset = 0;

                        m_swingLeft = !m_swingLeft;

                    } else { //No feasible solution has been found

                        std::cout << "Unable to satisfy constraints given the specified maxStepTime. In the last evaluation of constraints, the following were not satisfied:" << std::endl;

                        m_unicycleProblem.printViolatedConstraints(rPl, deltaAngle, deltaTime, newFootPosition, relativePlannedPosition);

                        if(!stanceFoot->getUnicycleStateFromStep(prevStep, unicycleState)){
                            return false;
                        }

                        bool isTiny = swingFoot->isTinyStep(unicycleState);

                        if(!isTiny){
                            if(!swingFoot->addParallelStep(*(stanceFoot), t)){
                                std::cout << "Error while inserting new step." << std::endl;
                                return false;
                            }

                            pauseTime = 0;
                            m_firstStep = false;
                            timeOffset = 0;
                            m_swingLeft = !m_swingLeft;

                        } else {

                            pauseTime = t - prevStep.impactTime; //We set as pause time the time since the last step up to now, since we did not manage to find any solution
                        }
                    }

                    stanceFoot = m_swingLeft ? m_right : m_left;
                    swingFoot = m_swingLeft ? m_left : m_right;

                    if (!(stanceFoot->getLastStep(prevStep))){
                        std::cout << "Error updating last step" << std::endl;
                        return false;
                    }

                } else { //if ((deltaTime > m_maxTime) || (t == m_endTime)) //here I need to find the best solution
                    std::cout << "EVALUATING Pose time: "<< interpolatedPath.back().time << " X: " << interpolatedPath.back().pose.position(0) 
                    << " Y: " << interpolatedPath.back().pose.position(1) << " Angle: " << interpolatedPath.back().pose.angle << std::endl;
                    if(!get_rPl(unicycleState, rPl)){
                        std::cerr << "Error while retrieving the relative position from right to left." << std::endl;
                        return false;
                    }

                    if(!get_newStepRelativePosition(unicycleState, relativePlannedPosition)){
                        std::cerr << "Error while retrieving the relative position" << std::endl;
                        return false;
                    }

                    swingFoot->getFootPositionFromUnicycle(unicycleState, newFootPosition);

                    //Update the latest stance foot
                    auto latestStanceFoot = m_swingLeft ? m_right : m_left;
                    Step tmpStep;
                    if (!(latestStanceFoot->getLastStep(tmpStep))){
                        std::cout << "Error updating last step" << std::endl;
                        return false;
                    }

                    if ((deltaTime > 0) && checkConstraints(rPl, deltaAngle, deltaTime, newFootPosition, tmpStep.position))
                    {
                        if(!m_unicycleProblem.getCostValue(rPl, deltaAngle, deltaTime, newFootPosition, relativePlannedPosition, cost)){
                            std::cout << "Error while evaluating the cost function." << std::endl;
                            return false;
                        }

                        std::cout<< "Evaluating best match: tOptim: " << tOptim << " minCost: " << minCost << 
                        " optimalState X: " << unicycleState.position(0) << " optimalState Y: " << unicycleState.position(1) << 
                        " optimalState Th: " << unicycleState.angle << std::endl;

                        if ((tOptim < 0) || (cost <= minCost)){ //no other feasible point has been found yet
                            tOptim = t;
                            minCost = cost;
                        }
                    }
                    else
                    {
                        std::cout << "deltaTime > 0 : " << deltaTime << std::endl;
                    }
                    
                }
            } else { //Arrive here if the step is tiny
                pauseTime += m_dT;
                std::cout << "TINY STEP - pauseTime: " << pauseTime << std::endl;
            }
        }

        if (((t+m_dT) > m_endTime) && (t < m_endTime)){
            std::cout<<"Exiting evaluation loop" << std::endl;
            break;
        }
    }

    //Add the last parallel terminal step
    if (m_addTerminalStep){

        auto bestMatch = std::find_if(interpolatedPath.begin(),
                                      interpolatedPath.end(),
                                      [=] (PosesPairInterpolator::PoseStamped pS) {return (pS.time == m_endTime);});

        if(!addTerminalStep(bestMatch->pose)){
            std::cout << "Error while adding the terminal step." << std::endl;
            return false;
        }
    }

    //Check from what foot start walking
    if ((numberOfStepsLeft == leftFoot->numberOfSteps()) && (numberOfStepsRight == rightFoot->numberOfSteps())){ //no steps have been added
        Step lastLeft, lastRight;

        leftFoot->getLastStep(lastLeft);
        rightFoot->getLastStep(lastRight);

        if (lastLeft.impactTime > lastRight.impactTime){
            lastRight.impactTime = lastLeft.impactTime;
            rightFoot->clearLastStep();
            rightFoot->addStep(lastRight);

            if (!m_resetStartingFoot) {
                m_startLeft = false;
            }

        } else if (lastLeft.impactTime < lastRight.impactTime){
            lastLeft.impactTime = lastRight.impactTime;
            leftFoot->clearLastStep();
            leftFoot->addStep(lastLeft);

            if (!m_resetStartingFoot) {
                m_startLeft = true;
            }
        }

    }

    std::cout << "Passing from: numberOfStepsLeft = " << numberOfStepsLeft << " to " << leftFoot->numberOfSteps() << " and " << 
    "Passing from: numberOfStepsRight = " << numberOfStepsRight << " to " << rightFoot->numberOfSteps() << std::endl;
    std::cout << "LEFT STEPS" << std::endl;
    for (auto step : leftFoot->getSteps()){
        std::cerr << "Position "<< step.position.toString() << std::endl;
        std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
        std::cerr << "Time  "<< step.impactTime << std::endl;
    }
    std::cout << std::endl << "RIGHT STEPS" << std::endl;
    for (auto step : rightFoot->getSteps()){
        std::cerr << "Position "<< step.position.toString() << std::endl;
        std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
        std::cerr << "Time  "<< step.impactTime << std::endl;
    }

    return true;
}

bool UnicyclePlanner::checkConstraints(iDynTree::Vector2 rPl, double deltaAngle, double deltaTime, iDynTree::Vector2 newFootPosition, iDynTree::Vector2 prevStepPosition){

    bool result = true;

    double distance = (iDynTree::toEigen(newFootPosition) - iDynTree::toEigen(prevStepPosition)).norm();

    if (distance > m_maxLength){
        std::cout <<"[ERROR] Distance constraint not satisfied: " << distance << std::endl;
        result = false;
    }

    if (deltaAngle > m_maxAngle){
        std::cout <<"[ERROR] Angle constraint not satisfied: " << deltaAngle << std::endl;
        result = false;
    }

    if (deltaTime < m_minTime){
        std::cout <<"[ERROR] Min time constraint not satisfied: " << deltaTime << std::endl;
        result = false;
    }

    if (rPl(1) < m_minWidth){
        std::cout <<"[ERROR] Width constraint not satisfied: " << rPl(1) << std::endl;
        result = false;
    }

    if(!result)
        return false;

    return true;
}

bool UnicyclePlanner::setInputPath (std::vector<UnicycleState> input)
{
    m_inputPath = input;
    return true;
}

/*
std::vector<UnicyclePlanner::PoseStamped> UnicyclePlanner::interpolatePath(std::vector<UnicycleState> &navigationPath, 
                                        const double maxVelocity, 
                                        const double maxLateralVelocity, 
                                        const double maxAngVelocity)
{
    std::vector<UnicyclePlanner::PoseStamped> interpolatedPath;   //output of the interpolation
    double t = m_initTime;          //time
    double prevTime = m_initTime;   //latest time istant of a pose in the path. It is initialized from the starting time, then will grow for each pose integrated
    PoseStamped initialPS {navigationPath[0], prevTime};
    interpolatedPath.push_back(initialPS);  //the first pose of the path

    for (size_t i = 0; i < navigationPath.size() - 1; ++i)
    {
        //Let's assume a linear uniform motion between two consecutive path poses at constant speed.
        //we now calculate the future time istant of the next (i+1) pose in the path and the geometrical components
        double distance = std::sqrt(pow(navigationPath[i+1].position(0) - navigationPath[i].position(0), 2) +
                                    pow(navigationPath[i+1].position(1) - navigationPath[i].position(1), 2) 
                                    );
        double slope = (navigationPath[i+1].position(1) - navigationPath[i].position(1)) / 
                        (navigationPath[i+1].position(0) - navigationPath[i].position(0));
        double slope_angle = atan(slope);
        double cos_theta = std::abs(cos(slope_angle));    
        double sin_theta = std::abs(sin(slope_angle)); 

        //if both poses aligned to the path = 0, if both poses aligned but not with the path > 0, if both poses perpendicular to the path = 1
        double proprotionFactor = (std::abs(slope_angle - navigationPath[i+1].angle) + std::abs(slope_angle - navigationPath[i].angle))/ 2 / M_PI_2 ;
        //linear speed module that moves from pose i to pose i+1
        double linearSpeed = std::abs(1 - proprotionFactor) * maxVelocity + proprotionFactor * maxLateralVelocity;

        //calculate the direction, on the plane, of the two consecutive poses on its components (x,y,theta)
        iDynTree::Vector3 direction;   //(x, y, theta) direction sign components / versors
        direction(0) = (navigationPath[i+1].position(0) - navigationPath[i].position(0) >= 0) ? 1 : -1;
        direction(1) = (navigationPath[i+1].position(1) - navigationPath[i].position(1) >= 0) ? 1 : -1;
        
        //Angles vary between (-pi, pi]
        double angleDifference = navigationPath[i+1].angle - navigationPath[i].angle;
        if (angleDifference > M_PI)
        {
            direction(2) = -1;
        }
        else if (angleDifference >=0 && angleDifference <= M_PI)
        {
            direction(2) = 1;
        }
        else if (angleDifference <0 && angleDifference >= -M_PI)
        {
            direction(2) = -1;
        }
        else    // angleDifference < -M_PI
        {
            direction(2) = 1;
        }

        //Times computation. The linear and angular motions are considered indipendent from each other
        double elapsedTimeLinear = distance / linearSpeed;  //Time required for moving from (x_i, y_i) to (x_i+1, y_i+1)
        double deltaPosesAngle = std::abs(navigationPath[i+1].angle - navigationPath[i].angle); //orientation difference between two consecutive path poses
        if (deltaPosesAngle >= M_PI)    //take into account angle periodicity (-pi, pi]
        {
            deltaPosesAngle = std::abs(deltaPosesAngle - 2* M_PI);
        }

        double elapsedTimeAngular = deltaPosesAngle / maxAngVelocity;   //Time required for moving from theta_i to theta_i+1
        double iterationEndTime = 0;    // Time needed for reaching the next pose in the path. Initialized to 0

        //SHIM CONTROLLER. rotation BEFORE linear movement if I have a too big deltaPosesAngle.
        // The robot will rotate in-place until the extra agle will be compensated, then it will roto-translate to the goal pose.
        if (elapsedTimeAngular > elapsedTimeLinear)     
        {
            //compute the interpolation for the rotation of the unicycle on itself untill is aligned and add this time to the elapsedTimeLinear
            //update also t for each dT passed
            int missingIterations = std::ceil((elapsedTimeAngular - elapsedTimeLinear) / m_dT) ;    //round up by excess to calculate the number of iterations for fulling the time gap
            UnicycleState shimState;
            //Rotation in place -> X and Y don't change
            shimState.position(0) = navigationPath[i].position(0);
            shimState.position(1) = navigationPath[i].position(1);
            for (size_t i = 1; i < missingIterations; i++)
            {
                t += m_dT;
                iterationEndTime += m_dT;
                shimState.angle = interpolatedPath.back().pose.angle + direction(2) * maxAngVelocity * m_dT;   
                //deal with angle periodicity
                if (shimState.angle > M_PI) //overshoot in the positive domain
                {
                    shimState.angle -= 2*M_PI;
                }
                else if (shimState.angle < -M_PI)   //overshoot in the negative domain
                {
                    shimState.angle += 2*M_PI;
                }
                
                //save the pose
                PoseStamped ps {shimState, t};
                interpolatedPath.push_back(ps);
            }
            //now we have elapsedTimeAngular <= elapsedTimeLinear
            //so we add elapsedTimeLinear to the missing time
            iterationEndTime += elapsedTimeLinear;
        }
        else
        {
            iterationEndTime = elapsedTimeLinear;   //we take the time from the linear motion
        }
        
        iterationEndTime += prevTime;    //add the time elapsed for all the previous poses in the path and the initial time

        //interpolate between two path poses
        int iterationNum = 0;
        double sweptAngle = 0; //the cumulative angle which is being swept after each iter
        double missingAngle = 0;
        if (elapsedTimeAngular >= elapsedTimeLinear)
        {
            missingAngle = std::ceil(elapsedTimeLinear / m_dT) * m_dT * maxAngVelocity;
        }
        else
        {
            missingAngle = elapsedTimeAngular * maxAngVelocity;
        }
        
        while (t < iterationEndTime)
        {
            //compute the next interpolated point
            t += m_dT;
            ++iterationNum;
            UnicycleState nextState;
            //check if we overshoot the final pose -> then we clamp it
            if (t >= iterationEndTime)
            {
                nextState.position(0) = navigationPath[i+1].position(0);
                nextState.position(1) = navigationPath[i+1].position(1);
                nextState.angle = navigationPath[i+1].angle;
            }
            else
            {
                //starting from the previous pose in the path, I add each passed increment on each component
                nextState.position(0) = interpolatedPath.back().pose.position(0) + direction(0) * m_dT * (linearSpeed * cos_theta);    //the first part of the equation computes the number of iteration of dT passed untill this time istant.
                                                                                                                                    //the second part is the x component of the maximum speed
                nextState.position(1) = interpolatedPath.back().pose.position(1) + direction(1) * m_dT * (linearSpeed * sin_theta);
                
                double angleIncrement = m_dT * maxAngVelocity;
                double next_angle = interpolatedPath.back().pose.angle + direction(2) * m_dT * maxAngVelocity;
                sweptAngle += angleIncrement;
                
                if (sweptAngle >= missingAngle)
                {
                    nextState.angle = navigationPath[i+1].angle;
                }
                else
                {
                    nextState.angle = next_angle;
                }
            }
            iterationNum = 0; //reset the counter for each couple of poses
            
            //save the pose
            PoseStamped ps {nextState, t};
            interpolatedPath.push_back(ps);
        }
        prevTime = t;   //keep track of the latest time istant
        //check if we have passed the maximum time horizon
        if (t >= m_endTime)
        {
            break;  //exit for loop
        }
    }
    return interpolatedPath;
}*/

std::vector<PosesPairInterpolator::PoseStamped> UnicyclePlanner::interpolatePath(std::vector<UnicycleState> &navigationPath, 
                                        const double maxVelocity, 
                                        const double maxLateralVelocity, 
                                        const double maxAngVelocity)
{
    std::vector<PosesPairInterpolator::PoseStamped> outputPath;   //output of the interpolation
    double prevTime = m_initTime;   //latest time istant of a pose in the path. It is initialized from the starting time, then will grow for each pose integrated
    PosesPairInterpolator::PoseStamped initialPS {navigationPath[0], prevTime};
    outputPath.push_back(initialPS);  //the first pose of the path

    for (size_t i = 0; i < navigationPath.size() - 1; ++i)
    {
        std::cout << "pairInterpolator" << std::endl;
        PosesPairInterpolator pairInterpolator(navigationPath[i], navigationPath[i+1], maxVelocity, maxLateralVelocity, maxAngVelocity, m_dT, prevTime);
        std::cout << "computeMotionParameters" << std::endl;
        if(!pairInterpolator.computeMotionParameters())
        {
            return outputPath;
        }
        std::cout << "ETA_Computation" << std::endl;
        if(!pairInterpolator.ETA_Computation())
        {
            return outputPath;
        }
        std::cout << "shimController" << std::endl;
        std::vector<PosesPairInterpolator::PoseStamped> shimPart = pairInterpolator.shimController(outputPath.back());
        if(!shimPart.empty())
        {
            //append the part to the path
            outputPath.insert(outputPath.end(), shimPart.begin(), shimPart.end());
        }
        std::cout << "interpolationPart" << std::endl;
        std::vector<PosesPairInterpolator::PoseStamped> interpolationPart = pairInterpolator.interpolate(outputPath.back());
        //append
        outputPath.insert(outputPath.end(), interpolationPart.begin(), interpolationPart.end());
        prevTime = interpolationPart.back().time;   //keep track of the latest time istant
        std::cout << "prevTime: " << prevTime << std::endl;
        //check if we have passed the maximum time horizon
        if (prevTime >= m_endTime)
        {
            break;  //exit for loop
        }
    }

    return outputPath;
}