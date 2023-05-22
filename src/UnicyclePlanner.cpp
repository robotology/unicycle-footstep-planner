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
        else if(m_currentController == UnicycleController::DIRECT)
        {
            unicycleState.position.zero();
            unicycleState.angle = 0.0;
        }
        else    //NAVIGATION
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
            m_navigationController->setInactiveUntil(initTime + m_nominalTime - m_minTime); //Same as direct
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

    //Same for navigation: we change the first stepping foot based on the lateral velocity
    if (m_firstStep && m_currentController == UnicycleController::NAVIGATION &&
            std::abs(m_navigationController->getDesiredLateralVelocity()) > 0)
    {
        //If we want to move right, use the right as first stepping foot
        m_swingLeft = m_navigationController->getDesiredLateralVelocity() > 0;
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
    ,m_navigationController(std::make_shared<UnicycleNavigationController>())
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
            m_directController->setSlowWhenTurnGain(slowWhenTurnGain) && 
            m_navigationController->setSlowWhenTurnGain(slowWhenTurnGain);
}

bool UnicyclePlanner::setSlowWhenBackwardFactor(double slowWhenBackwardFactor)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setSlowWhenBackwardFactor(slowWhenBackwardFactor) &&
            m_directController->setSlowWhenBackwardFactor(slowWhenBackwardFactor) &&
            m_navigationController->setSlowWhenBackwardFactor(slowWhenBackwardFactor);
}

bool UnicyclePlanner::setSlowWhenSidewaysFactor(double slowWhenSidewaysFactor)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_personFollowingController->setSlowWhenSidewaysFactor(slowWhenSidewaysFactor) && 
            m_directController->setSlowWhenSidewaysFactor(slowWhenSidewaysFactor) &&
            m_navigationController->setSlowWhenSidewaysFactor(slowWhenSidewaysFactor);
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

    if (!m_navigationController->setSaturations(maxVelocity, maxAngVelocity))
        return false;

    double maxForwardSpeed, maxLateralSpeed, maxAngularSpeed;
    if(!m_navigationController->getSaturationLimits(maxForwardSpeed, maxLateralSpeed, maxAngularSpeed))
        return false;

    if(!m_navigationController->setMaxVelocities(maxForwardSpeed, maxLateralSpeed, maxAngularSpeed))
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
    std::cout << "EVALUATION LOOP" << std::endl;
    std::cout << "Eval params: m_minTime: " << m_minTime << " m_nominalTime " << m_nominalTime << " m_maxTime " << m_maxTime << std::endl;
    while (t <= m_endTime){
        deltaTime = t - prevStep.impactTime;
        std::cout << " delta time: " << deltaTime << std::endl;
        if(!getIntegratorSolution(t, unicycleState)){
            return false;
        }
        std::cout << " Time: " << t << " Integrated state - x: " << unicycleState.position(0) << " y: " << unicycleState.position(1) << " angle: " << unicycleState.angle << std::endl;

        deltaAngle = std::abs(stanceFoot->getUnicycleAngleFromStep(prevStep) - unicycleState.angle);

        if ((deltaTime >= m_minTime) && (t > (m_initTime + timeOffset))){ //The step is not too fast
            if (!(swingFoot->isTinyStep(unicycleState))){ //the step is not tiny
                deltaTime -= pauseTime; //deltaTime is the duration of a step. We remove the time in which the robot is simply standing still, otherwise the following condition could be triggered.
                std::cout << " delta time after pause: " << deltaTime << std::endl;
                if ((deltaTime > m_maxTime) || (t == m_endTime)){ //If this condition is true, it means we just exited the feasible region for what concerns the time. Hence, we check if we found a feasible solutions
                    std::cout << "Exited the feasible region for what concerns the time " << std::endl;
                    if (tOptim > 0){  //a feasible point has been found

                        if(!getIntegratorSolution(tOptim, unicycleState)){
                            return false;
                        }

                        std::cout << "\n FOUND BEST MATCH AT Time: " << t << " Integrated state - x: " << unicycleState.position(0) << " y: " << unicycleState.position(1) << " angle: " << unicycleState.angle << std::endl;


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
                        std::cout << "Cost value of point: " << cost << std::endl;

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
    else if (controller == UnicycleController::DIRECT)
    {
        m_currentController = controller;
        return m_unicycle->setController(m_directController);
    }
    else if (controller == UnicycleController::NAVIGATION)
    {
        m_currentController = controller;
        return m_unicycle->setController(m_navigationController);
    }

    return false;
}

bool UnicyclePlanner::setInputPath (std::vector<UnicycleState> input)
{
    //TODO add sanity check
    m_navigationController->setNavigationPath(input);
    return true;
}