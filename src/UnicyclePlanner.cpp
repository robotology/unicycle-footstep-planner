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

    iDynTree::Vector2 initialUnicycle, dummyVector;
    initialUnicycle.zero();
    dummyVector.zero();
    double initialAngle = 0;
    double initTimeFromFeet;

    if ((m_left->numberOfSteps() == 0) && (m_right->numberOfSteps() == 0)){

        if (!m_controller->getDesiredPoint(initTime, initialUnicycle, dummyVector))
            return false;

        initialUnicycle(0) = initialUnicycle(0) - m_controller->getPersonDistance()(0);
        initialUnicycle(1) = initialUnicycle(1) - m_controller->getPersonDistance()(1);
        initialAngle = 0;

        m_left->addStepFromUnicycle(initialUnicycle, initialAngle, initTime);
        m_right->addStepFromUnicycle(initialUnicycle, initialAngle, initTime);

        m_firstStep = true;
        m_swingLeft = m_startLeft;

    } else {
        if (m_left->numberOfSteps() == 0){
            Step lastFoot;

            if (!(m_right->getLastStep(lastFoot)))
                return false;

            if (!(m_right->getUnicyclePositionFromFoot(lastFoot.position, lastFoot.angle, initialUnicycle)))
                return false;

            initialAngle = lastFoot.angle;
            initTimeFromFeet = lastFoot.impactTime;
            m_swingLeft = true;
            m_firstStep = false;

        } else if (m_right->numberOfSteps() == 0){
            Step lastFoot;

            if (!(m_left->getLastStep(lastFoot)))
                return false;

            if (!(m_left->getUnicyclePositionFromFoot(lastFoot.position, lastFoot.angle, initialUnicycle)))
                return false;

            initialAngle = lastFoot.angle;
            initTimeFromFeet = lastFoot.impactTime;
            m_swingLeft = false;
            m_firstStep = false;

        } else {
            Step lastFootL, lastFootR;

            if (!(m_left->getLastStep(lastFootL)) || !(m_right->getLastStep(lastFootR)))
                return false;

            if(lastFootL.impactTime == lastFootR.impactTime){
                if (m_startLeft){
                    if (!(m_right->getUnicyclePositionFromFoot(lastFootR.position, lastFootR.angle, initialUnicycle)))
                        return false;

                    initialAngle = lastFootR.angle;
                    m_swingLeft = true;
                } else {
                    if (!(m_left->getUnicyclePositionFromFoot(lastFootL.position, lastFootL.angle, initialUnicycle)))
                        return false;

                    initialAngle = lastFootL.angle;
                    m_swingLeft = false;
                }
                initTimeFromFeet = lastFootL.impactTime;
                m_firstStep = true;
            } else if (lastFootL.impactTime < lastFootR.impactTime) {
                if (!(m_right->getUnicyclePositionFromFoot(lastFootR.position, lastFootR.angle, initialUnicycle)))
                    return false;

                initialAngle = lastFootR.angle;
                m_swingLeft = true;
                m_firstStep = false;
                initTimeFromFeet = lastFootR.impactTime;
            } else {
                if (!(m_left->getUnicyclePositionFromFoot(lastFootL.position, lastFootL.angle, initialUnicycle)))
                    return false;

                initialAngle = lastFootL.angle;
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
        initialPoint.yDesired = m_controller->getPersonPosition(initialUnicycle, initialAngle);
        dummyVector.zero();
        initialPoint.yDotDesired = dummyVector;

        if (!(m_controller->setDesiredPoint(initialPoint))){
            std::cerr <<"Failed to set a the intial reference given the provided FootPrints." <<std::endl;
            return false;
        }

        if (m_firstStep) {
            initialPoint.initTime = initTime + m_nominalTime - m_minTime;

            if (!(m_controller->setDesiredPoint(initialPoint))){
                std::cerr <<"Failed to set a the intial dummy reference to have a slower first step." <<std::endl;
                return false;
            }
        }
    }

    if(!(m_unicycle->setInitialState(initialUnicycle, initialAngle))){
        std::cerr << "Error while setting the initial state for the integrator." << std::endl;
        return false;
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

    if (!(m_left->setTinyStepLength(m_minLength)))
        return false;
    if (!(m_right->setTinyStepLength(m_minLength)))
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

bool UnicyclePlanner::get_rPl(const iDynTree::Vector2& unicyclePosition, double unicycleAngle, iDynTree::Vector2& rPl)
{
    static iDynTree::MatrixDynSize rTranspose(2,2);
    iDynTree::Vector2 newPosition;

    Step prevStep;
    if (m_swingLeft){
        if (!(m_right->getLastStep(prevStep)))
            return false;
        if(!(m_left->getFootPositionFromUnicycle(unicyclePosition, unicycleAngle, newPosition)))
            return false;

        double c_theta = std::cos(prevStep.angle);
        double s_theta = std::sin(prevStep.angle);

        rTranspose(0,0) = c_theta;
        rTranspose(1,0) = -s_theta;
        rTranspose(0,1) = s_theta;
        rTranspose(1,1) = c_theta;

        iDynTree::toEigen(rPl) =
                iDynTree::toEigen(rTranspose)*(iDynTree::toEigen(newPosition) - iDynTree::toEigen(prevStep.position));
    } else {
        if (!(m_left->getLastStep(prevStep)))
            return false;
        if(!(m_right->getFootPositionFromUnicycle(unicyclePosition, unicycleAngle, newPosition)))
            return false;

        double c_theta = std::cos(unicycleAngle);
        double s_theta = std::sin(unicycleAngle);

        rTranspose(0,0) = c_theta;
        rTranspose(1,0) = -s_theta;
        rTranspose(0,1) = s_theta;
        rTranspose(1,1) = c_theta;

        iDynTree::toEigen(rPl) =
                iDynTree::toEigen(rTranspose)*(iDynTree::toEigen(prevStep.position) - iDynTree::toEigen(newPosition));
    }
    return true;
}

bool UnicyclePlanner::getIntegratorSolution(double time, iDynTree::Vector2 &unicyclePosition, double& unicycleAngle) const
{
    static iDynTree::VectorDynSize stateBuffer(3);
    if(!m_integrator.getSolution(time,stateBuffer)){
        std::cerr << "Error while retrieving the integrator solution." << std::endl;
        return false;
    }
    unicyclePosition(0) = stateBuffer(0);
    unicyclePosition(1) = stateBuffer(1);
    unicycleAngle = stateBuffer(2);
    return true;
}

bool UnicyclePlanner::addTerminalStep(const iDynTree::Vector2& lastUnicyclePosition, double lastUnicycleAngle)
{
    std::shared_ptr<UnicycleFoot> stanceFoot, swingFoot;

    stanceFoot = m_swingLeft ? m_right : m_left;
    swingFoot = m_swingLeft ? m_left : m_right;

    Step prevStep;
    if (!(stanceFoot->getLastStep(prevStep)))
        return false;

    iDynTree::Vector2 rPl;

    if (!get_rPl(lastUnicyclePosition, lastUnicycleAngle, rPl))
        return false;

    double deltaAngle = std::abs(lastUnicycleAngle - prevStep.angle);
    double deltaTime = m_endTime - prevStep.impactTime;

    bool isTinyForStance = stanceFoot->isTinyStep(lastUnicyclePosition, lastUnicycleAngle);
    isTinyForStance = isTinyForStance && (deltaAngle < m_minAngle); //two steps will be taken, first by the swing leg and then by the stance leg to make the two feet parallel. Since the constriants on this second step are implicitly satisfied by the geometry of the unicycle, we focus in avoiding that this second step break the "tiny step" cosntraint. The constraints are checked for the first step.
    bool constraintsSatisfied = m_unicycleProblem.areConstraintsSatisfied(rPl, deltaAngle, deltaTime);
    bool isTinyForSwing = swingFoot->isTinyStep(lastUnicyclePosition, lastUnicycleAngle);
    isTinyForSwing = isTinyForSwing && (deltaAngle < m_minAngle);


    if (constraintsSatisfied && !isTinyForStance && !isTinyForSwing){
        if(!(swingFoot->addStepFromUnicycle(lastUnicyclePosition, lastUnicycleAngle, m_endTime + m_nominalTime))){
            std::cerr << "Error while adding terminal step." << std::endl;
            return false;
        }

        if(!(stanceFoot->addStepFromUnicycle(lastUnicyclePosition, lastUnicycleAngle, m_endTime + 2*m_nominalTime))){
            std::cerr << "Error while adding terminal step." << std::endl;
            return false;
        }
    } else {
        if (!constraintsSatisfied){
            std::cerr << "Unable to satisfy constraints on the last step. The following constraints are not satisfied:" << std::endl;
            m_unicycleProblem.printViolatedConstraints(rPl, deltaAngle, deltaTime);
        } else {
            iDynTree::Vector2 previousUnicycle;

            if(!stanceFoot->getUnicyclePositionFromFoot(prevStep.position, prevStep.angle, previousUnicycle)){
                return false;
            }
            double previousUnicycleAngle = prevStep.angle;
            isTinyForSwing = swingFoot->isTinyStep(previousUnicycle, previousUnicycleAngle);

            if (!isTinyForSwing){
                //std::cerr <<"Avoiding tiny step as last step. Setting feet parallel." << std::endl;
                if(!(swingFoot->addParallelStep(*(stanceFoot), m_endTime + m_nominalTime))){
                    std::cerr << "Error while adding terminal step." << std::endl;
                    return false;
                }
            } //else {
                //std::cerr <<"Avoiding tiny step as last step." << std::endl;
            //}
        }
    }

    return true;

}

UnicyclePlanner::UnicyclePlanner()
    :m_controller(std::make_shared<UnicyleController>())
    ,m_unicycle(std::make_shared<ControlledUnicycle>())
    ,m_integrator(m_unicycle)
    ,m_endTime(0.01)
    ,m_minTime(3)
    ,m_maxTime(10)
    ,m_nominalTime(4)
    ,m_dT(0.01)
    ,m_minAngle(iDynTree::deg2rad(5))
    ,m_nominalWidth(0.14)
    ,m_maxLength(0.2)
    ,m_minLength(0.05)
    ,m_maxAngle(iDynTree::deg2rad(45))
    ,m_addTerminalStep(true)
    ,m_startLeft(true)
    ,m_resetTimings(false)
    ,m_firstStep(false)
    ,m_left(nullptr)
    ,m_right(nullptr)
    ,m_swingLeft(true)
{
    m_unicycle->setController(m_controller);
    m_integrator.setMaximumStepSize(0.01);
    m_unicycleProblem.setMaxLength(0.20);
    m_unicycleProblem.setMinWidth(0.08);
    m_unicycleProblem.setMaxAngleVariation(iDynTree::deg2rad(45));
    m_unicycleProblem.setCostWeights(1.0, 2.5);
}

bool UnicyclePlanner::setDesiredPersonDistance(double xPosition, double yPosition)
{
    return m_controller->setPersonDistance(xPosition, yPosition);
}

bool UnicyclePlanner::setControllerGain(double controllerGain)
{
    return m_controller->setGain(controllerGain);
}

bool UnicyclePlanner::setSlowWhenTurnGain(double slowWhenTurnGain)
{
    return m_controller->setSlowWhenTurnGain(slowWhenTurnGain);
}

bool UnicyclePlanner::addDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2 &yDesired)
{
    TrajectoryPoint newPoint;
    newPoint.yDesired = yDesired;
    newPoint.yDotDesired.zero();
    newPoint.initTime = initTime;
    return m_controller->setDesiredPoint(newPoint);
}

bool UnicyclePlanner::addDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2 &yDesired, const iDynTree::Vector2 &yDotDesired)
{
    TrajectoryPoint newPoint;
    newPoint.yDesired = yDesired;
    newPoint.yDotDesired = yDotDesired;
    newPoint.initTime = initTime;
    return m_controller->setDesiredPoint(newPoint);
}

void UnicyclePlanner::clearDesiredTrajectory()
{
    m_controller->clearDesiredTrajectory();
}

bool UnicyclePlanner::clearDesiredTrajectoryUpTo(double time)
{
    return m_controller->clearDesiredTrajectoryUpTo(time);
}

bool UnicyclePlanner::setEndTime(double endTime)
{
    if (endTime < 0){
        std::cerr << "The endTime is supposed to be non-negative." <<std::endl;
        return false;
    }
    m_endTime = endTime;
    return true;
}

bool UnicyclePlanner::setMaximumIntegratorStepSize(double dT)
{
    return m_integrator.setMaximumStepSize(dT);
}

bool UnicyclePlanner::setMaxStepLength(double maxLength)
{
    if(m_unicycleProblem.setMaxLength(maxLength)){
        m_maxLength = maxLength;
        return true;
    }
    return false;
}

bool UnicyclePlanner::setMinStepWidth(double minWidth)
{
    return m_unicycleProblem.setMinWidth(minWidth);
}

bool UnicyclePlanner::setMaxAngleVariation(double maxAngleInRad)
{
    if (m_unicycleProblem.setMaxAngleVariation(maxAngleInRad)){
        m_maxAngle = maxAngleInRad;
        return true;
    }
    return false;
}

bool UnicyclePlanner::setCostWeights(double positionWeight, double timeWeight)
{
    return m_unicycleProblem.setCostWeights(positionWeight, timeWeight);
}

bool UnicyclePlanner::setStepTimings(double minTime, double maxTime, double nominalTime)
{
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
    if (dT < 0){
        std::cerr << "The planner period is supposed to be non-negative." << std::endl;
        return false;
    }

    m_dT = dT;

    return true;
}

bool UnicyclePlanner::setMinimumAngleForNewSteps(double minAngleInRad)
{
    if (minAngleInRad < 0){
        std::cerr << "The minimum angle is supposed to be non-negative." << std::endl;
        return false;
    }

    m_minAngle = minAngleInRad;

    return true;
}

bool UnicyclePlanner::setMinimumStepLength(double minLength)
{
    if (minLength < 0){
        std::cerr << "The minimum length is supposed to be non-negative." << std::endl;
        return false;
    }
    m_minLength = minLength;
    return true;
}

bool UnicyclePlanner::setNominalWidth(double nominalWidth)
{
    if (nominalWidth < 0){
        std::cerr << "The nominal width is supposed to be non-negative." << std::endl;
        return false;
    }

    m_nominalWidth = nominalWidth;

    return true;
}

bool UnicyclePlanner::setWidthSetting(double minWidth, double nominalWidth)
{
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
    m_addTerminalStep = addStep;
}

void UnicyclePlanner::startWithLeft(bool startLeft)
{
    m_startLeft = startLeft;
}

void UnicyclePlanner::resetTimingsIfStill(bool resetTimings)
{
    m_resetTimings = resetTimings;
}

bool UnicyclePlanner::computeNewSteps(std::shared_ptr< FootPrint > leftFoot, std::shared_ptr< FootPrint > rightFoot, double initTime)
{
    if (!leftFoot || !rightFoot){
        std::cerr <<"Empty feet pointers."<<std::endl;
        return false;
    }

    if (m_nominalWidth > m_maxLength){
        std::cerr << "Error: the maxLength parameter seems to be too restrictive. Notice that it represents the cartesian distance between the two feet (width included)." <<std::endl;
        return false;
    }

    if (m_minAngle >= m_maxAngle) {
        std::cerr << "Error: the minAngle parameter is supposed to be lower than the maxAngle. Otherwise, when the feet start parallel, it would be impossible to rotate the swing foot." << std::endl;
        return false;
    }

    m_left.reset(new UnicycleFoot(leftFoot));
    m_right.reset(new UnicycleFoot(rightFoot));

    double maxVelocity = std::sqrt(std::pow(m_maxLength,2) - std::pow(m_nominalWidth,2))/m_minTime * 0.90;
    double maxAngVelocity = m_maxAngle/m_minTime*0.70;
    if (!m_controller->setSaturations(maxVelocity, maxAngVelocity))
        return false;

    if (!initializePlanner(initTime)){
        std::cerr << "Error during planner initialization." <<std::endl;
        return false;
    }

    double cost, minCost = 1e10; //this big number is actually not necessary since there is a check on tOptim also
    double deltaTime = 0, pauseTime = 0;
    double deltaAngle = 0;
    iDynTree::Vector2 unicyclePosition, rPl;
    rPl.zero();
    double unicycleAngle;
    std::shared_ptr<UnicycleFoot> stanceFoot, swingFoot;

    size_t numberOfStepsLeft = leftFoot->numberOfSteps();
    size_t numberOfStepsRight = rightFoot->numberOfSteps();

    stanceFoot = m_swingLeft ? m_right : m_left;
    swingFoot = m_swingLeft ? m_left : m_right;

    Step prevStep;
    if (!(stanceFoot->getLastStep(prevStep)))
        return false;

    double t = initTime, tOptim = -1.0;
    pauseTime = t - prevStep.impactTime;

    double timeOffset;

    if (m_firstStep){
        timeOffset = m_nominalTime;
    } else {
        timeOffset = 0;
    }

    bool pauseActivated = false;
    bool avoidPause = false;

    while (t <= m_endTime){
        deltaTime = t - prevStep.impactTime;

        if(!getIntegratorSolution(t, unicyclePosition, unicycleAngle)){
            return false;
        }

        deltaAngle = std::abs(prevStep.angle - unicycleAngle);

        if ((deltaTime >= m_minTime) && (t > (initTime + timeOffset))){ //The step is not too fast
            if (avoidPause || (!(swingFoot->isTinyStep(unicyclePosition, unicycleAngle)) || (deltaAngle > m_minAngle))){ //the step is not tiny
                pauseActivated = false;
                deltaTime -= pauseTime;
                if ((deltaTime > m_maxTime) || (t == m_endTime)){ //the step is not too slow
                    if (tOptim > 0){  //a feasible point has been found

                        if(!getIntegratorSolution(tOptim, unicyclePosition, unicycleAngle)){
                            return false;
                        }

                        if(!swingFoot->addStepFromUnicycle(unicyclePosition, unicycleAngle, tOptim)){
                            std::cerr << "Error while inserting new step." << std::endl;
                            return false;
                        }

                        t = tOptim;
                        pauseTime = 0;
                        avoidPause = false;

                    } else { //if (tOptim > 0)
                        if (pauseActivated) {
                            std::cerr << "Ignoring pause conditions to try to satisfy constraints." << std::endl;
                            avoidPause = true;
                            t = std::max(initTime, t - pauseTime);
                        } else {

                            std::cerr << "Unable to satisfy constraints given the specified maxStepTime. In the last evaluation of constraints, the following where not satisfied:" << std::endl;

                            m_unicycleProblem.printViolatedConstraints(rPl, deltaAngle, deltaTime);

                            if(!stanceFoot->getUnicyclePositionFromFoot(prevStep.position, prevStep.angle, unicyclePosition)){
                                std::cerr << "Error while computing the unicycle position." << std::endl;
                                return false;
                            }

                            bool isTiny = swingFoot->isTinyStep(unicyclePosition, prevStep.angle);
                            //bool isTiny = false;

                            if(!isTiny){
                                if(!swingFoot->addParallelStep(*(stanceFoot), t)){
                                    std::cerr << "Error while inserting new step." << std::endl;
                                    return false;
                                }

                                pauseTime = 0;
                            } else {

                                pauseTime += m_nominalTime;
                            }

                            Step lastSwingStep;
                            if (!(swingFoot->getLastStep(lastSwingStep))){
                                std::cerr << "Error getting last swing step" << std::endl;
                                return false;
                            }
                            t = std::max(prevStep.impactTime, lastSwingStep.impactTime) + m_nominalTime + pauseTime;

                            if (t < initTime) {
                                std::cerr << "Something went wrong when updating time variable." << std::endl;
                                return false;
                            }
                        }

                    }
                    //reset
                    tOptim = -1.0;
                    m_firstStep = false;
                    timeOffset = 0;

                    m_swingLeft = !m_swingLeft;
                    stanceFoot = m_swingLeft ? m_right : m_left;
                    swingFoot = m_swingLeft ? m_left : m_right;

                    if (!(stanceFoot->getLastStep(prevStep))){
                        std::cerr << "Error updating last step" << std::endl;
                        return false;
                    }

                } else { //if ((deltaTime > m_maxTime) || (t == m_endTime)) //here I need to find the best solution

                    if(!get_rPl(unicyclePosition, unicycleAngle, rPl)){
                        std::cerr << "Error while retrieving the relative position" << std::endl;
                        return false;
                    }

                    if((deltaTime > 0) && (m_unicycleProblem.areConstraintsSatisfied(rPl, deltaAngle, deltaTime))){ //constraints are satisfied

                        if(!m_unicycleProblem.getCostValue(rPl, deltaAngle, deltaTime, cost)){
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
                pauseActivated = true;
            }
        }

        if (((t+m_dT) > m_endTime) && (t < m_endTime)){
            t = m_endTime;
        } else {
            t += m_dT;
        }
    }

    if (m_addTerminalStep){

        if(!getIntegratorSolution(m_endTime, unicyclePosition, unicycleAngle)){
            return false;
        }

        if(!addTerminalStep(unicyclePosition, unicycleAngle)){
            std::cerr << "Error while adding the terminal step." << std::endl;
            return false;
        }

        if (m_resetTimings && (numberOfStepsLeft == leftFoot->numberOfSteps()) && (numberOfStepsRight == rightFoot->numberOfSteps())){ //no steps have been added
            Step lastLeft, lastRight;

            leftFoot->getLastStep(lastLeft);
            rightFoot->getLastStep(lastRight);

            if (lastLeft.impactTime > lastRight.impactTime){
                lastRight.impactTime = lastLeft.impactTime;
                rightFoot->clearLastStep();
                rightFoot->addStep(lastRight);
            } else if (lastLeft.impactTime < lastRight.impactTime){
                lastLeft.impactTime = lastRight.impactTime;
                leftFoot->clearLastStep();
                leftFoot->addStep(lastLeft);
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
    double unicycleAngle;
    iDynTree::Vector2 unicyclePosition;

    if(!getIntegratorSolution(time, unicyclePosition, unicycleAngle)){
        std::cerr << "Time is out of range." << std::endl;
        return false;
    }
    personPosition = m_controller->getPersonPosition(unicyclePosition, unicycleAngle);
    return true;
}


