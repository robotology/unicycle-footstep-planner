/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLEPLANNER_H
#define UNICYCLEPLANNER_H

#include "ControlledUnicycle.h"
#include "UnicycleController.h"
#include "UnicycleOptimization.h"
#include "UnicycleFoot.h"
#include <iDynTree/Integrators/RK4.h>
#include <memory>

class UnicyclePlanner {
    std::shared_ptr<UnicyleController> m_controller;
    std::shared_ptr<ControlledUnicycle> m_unicycle;
    iDynTree::optimalcontrol::integrators::RK4 m_integrator;
    UnicycleOptimization m_unicycleProblem;
    double m_endTime, m_minTime, m_maxTime, m_nominalTime, m_dT, m_minAngle, m_nominalWidth, m_maxLength, m_minLength, m_maxAngle;
    bool m_addTerminalStep, m_startLeft, m_resetStartingFoot, m_firstStep;

    std::shared_ptr<UnicycleFoot> m_left, m_right;

    //state
    bool m_swingLeft;

    bool getInitialStateFromFeet(double initTime);

    bool initializePlanner(double initTime);

    bool get_rPl(const iDynTree::Vector2 &unicyclePosition, double unicycleAngle, iDynTree::Vector2 &rPl); //depending on left and right foot and on swing_left

    bool getIntegratorSolution(double time, iDynTree::Vector2& unicyclePosition, double &unicycleAngle) const;

    bool addTerminalStep(const iDynTree::Vector2 &lastUnicyclePosition, double lastUnicycleAngle);

public:

    UnicyclePlanner();

    //Controller inputs
    bool setDesiredPersonDistance(double xPosition, double yPosition);

    bool setControllerGain(double controllerGain); //optional

    bool setSlowWhenTurnGain(double slowWhenTurnGain); //if >0 the unicycle progress more slowly when also turning.

    bool addDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2& yDesired, const iDynTree::Vector2& yDotDesired); //If two points have the same initTime it is an undefined behavior. It keeps the desired values constant from initTime to the initTime of the next desired point (they are automatically ordered)

    bool addDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2& yDesired);// like the above but assumes zero velocity

    void clearDesiredTrajectory();

    bool clearDesiredTrajectoryUpTo(double time);

    //Integrator inputs
    bool setEndTime(double endTime);

    bool setMaximumIntegratorStepSize(double dT);

    //Constraints
    bool setMaxStepLength(double maxLength);

    [[deprecated("use the method setWidthSettings instead.")]]
    bool setMinStepWidth(double minWidth);

    bool setMaxAngleVariation(double maxAngleInRad); //in radians!

    //Cost
    bool setCostWeights(double positionWeight, double timeWeight);

    //Sampler
    bool setStepTimings(double minTime, double maxTime, double nominalTime);

    bool setPlannerPeriod(double dT);

    bool setMinimumAngleForNewSteps(double minAngleInRad); //in radians //to rotate in place

    bool setMinimumStepLength(double minLength);

    [[deprecated("use the method setWidthSettings instead.")]]
    bool setNominalWidth(double nominalWidth);

    bool setWidthSetting(double minWidth, double nominalWidth);

    void addTerminalStep(bool addStep);

    void startWithLeft(bool startLeft);

    [[deprecated("timings will always be resetted. User can chose whether resetting also the foot or not. Use the method resetStartingFootIfStill")]]
    void resetTimingsIfStill(bool resetTimings) {
        resetStartingFootIfStill(resetTimings);
    }

    void resetStartingFootIfStill(bool resetStartingFoot);

    bool computeNewSteps(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime); //if the inputs are empty, the initTime is obtained from the first trajectory point, otherwise the initTime is the latest impactTime

    bool startWithLeft() const;

    bool getPersonPosition(double time, iDynTree::Vector2 &personPosition);
};

#endif // UNICYCLEPLANNER_H
