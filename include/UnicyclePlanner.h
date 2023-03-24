/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLEPLANNER_H
#define UNICYCLEPLANNER_H

#include "ControlledUnicycle.h"
#include "PersonFollowingController.h"
#include "UnicycleDirectController.h"
#include "UnicycleOptimization.h"
#include "UnicycleFoot.h"
#include "FreeSpaceEllipse.h"
#include <iDynTree/Integrators/ForwardEuler.h>
#include <memory>
#include <mutex>

enum class FreeSpaceEllipseMethod
{
    REFERENCE_ONLY,
    FOOTSTEPS_ONLY,
    REFERENCE_AND_FOOTSTEPS
};

enum class UnicycleController
{
    PERSON_FOLLOWING,
    DIRECT
};

class UnicyclePlanner {
    std::shared_ptr<PersonFollowingController> m_personFollowingController;
    std::shared_ptr<UnicycleDirectController> m_directController;
    //UnicycleController m_currentController; moved to public
    std::shared_ptr<ControlledUnicycle> m_unicycle;
    iDynTree::optimalcontrol::integrators::ForwardEuler m_integrator;
    UnicycleOptimization m_unicycleProblem;
    double m_initTime, m_endTime, m_minTime, m_maxTime, m_nominalTime, m_dT, m_minAngle, m_nominalWidth, m_maxLength, m_minLength, m_maxAngle;
    bool m_addTerminalStep, m_startLeft, m_resetStartingFoot, m_firstStep;
    FreeSpaceEllipseMethod m_freeSpaceMethod;
    double m_leftYawOffset, m_rightYawOffset;
    double m_linearVelocityConservativeFactor, m_angularVelocityConservativeFactor;
    std::mutex m_mutex;
    std::vector<UnicycleState> m_inputPath;

    std::shared_ptr<UnicycleFoot> m_left, m_right;

    struct PoseStamped
    {
        UnicycleState pose;
        double time;
    };

    std::vector<PoseStamped> m_integratedPath;

    //state
    bool m_swingLeft;

    bool getInitialStateFromFeet(double initTime);

    bool initializePlanner(double initTime);

    bool get_rPl(const UnicycleState &unicycleState, iDynTree::Vector2 &rPl); //depending on left and right foot and on swing_left

    bool getIntegratorSolution(double time, UnicycleState &unicycleState) const;

    bool addTerminalStep(const UnicycleState &lastUnicycleState);

    bool checkConstraints(iDynTree::Vector2 _rPl, double deltaAngle, double deltaTime, iDynTree::Vector2 newFootPosition, iDynTree::Vector2 prevStep);

public:

    UnicycleController m_currentController;

    UnicyclePlanner();

    //Controller inputs
    bool setDesiredPersonDistance(double xPosition, double yPosition);

    [[deprecated("Use setPersonFollowingControllerGain instead.")]]
    bool setControllerGain(double controllerGain); //optional

    bool setPersonFollowingControllerGain(double controllerGain); //optional

    bool setSlowWhenTurnGain(double slowWhenTurnGain); //if >0 the unicycle progress more slowly when also turning.

    bool setSlowWhenBackwardFactor(double slowWhenBackwardFactor); //if >0 the unicycle progress more slowly when going backward. It is a multiplicative gain

    bool setSlowWhenSidewaysFactor(double slowWhenSidewaysFactor); //if >0 the unicycle progress more slowly when going sideways. It is a multiplicative gain

    [[deprecated("Use addPersonFollowingDesiredTrajectoryPoint instead.")]]
    bool addDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2& yDesired, const iDynTree::Vector2& yDotDesired);

    bool addPersonFollowingDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2& yDesired, const iDynTree::Vector2& yDotDesired); //If two points have the same initTime it is an undefined behavior. It keeps the desired values constant from initTime to the initTime of the next desired point (they are automatically ordered)

    [[deprecated("Use addPersonFollowingDesiredTrajectoryPoint instead.")]]
    bool addDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2& yDesired);// like the above but assumes zero velocity

    bool addPersonFollowingDesiredTrajectoryPoint(double initTime, const iDynTree::Vector2& yDesired);// like the above but assumes zero velocity

    [[deprecated("Use clearPersonFollowingDesiredTrajectory instead.")]]
    void clearDesiredTrajectory();

    void clearPersonFollowingDesiredTrajectory();

    [[deprecated("Use clearPersonFollowingDesiredTrajectoryUpTo instead.")]]
    bool clearDesiredTrajectoryUpTo(double time);

    bool clearPersonFollowingDesiredTrajectoryUpTo(double time);

    void setDesiredDirectControl(double forwardVelocity, double angularVelocity, double lateralVelocity);

    bool setSaturationsConservativeFactors(double linearVelocityConservativeFactor, double angularVelocityConservativeFactor);

    //Integrator inputs
    bool setMaximumIntegratorStepSize(double dT);

    //Constraints
    bool setMaxStepLength(double maxLength);

    bool setMaxAngleVariation(double maxAngleInRad); //in radians!

    //Cost
    bool setCostWeights(double positionWeight, double timeWeight);

    //Sampler
    bool setStepTimings(double minTime, double maxTime, double nominalTime);

    bool setPlannerPeriod(double dT);

    bool setMinimumAngleForNewSteps(double minAngleInRad); //in radians //to rotate in place

    bool setMinimumStepLength(double minLength);

    bool setWidthSetting(double minWidth, double nominalWidth);

    void addTerminalStep(bool addStep);

    void startWithLeft(bool startLeft);

    void resetStartingFootIfStill(bool resetStartingFoot);

    void setLeftFootYawOffsetInRadians(double leftYawOffsetInRadians);

    void setRightFootYawOffsetInRadians(double rightYawOffsetInRadians);

    bool computeNewSteps(std::shared_ptr<FootPrint> leftFoot, std::shared_ptr<FootPrint> rightFoot, double initTime, double endTime); //if the inputs are empty, the initTime is obtained from the first trajectory point, otherwise the initTime is the latest impactTime

    bool startWithLeft() const;

    bool getPersonPosition(double time, iDynTree::Vector2 &personPosition);

    void setFreeSpaceEllipseMethod(FreeSpaceEllipseMethod method);

    bool setFreeSpaceEllipse(const FreeSpaceEllipse& freeSpaceEllipse);

    bool setFreeSpaceEllipseConservativeFactor(double conservativeFactor); //Used only to saturate the references

    bool setInnerFreeSpaceEllipseOffset(double offset);

    bool setInnerFreeSpaceEllipseOffsets(double semiMajorAxisOffset, double semiMinorAxisOffset);

    bool setUnicycleController(UnicycleController controller);

    bool interpolateNewStepsFromPath(std::shared_ptr< FootPrint > leftFoot, std::shared_ptr< FootPrint > rightFoot, double initTime, double endTime);

    bool setInputPath (std::vector<UnicycleState> input);
};

#endif // UNICYCLEPLANNER_H
