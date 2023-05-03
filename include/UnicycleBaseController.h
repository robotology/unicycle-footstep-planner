/*
 * Copyright (C) 20022 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLEBASECONTROLLER_H
#define UNICYCLEBASECONTROLLER_H

#include <iDynTree/Controller.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>

class UnicycleBaseController : public iDynTree::optimalcontrol::Controller{

    double m_maxLinearVelocity, m_maxAngularVelocity;
    double m_slowWhenTurnGain;
    double m_slowWhenBackwardFactor;
    double m_slowWhenSidewaysFactor;

    double saturate(double input, double saturation);

    double saturate(double input, double positiveSaturation, double negativeSaturation);

public:

    UnicycleBaseController();

    virtual bool doUnicycleControl(double& forwardSpeed, double& angularVelocity, double& lateralVelocity) = 0;

    virtual bool setUnicycleStateFeedback(const double t, const iDynTree::Vector2& unicyclePosition, double unicycleOrientation) = 0;

    bool doControl(iDynTree::VectorDynSize &controllerOutput) final;

    bool setStateFeedback(const double t, const iDynTree::VectorDynSize &stateFeedback) final;

    bool setSaturations(double maxLinearVelocity, double maxAngularVelocity);

    bool setSlowWhenTurnGain(double slowWhenTurnGain); //if >0 the unicycle progress more slowly when also turning.

    bool setSlowWhenBackwardFactor(double slowWhenBackwardFactor); //if >0 the unicycle progress more slowly when going backward. It is a multiplicative gain

    bool setSlowWhenSidewaysFactor(double slowWhenSidewaysFactor); //if >0 the unicycle progress more slowly when going backward. It is a multiplicative gain

    bool getSaturationLimits(double& maxForwardSpeed, double& maxLateralVelocity, double& maxAngularVelocity);
};

#endif // UNICYCLEBASECONTROLLER_H
