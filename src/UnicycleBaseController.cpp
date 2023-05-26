/*
 * Copyright (C) 20022 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <UnicycleBaseController.h>
#include <iostream>
#include <cmath>

double UnicycleBaseController::saturate(double input, double saturation)
{
    return saturate(input, saturation, -saturation);
}

double UnicycleBaseController::saturate(double input, double positiveSaturation, double negativeSaturation)
{
    if ((positiveSaturation < 0) || (negativeSaturation > positiveSaturation))
        return input;

    if (input > positiveSaturation)
        return positiveSaturation;
    else if (input < negativeSaturation)
        return negativeSaturation;
    else return input;
}

UnicycleBaseController::UnicycleBaseController()
    : iDynTree::optimalcontrol::Controller(3)
    ,m_maxLinearVelocity(-1) //negative value -> don't saturate
    ,m_maxAngularVelocity(-1) //negative value -> don't saturate
    ,m_slowWhenTurnGain(0.0)
    ,m_slowWhenBackwardFactor(1.0)
    ,m_slowWhenSidewaysFactor(1.0)
{ }

bool UnicycleBaseController::doControl(iDynTree::VectorDynSize &controllerOutput)
{
    if(controllerOutput.size() != this->controlSpaceSize())
        controllerOutput.resize(this->controlSpaceSize());


    if (!doUnicycleControl(controllerOutput(0), controllerOutput(1), controllerOutput(2)))
    {
        return false;
    }

    controllerOutput(1) = saturate(controllerOutput(1), m_maxAngularVelocity);
    double velocitySaturation = m_maxLinearVelocity / (1.0 + m_slowWhenTurnGain * (std::sqrt(controllerOutput(1) * controllerOutput(1))));
    controllerOutput(0) = saturate(controllerOutput(0), velocitySaturation, -m_slowWhenBackwardFactor * velocitySaturation);
    controllerOutput(2) = saturate(controllerOutput(2), m_slowWhenSidewaysFactor * m_maxLinearVelocity);

    return true;
}

bool UnicycleBaseController::setStateFeedback(const double t, const iDynTree::VectorDynSize &stateFeedback)
{
    if (stateFeedback.size() != 3)
        return false;

    iDynTree::Vector2 unicyclePosition;
    double unicycleOrientation;

    unicycleOrientation = stateFeedback(2);

    unicyclePosition(0) = stateFeedback(0);
    unicyclePosition(1) = stateFeedback(1);

    return setUnicycleStateFeedback(t, unicyclePosition, unicycleOrientation);

}

bool UnicycleBaseController::setSaturations(double maxLinearVelocity, double maxAngularVelocity)
{
    if ((maxLinearVelocity < 0)||(maxAngularVelocity < 0)){
        std::cerr << "The saturations are on the absolute value, thus they need to be non-negative." << std::endl;
        return false;
    }

    m_maxLinearVelocity = maxLinearVelocity;
    m_maxAngularVelocity = maxAngularVelocity;

    return true;
}

bool UnicycleBaseController::setSlowWhenTurnGain(double slowWhenTurnGain)
{
    if (slowWhenTurnGain < 0){
        std::cerr << "The slowWhenTurn gain is supposed to be non-negative." << std::endl;
        return false;
    }

    m_slowWhenTurnGain = slowWhenTurnGain;
    return true;
}

bool UnicycleBaseController::setSlowWhenBackwardFactor(double slowWhenBackwardFactor)
{
    if (slowWhenBackwardFactor < 0){
        std::cerr << "The slowWhenBackwardFactor multiplier is supposed to be non-negative." << std::endl;
        return false;
    }

    m_slowWhenBackwardFactor = slowWhenBackwardFactor;
    return true;
}

bool UnicycleBaseController::setSlowWhenSidewaysFactor(double slowWhenSidewaysFactor)
{
    if (slowWhenSidewaysFactor < 0){
        std::cerr << "The slowWhenSidewaysFactor multiplier is supposed to be non-negative." << std::endl;
        return false;
    }

    m_slowWhenSidewaysFactor = slowWhenSidewaysFactor;
    return true;
}

bool UnicycleBaseController::getSaturationLimits(double& maxForwardSpeed, double& maxLateralVelocity, double& maxAngularVelocity)
{
    maxForwardSpeed = m_maxLinearVelocity;
    maxLateralVelocity = m_maxLinearVelocity * m_slowWhenSidewaysFactor;
    maxAngularVelocity = m_maxAngularVelocity * m_slowWhenTurnGain;
    return true;
}
