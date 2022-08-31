/*
 * Copyright (C) 20022 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <UnicycleDirectController.h>


UnicycleDirectController::UnicycleDirectController()
    : m_desiredForwardSpeed(0.0)
    , m_desiredAngularVelocity(0.0)
    , m_desiredLateralVelocity(0.0)
    , m_time(0.0)
    , m_deactivationEndTime(0.0)
{

}

bool UnicycleDirectController::doUnicycleControl(double &forwardSpeed, double &angularVelocity, double &lateralVelocity)
{

    if (m_time < m_deactivationEndTime)
    {
        forwardSpeed = 0.0;
        angularVelocity = 0.0;
        lateralVelocity = 0.0;

        return true;
    }

    forwardSpeed = m_desiredForwardSpeed;
    angularVelocity = m_desiredAngularVelocity;
    lateralVelocity = m_desiredLateralVelocity;

    return true;
}

bool UnicycleDirectController::setUnicycleStateFeedback(const double t, const iDynTree::Vector2 &, double)
{
    m_time = t;
    return true;
}

void UnicycleDirectController::setConstantControl(double forwardSpeed, double angularVelocity, double lateralVelocity)
{
    m_desiredForwardSpeed = forwardSpeed;
    m_desiredAngularVelocity = angularVelocity;
    m_desiredLateralVelocity = lateralVelocity;
}

void UnicycleDirectController::setInactiveUntil(double endTime)
{
    m_deactivationEndTime = endTime;
}
