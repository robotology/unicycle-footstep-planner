/*
 * Copyright (C) 20022 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLEDIRECTCONTROLLER_H
#define UNICYCLEDIRECTCONTROLLER_H

#include "UnicycleBaseController.h"

class UnicycleDirectController : public UnicycleBaseController
{

    double m_desiredForwardSpeed, m_desiredAngularVelocity, m_desiredLateralVelocity;
    double m_time;
    double m_deactivationEndTime;

public:

    UnicycleDirectController();

    virtual bool doUnicycleControl(double& forwardSpeed, double& angularVelocity, double& lateralVelocity);

    virtual bool setUnicycleStateFeedback(const double t, const iDynTree::Vector2& , double );

    void setConstantControl(double forwardSpeed, double angularVelocity, double lateralVelocity);

    void setInactiveUntil(double endTime);

};

#endif // UNICYCLEDIRECTCONTROLLER_H
