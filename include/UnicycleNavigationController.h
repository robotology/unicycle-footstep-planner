/*
 * Copyright (C) 20023 Fondazione Istituto Italiano di Tecnologia
 * Authors: Simone Micheletti
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLE_NAVIGATION_CONTROLLER_H
#define UNICYCLE_NAVIGATION_CONTROLLER_H

#include "UnicycleBaseController.h"
#include "UnicycleState.h"
#include <vector>

class UnicycleNavigationController : public UnicycleBaseController
{
private:
    double m_maxVelocity, m_maxLateralVelocity, m_maxAngularVelocity;   //saturation limits
    double m_desiredForwardSpeed, m_desiredAngularVelocity, m_desiredLateralVelocity;
    double m_time;
    double m_deactivationEndTime;

    std::vector<UnicycleState> m_navigationPath;
    UnicycleState m_state;
    //
    int m_poseIndex;    //current next pose of the path yet to be reached
    double m_ETA;   //Time expected to reach the next pose in the path

    //Motion Parameters
    double m_linearETA; //Relative Time needed to move in x, y
    double m_linearSpeed;   //Absolute speed on the segment connecting the two path poses
    double m_cosSlope, m_sinSlope;  //projection component of the conjunction of the two poses on the local frame

    bool computeDesiredVelocities();

    bool computeMotionParameters();

public:

    UnicycleNavigationController();

    virtual bool doUnicycleControl(double& forwardSpeed, double& angularVelocity, double& lateralVelocity) override;

    virtual bool setUnicycleStateFeedback(const double t, const iDynTree::Vector2& position, double angle) override;

    void setConstantControl(double forwardSpeed, double angularVelocity, double lateralVelocity);

    void setInactiveUntil(double endTime);

    double getDesiredLateralVelocity() const;

    bool setNavigationPath(std::vector<UnicycleState>& path);

    bool setMaxVelocities(double & maxVelocity, double & maxLateralVelocity, double & maxAngularVelocity);

};

#endif // UNICYCLE_NAVIGATION_CONTROLLER_H