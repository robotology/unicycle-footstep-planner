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
    const double m_zeroTolerance = 1E-6;                                                //numerical threshold below which we clip velocities to zero
    double m_maxVelocity, m_maxLateralVelocity, m_maxAngularVelocity;                   //saturation limits
    double m_desiredForwardSpeed, m_desiredAngularVelocity, m_desiredLateralVelocity;   //controller outputs
    double m_time;                                                                      //current time of the unicycle feedback
    double m_dt = 0.01;                                                                 //time step of the integrator
    double m_deactivationEndTime;                                                       //initial time that the unicycle stays still

    std::vector<UnicycleState> m_navigationPath;    //input path from the navigation stack
    UnicycleState m_state;                          //state of the unicycle from feedback

    int m_poseIndex;            //current next pose of the path that has to be reached
    double m_ETA;               //Time expected to reach the next pose in the path
    bool m_newPathReceived;     //Flag expessing that a new path has been received

    bool computeDesiredVelocities();

public:

    UnicycleNavigationController();

    virtual bool doUnicycleControl(double& forwardSpeed, double& angularVelocity, double& lateralVelocity) override;

    virtual bool setUnicycleStateFeedback(const double t, const iDynTree::Vector2& position, double angle) override;

    void setConstantControl(double forwardSpeed, double angularVelocity, double lateralVelocity);

    void setInactiveUntil(double endTime);

    double getDesiredLateralVelocity() const;

    bool setNavigationPath(const std::vector<UnicycleState>& path);

    bool setMaxVelocities(double & maxVelocity, double & maxLateralVelocity, double & maxAngularVelocity);

    bool setTimeStep(double &dT);

};

#endif // UNICYCLE_NAVIGATION_CONTROLLER_H