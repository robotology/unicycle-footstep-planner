/*
 * Copyright (C) 20023 Fondazione Istituto Italiano di Tecnologia
 * Authors: Simone Micheletti
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <UnicycleNavigationController.h>
#include <cmath>
#include <iostream>

UnicycleNavigationController::UnicycleNavigationController()
    : m_desiredForwardSpeed(0.0)
    , m_desiredAngularVelocity(0.0)
    , m_desiredLateralVelocity(0.0)
    , m_time(0.0)
    , m_deactivationEndTime(0.0)
    , m_poseIndex(1)
{

}

bool UnicycleNavigationController::doUnicycleControl(double &forwardSpeed, double &angularVelocity, double &lateralVelocity)
{
    if (m_time < m_deactivationEndTime)
    {
        forwardSpeed = 0.0;
        angularVelocity = 0.0;
        lateralVelocity = 0.0;

        return true;
    }
    
    computeDesiredVelocities();

    forwardSpeed = m_desiredForwardSpeed;
    angularVelocity = m_desiredAngularVelocity;
    lateralVelocity = m_desiredLateralVelocity;

    return true;
}

bool UnicycleNavigationController::setUnicycleStateFeedback(const double t, const iDynTree::Vector2 & position, double angle)
{
    m_time = t;
    m_state.position = position;
    //Handle angle periodicity for continuous angles bigger than 2pi
    if (angle > (2*M_PI))
    {
        m_state.angle = angle - ((int)(angle/(2*M_PI))) * 2*M_PI; //use int truncation towards 0
    }
    else if(angle < -(2*M_PI))
    {
        m_state.angle = angle + ((int)(angle/(2*M_PI))) * 2*M_PI;
    }
    else
    {
        m_state.angle = angle;
    }
    return true;
}

bool UnicycleNavigationController::computeDesiredVelocities()
{
    //Next Pose Check: has to be done only when we don't have a new path received, otherwise we will skip the first pose
    if (!m_newPathReceived)
    {
        //Skip this if I have just received a new path
        if (m_ETA < m_dt)
        {
            if (m_poseIndex == m_navigationPath.size() - 1)
            {
                m_desiredForwardSpeed = 0.0;
                m_desiredAngularVelocity = 0.0;
                m_desiredLateralVelocity = 0.0;
                return true;
            }
            ++m_poseIndex;
        }
    }
    else
    {
        m_newPathReceived = false;
    }

    //Computing the direction towards the next pose
    double distance = std::sqrt(pow(m_navigationPath[m_poseIndex].position(0) - m_state.position(0), 2) +
                           pow(m_navigationPath[m_poseIndex].position(1) - m_state.position(1), 2) 
                           );   // faster than hypot but without overflow check
    double slopeAngle = atan2(m_navigationPath[m_poseIndex].position(1) - m_state.position(1), 
                        m_navigationPath[m_poseIndex].position(0) - m_state.position(0));            //projection component of the conjunction of the two poses on the local frame
    double cosSlope = cos(slopeAngle);    
    double sinSlope = sin(slopeAngle); 

    //Project each speed component on the segment connecting the two poses
    //Let's find the linear speed on the connection of the two path poses
    double x_projectionSpeed, y_projectionSpeed;
    double linearSpeed = 0.0;     //Absolute speed on the segment connecting the two path poses
    if (std::abs(cosSlope) < m_zeroTolerance)
    {   //this means that we are moving purely sideways
        linearSpeed = m_maxLateralVelocity;    
    }
    else if (std::abs(sinSlope) < m_zeroTolerance)
    {   //this means that we are moving purely in front
        linearSpeed = m_maxVelocity;
    }
    else
    {   //diagonal motion
        x_projectionSpeed = m_maxVelocity/std::abs(cosSlope);
        y_projectionSpeed = m_maxLateralVelocity/std::abs(sinSlope);
        //The smallest saturation gives the limit to the module of the linear speed
        if (x_projectionSpeed < y_projectionSpeed)
        {
            linearSpeed = x_projectionSpeed;
        }
        else
        {
            linearSpeed = y_projectionSpeed;
        }
    }

    double angleDifference = m_navigationPath[m_poseIndex].angle - m_state.angle;
    //ETAs computation
    double linearETA = distance / linearSpeed;  //Time required for moving from (x_i, y_i) to (x_i+1, y_i+1)
    double absoluteAngleDifference = std::abs(angleDifference); 
    if (absoluteAngleDifference >= M_PI)
    {
        absoluteAngleDifference = std::abs(absoluteAngleDifference - 2*M_PI);
    }
    double angularETA = absoluteAngleDifference / m_maxAngularVelocity;   //Time required for moving from theta_i to theta_i+1

    if (angularETA > linearETA)
    {
        m_ETA = angularETA;
    }
    else
    {
        m_ETA = linearETA;
    }
    
    int angleDirection;
    
    //Since the angles vary between (-pi, pi]
    if (angleDifference > M_PI)
    {
        angleDirection = -1;
    }
    else if (angleDifference >= 0 && angleDifference <= M_PI)
    {
        angleDirection = 1;
    }
    else if (angleDifference <0 && angleDifference >= -M_PI)
    {
        angleDirection = -1;
    }
    else    // angleDifference < -M_PI
    {
        angleDirection = 1;
    }

    //Speed computation 
    angularETA = absoluteAngleDifference / m_maxAngularVelocity;
    //TODO use an optimal controller to decide whether to turn and how much instead of time quantities
    if (angularETA > linearETA)   //Shim controller
    {
        //rotate in place
        m_desiredForwardSpeed = 0.0;
        m_desiredLateralVelocity = 0.0;
        m_desiredAngularVelocity = angleDirection * m_maxAngularVelocity;
    }
    else
    {
        m_desiredAngularVelocity = angleDirection * m_maxAngularVelocity;      //w speed
        double Vx_desired = linearSpeed * cosSlope;               //projection of the linear speed on its components
        double Vy_desired = linearSpeed * sinSlope;
        double angle = m_state.angle;
        //Tranform into local frame (inverse of rotation matrix in state dynamics)
        m_desiredForwardSpeed = Vx_desired * cos(angle) + Vy_desired * sin(angle);
        m_desiredLateralVelocity = - Vx_desired * sin(angle) + Vy_desired * cos(angle);
        //Debug print
        //if (m_desiredForwardSpeed > m_maxVelocity)
        //{
        //    std::cout << "FORWARD SPEED SATURATION" << std::endl;
        //}
        //if (m_desiredLateralVelocity > m_maxLateralVelocity)
        //{
        //    std::cout << "LATERAL SPEED SATURATION" << std::endl;
        //}
    }

    //clip velocities for small quantities, to avoid computational error accumulation/oscillations
    //TODO - check if limits should be based on discrete time increment for speed clipping
    //if (relativeAngleDifference< 1E-8)
    //{
    //    m_desiredAngularVelocity = 0.0;
    //}
    if (std::abs(m_desiredForwardSpeed) < m_zeroTolerance)
    {
        m_desiredForwardSpeed = 0.0;
    }
    if (std::abs(m_desiredLateralVelocity) < m_zeroTolerance)
    {
        m_desiredLateralVelocity = 0.0;
    }
    return true;
}

void UnicycleNavigationController::setConstantControl(double forwardSpeed, double angularVelocity, double lateralVelocity)
{
    m_desiredForwardSpeed = forwardSpeed;
    m_desiredAngularVelocity = angularVelocity;
    m_desiredLateralVelocity = lateralVelocity;
}

void UnicycleNavigationController::setInactiveUntil(double endTime)
{
    m_deactivationEndTime = std::abs(endTime);
}

double UnicycleNavigationController::getDesiredLateralVelocity() const
{
    return m_desiredLateralVelocity;
}

bool UnicycleNavigationController::setNavigationPath(std::vector<UnicycleState>& path)
{
    if (path.size() < 2)
    {
        std::cerr << "[UnicycleNavigationController::setNavigationPath] The path should have at least two poses in it" << std::endl;
        return false;
    }
    
    m_navigationPath = path;
    m_poseIndex = 1;            //reset pose index
    m_newPathReceived = true;
    return true;
}

bool UnicycleNavigationController::setMaxVelocities(double & maxVelocity, double & maxLateralVelocity, double & maxAngularVelocity)
{
    if (maxVelocity < 0 || maxLateralVelocity < 0 || maxAngularVelocity < 0)
    {
        std::cout << "[WARNING] Setting negative Max Velocities, their module will be used instead." << std::endl;
    }
    if (std::abs(maxVelocity) < m_zeroTolerance || std::abs(maxLateralVelocity) < m_zeroTolerance || std::abs(maxAngularVelocity) < m_zeroTolerance)
    {
        std::cerr << "[UnicycleNavigationController::setMaxVelocities] Setting Max Velocities too small" << std::endl;
        return false;
    }
    
    m_maxVelocity = std::abs(maxVelocity);
    m_maxLateralVelocity = std::abs(maxLateralVelocity);
    m_maxAngularVelocity = std::abs(maxAngularVelocity);
    return true;
}

bool UnicycleNavigationController::setTimeStep(double &dT)
{
    if (dT < 0)
    {
        std::cout << "[WARNING] Setting negative Time Step, the module will be used instead." << std::endl;
    }
    if (dT == 0)
    {
        std::cerr << "[UnicycleNavigationController::setTimeStep] Setting Time Step to 0 is forbidden. It will create an endless loop" << std::endl;
        return false;
    }
    
    m_dt = std::abs(dT);
    return true;
}