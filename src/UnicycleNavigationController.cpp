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

    //std::cout << "Setting Velocities: time: "<< m_time <<" m_desiredForwardSpeedX: " << m_desiredForwardSpeed << " m_desiredLateralVelocity: " << m_desiredLateralVelocity << " m_desiredAngularVelocity: " << m_desiredAngularVelocity  << std::endl;

    forwardSpeed = m_desiredForwardSpeed;
    angularVelocity = m_desiredAngularVelocity;
    lateralVelocity = m_desiredLateralVelocity;

    return true;
}

bool UnicycleNavigationController::setUnicycleStateFeedback(const double t, const iDynTree::Vector2 & position, double angle)
{
    m_time = t;
    m_state.position = position;
    m_state.angle = angle;
    return true;
}

//TODO pass in the output variables by reference
bool UnicycleNavigationController::computeDesiredVelocities()
{
    //Next Pose Check: has to be done only when we don't have a new path received, otherwise we will skip the first pose
    if (!m_newPathReceived)
    {
        if (m_ETA < m_dt)
        {
            if (m_poseIndex == m_navigationPath.size() - 1)
            {
                m_desiredForwardSpeed = 0.0;
                m_desiredAngularVelocity = 0.0;
                m_desiredLateralVelocity = 0.0;
                std::cout << "Reached Final Pose: " << m_poseIndex << std::endl;
                return true;
            }

            ++m_poseIndex;
            std::cout << "Switching to pose: " << m_poseIndex << std::endl;
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
    double slope = (m_navigationPath[m_poseIndex].position(1) - m_state.position(1)) / 
                   (m_navigationPath[m_poseIndex].position(0) - m_state.position(0));
    m_slopeAngle = atan(slope);
    m_cosSlope = std::abs(cos(m_slopeAngle));    
    m_sinSlope = std::abs(sin(m_slopeAngle)); 
    const double computationalThreshold = 1E-6;
    //Project each speed component on the segment connecting the two poses
    //Let's find the linear speed on the connection of the two path poses
    double x_projectionSpeed, y_projectionSpeed;
    if (m_cosSlope < computationalThreshold)
    {   //this means that we are moving purely sideways
        m_linearSpeed = m_maxLateralVelocity;    
    }
    else if (m_sinSlope < computationalThreshold)
    {   //this means that we are moving purely in front
        m_linearSpeed = m_maxVelocity;
    }
    else
    {   //diagonal motion
        x_projectionSpeed = m_maxVelocity/m_cosSlope;
        y_projectionSpeed = m_maxLateralVelocity/m_sinSlope;
        //The smallest saturation gives the limit to the module of the linear speed
        if (x_projectionSpeed < y_projectionSpeed)
        {
            m_linearSpeed = x_projectionSpeed;
        }
        else
        {
            m_linearSpeed = y_projectionSpeed;
        }
    }

    double angleDifference = m_navigationPath[m_poseIndex].angle - m_state.angle;
    //ETAs computation
    m_linearETA = distance / m_linearSpeed;  //Time required for moving from (x_i, y_i) to (x_i+1, y_i+1)
    double absoluteAngleDifference = std::abs(angleDifference); 
    if (absoluteAngleDifference >= M_PI)
    {
        absoluteAngleDifference = std::abs(absoluteAngleDifference - 2*M_PI);
    }
    double angularETA = absoluteAngleDifference / m_maxAngularVelocity;   //Time required for moving from theta_i to theta_i+1

    if (angularETA > m_linearETA)
    {
        m_ETA = angularETA;
    }
    else
    {
        m_ETA = m_linearETA;
    }
    
    iDynTree::Vector3 localVersors;
    //calculate the direction, on the plane, of the two consecutive poses on its components (x,y,theta)
    //(x, y, theta) direction sign components / versors
    //should we use the current state? TODO
    localVersors(0) = (m_navigationPath[m_poseIndex].position(0) - m_state.position(0) >= 0) ? 1 : -1;
    localVersors(1) = (m_navigationPath[m_poseIndex].position(1) - m_state.position(1) >= 0) ? 1 : -1;
    
    //Since the angles vary between (-pi, pi]
    if (angleDifference > M_PI)
    {
        localVersors(2) = -1;
    }
    else if (angleDifference >= 0 && angleDifference <= M_PI)
    {
        localVersors(2) = 1;
    }
    else if (angleDifference <0 && angleDifference >= -M_PI)
    {
        localVersors(2) = -1;
    }
    else    // angleDifference < -M_PI
    {
        localVersors(2) = 1;
    }

    //Speed computation 
    angularETA = absoluteAngleDifference / m_maxAngularVelocity;
    
    if (angularETA > m_linearETA)   //Shim controller
    {
        //rotate in place
        m_desiredForwardSpeed = 0.0;
        m_desiredLateralVelocity = 0.0;
        m_desiredAngularVelocity = localVersors(2) * m_maxAngularVelocity;
    }
    else
    {
        m_desiredAngularVelocity = localVersors(2) * m_maxAngularVelocity;      //w speed
        double Vx_desired = localVersors(0) * m_linearSpeed * m_cosSlope;               //projection of the linear speed on its components
        double Vy_desired = localVersors(1) * m_linearSpeed * m_sinSlope;
        double angle = m_state.angle;
        //Tranform into local frame (inverse of rotation matrix in state dynamics)
        m_desiredForwardSpeed = Vx_desired * cos(angle) + Vy_desired * sin(angle);
        m_desiredLateralVelocity = - Vx_desired * sin(angle) + Vy_desired * cos(angle);
        //Debug print
        if (m_desiredForwardSpeed > m_maxVelocity)
        {
            std::cout << "FORWARD SPEED SATURATION" << std::endl;
        }
        if (m_desiredLateralVelocity > m_maxLateralVelocity)
        {
            std::cout << "LATERAL SPEED SATURATION" << std::endl;
        }
    }

    //clip velocities for small quantities, to avoid computational error accumulation/oscillations
    //TODO - limits should be based on discrete time increment X speed
    //if (relativeAngleDifference< 1E-8)
    //{
    //    m_desiredAngularVelocity = 0.0;
    //}
    if (std::abs(m_desiredForwardSpeed) < 1E-6)
    {
        m_desiredForwardSpeed = 0.0;
    }
    if (std::abs(m_desiredLateralVelocity) < 1E-6)
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
    m_deactivationEndTime = endTime;
}

double UnicycleNavigationController::getDesiredLateralVelocity() const
{
    return m_desiredLateralVelocity;
}

bool UnicycleNavigationController::setNavigationPath(std::vector<UnicycleState>& path)
{
    m_navigationPath = path;    // TODO - add consistency check
    m_poseIndex = 1;            //reset pose index
    m_newPathReceived = true;   
    return true;
}

bool UnicycleNavigationController::setMaxVelocities(double & maxVelocity, double & maxLateralVelocity, double & maxAngularVelocity)
{
    m_maxVelocity = maxVelocity;
    m_maxLateralVelocity = maxLateralVelocity;
    m_maxAngularVelocity = maxAngularVelocity;
    return true;
}

bool UnicycleNavigationController::setTimeStep(double &dT)
{
    //TODO - add check
    m_dt = dT;
    return true;
}