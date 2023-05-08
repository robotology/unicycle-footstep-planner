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
    if (m_newPathReceived)
    {
        computeMotionParameters();
        m_newPathReceived = false;
    }
    
    if (m_time < m_deactivationEndTime)
    {
        forwardSpeed = 0.0;
        angularVelocity = 0.0;
        lateralVelocity = 0.0;

        return true;
    }

    // check when to switch the the next pose in the path
    // use time and TODO space checks
    if (m_time >= m_ETA)
    {   
        //check if the end of the path has been reached -> stay still
        if (m_poseIndex == m_navigationPath.size() - 1)
        {
            forwardSpeed = 0.0;
            angularVelocity = 0.0;
            lateralVelocity = 0.0;

            return true;
        }
        
        ++m_poseIndex;
        std::cout << "Switching to pose: " << m_poseIndex << std::endl;
        computeMotionParameters();  //each time I switch the next pose I have to recompute the motion
    }
    
    computeDesiredVelocities();

    std::cout << "Setting Velocities: time: "<< m_time <<" m_desiredForwardSpeedX: " << m_desiredForwardSpeed << " m_desiredLateralVelocity: " << m_desiredLateralVelocity << " m_desiredAngularVelocity: " << m_desiredAngularVelocity  << std::endl;


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
    std::cout << "Unicycle postion FEEDBACK: time: "<< t <<" X: " << position(0) << " Y: " << position(1) << " Angle: " << angle  << std::endl;
    return true;
}

bool UnicycleNavigationController::computeMotionParameters()
{
    double distance = std::sqrt(pow(m_navigationPath[m_poseIndex].position(0) - m_navigationPath[m_poseIndex - 1].position(0), 2) +
                           pow(m_navigationPath[m_poseIndex].position(1) - m_navigationPath[m_poseIndex - 1].position(1), 2) 
                           );   // faster than hypot but without overflow check
    double slope = (m_navigationPath[m_poseIndex].position(1) - m_navigationPath[m_poseIndex - 1].position(1)) / 
                   (m_navigationPath[m_poseIndex].position(0) - m_navigationPath[m_poseIndex - 1].position(0));
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

    double angleDifference = m_navigationPath[m_poseIndex].angle - m_navigationPath[m_poseIndex - 1].angle;
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
        m_ETA = angularETA + m_time;
    }
    else
    {
        m_ETA = m_linearETA + m_time;
    }
    std::cout << "Computed ETAs - m_linearETA: " << m_linearETA << " angularETA: " << angularETA << " m_ETA: " << m_ETA << " Linear Speed: " << m_linearSpeed << std::endl;
    return true;
}

bool UnicycleNavigationController::computeDesiredVelocities()
{
    iDynTree::Vector3 localVersors;
    //calculate the direction, on the plane, of the two consecutive poses on its components (x,y,theta)
    //(x, y, theta) direction sign components / versors
    //should we use the current state? TODO
    localVersors(0) = (m_navigationPath[m_poseIndex].position(0) - m_navigationPath[m_poseIndex - 1].position(0) >= 0) ? 1 : -1;
    localVersors(1) = (m_navigationPath[m_poseIndex].position(1) - m_navigationPath[m_poseIndex - 1].position(1) >= 0) ? 1 : -1;

    // angle difference from the 
    double relativeAngleDifference = m_navigationPath[m_poseIndex].angle - m_state.angle;
    //Since the angles vary between (-pi, pi]
    if (relativeAngleDifference > M_PI)
    {
        localVersors(2) = -1;
    }
    else if (relativeAngleDifference >= 0 && relativeAngleDifference <= M_PI)
    {
        localVersors(2) = 1;
    }
    else if (relativeAngleDifference <0 && relativeAngleDifference >= -M_PI)
    {
        localVersors(2) = -1;
    }
    else    // angleDifference < -M_PI
    {
        localVersors(2) = 1;
    }

    //Speed computation
    double absoluteAngleDifference = std::abs(relativeAngleDifference); 
    if (absoluteAngleDifference >= M_PI)
    {
        absoluteAngleDifference = std::abs(absoluteAngleDifference - 2*M_PI);
    }
    double angularETA = absoluteAngleDifference / m_maxAngularVelocity;
    
    if (angularETA > m_linearETA)   //Shim controller
    {
        //rotate in place
        m_desiredForwardSpeed = 0.0;
        m_desiredLateralVelocity = 0.0;
        m_desiredAngularVelocity = localVersors(2) * m_maxAngularVelocity;
    }
    else
    {
        //roto-traslate
        //TODO COMPENSATE FOR ROTATIONAL DRIFT
        if (relativeAngleDifference < 0.01)
        {
            m_desiredAngularVelocity = 0.0;
            m_desiredForwardSpeed = localVersors(0) * m_linearSpeed * m_cosSlope;
            m_desiredLateralVelocity = localVersors(1) * m_linearSpeed * m_sinSlope;
        }
        else
        {
            m_desiredAngularVelocity = localVersors(2) * m_maxAngularVelocity;      //w speed
            double Vx_desired = localVersors(0) * m_linearSpeed * m_cosSlope;               //projection of the linear speed on its components
            double Vy_desired = localVersors(1) * m_linearSpeed * m_sinSlope;
            double dW = m_desiredAngularVelocity * m_dt;    //angle swept during an iteration
            //std::cout << "Vx: " << Vx <<  " Vy: " << Vy << " dW: " << dW << std::endl;
            //Compute directly in local frame
            //m_desiredForwardSpeed = cos(m_slopeAngle - m_state.angle - dW) * (Vx_desired * cos(- dW) - Vy_desired * sin(- dW));
            //m_desiredLateralVelocity = sin(m_slopeAngle - m_state.angle - dW) * (Vx_desired * sin(-dW) + Vy_desired * cos(-dW));
            m_desiredForwardSpeed = Vx_desired;
            m_desiredLateralVelocity = Vy_desired;
            //double tmp_x = cos(m_slopeAngle - dW) * (Vx_desired * cos(- dW) - Vy_desired * sin(- dW));
            //double tmp_y = sin(m_slopeAngle - dW) * (Vx_desired * sin(-dW) + Vy_desired * cos(-dW));
            //Transform it in local frame
            //m_desiredForwardSpeed = tmp_x * cos(-m_state.angle) - tmp_y * sin(-m_state.angle);
            //m_desiredLateralVelocity = tmp_x * sin(-m_state.angle) + tmp_y * cos(-m_state.angle);
        }
        //let's predict the next position, then compensate with the error of the desired position
        //TODO
    }

    //clip velocities for small quantities, to avoid computational error accumulation/oscillations
    //TODO - limits should be based on discrete time increment X speed
    if (relativeAngleDifference<0.01)
    {
        m_desiredAngularVelocity = 0.0;
    }
    if (std::abs(m_desiredForwardSpeed) < 0.0001)
    {
        m_desiredForwardSpeed = 0.0;
    }
    if (std::abs(m_desiredLateralVelocity) < 0.0001)
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
    m_poseIndex = 1;    //reset pose index
    m_newPathReceived = true;
    return true;
}

bool UnicycleNavigationController::setMaxVelocities(double & maxVelocity, double & maxLateralVelocity, double & maxAngularVelocity)
{
    std::cout << "Setting max velocities: maxVelocity: " << maxVelocity << " maxLateralVelocity: " << maxLateralVelocity << " maxAngularVelocity: " << maxAngularVelocity << std::endl;
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