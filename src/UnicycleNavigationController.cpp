/*
 * Copyright (C) 20023 Fondazione Istituto Italiano di Tecnologia
 * Authors: Simone Micheletti
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <UnicycleNavigationController.h>
#include <cmath>

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
        computeMotionParameters();
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
    m_state.angle = angle;
    return true;
}

bool UnicycleNavigationController::computeMotionParameters()
{
    double distance = std::sqrt(pow(m_navigationPath[m_poseIndex].position(0) - m_navigationPath[m_poseIndex - 1].position(0), 2) +
                           pow(m_navigationPath[m_poseIndex].position(1) - m_navigationPath[m_poseIndex - 1].position(1), 2) 
                           );   // faster than hypot but without overflow check
    double slope = (m_navigationPath[m_poseIndex].position(1) - m_navigationPath[m_poseIndex - 1].position(1)) / 
                   (m_navigationPath[m_poseIndex].position(0) - m_navigationPath[m_poseIndex - 1].position(0));
    double slopeAngle = atan(slope);
    m_cosSlope = std::abs(cos(slopeAngle));    
    m_sinSlope = std::abs(sin(slopeAngle)); 

    //if both poses aligned to the path = 0, if both poses aligned but not with the path > 0, if both poses perpendicular to the path = 1
    double proprotionFactor = (std::abs(slopeAngle - m_navigationPath[m_poseIndex].angle) + 
                               std::abs(slopeAngle - m_navigationPath[m_poseIndex - 1].angle)) 
                               / 2 / M_PI_2 ;
    //linear speed module that moves from pose i to pose i+1
    m_linearSpeed = std::abs(1 - proprotionFactor) * m_maxVelocity + proprotionFactor * m_maxLateralVelocity;

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

    return true;
}

bool UnicycleNavigationController::computeDesiredVelocities()
{
    iDynTree::Vector3 localVersors;
    //calculate the direction, on the plane, of the two consecutive poses on its components (x,y,theta)
    //(x, y, theta) direction sign components / versors
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
        m_desiredForwardSpeed = localVersors(0) * m_linearSpeed * m_cosSlope;
        m_desiredLateralVelocity = localVersors(1) * m_linearSpeed * m_sinSlope;
        m_desiredAngularVelocity = localVersors(2) * m_maxAngularVelocity;
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
    return true;
}

bool UnicycleNavigationController::setMaxVelocities(double & maxVelocity, double & maxLateralVelocity, double & maxAngularVelocity)
{
    m_maxVelocity = maxVelocity;
    m_maxLateralVelocity = maxLateralVelocity;
    m_maxAngularVelocity = maxAngularVelocity;
    return true;
}