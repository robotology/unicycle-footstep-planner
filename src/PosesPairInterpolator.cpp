/*
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia
 * Authors: Simone Micheletti
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "PosesPairInterpolator.h"
#include <iostream>

PosesPairInterpolator::PosesPairInterpolator(UnicycleState startPose, UnicycleState nextPose, 
                          const double maxVelocity, const double maxLateralVelocity, const double maxAngVelocity, double timeIncrement,
                          double startTime)
{   
    m_maxVelocity = maxVelocity;
    m_maxLateralVelocity = maxLateralVelocity;
    m_maxAngVelocity = maxAngVelocity;
    m_startPose = startPose;
    m_nextPose = nextPose;
    if (timeIncrement > 0)
    {
        m_dT = timeIncrement;
    }
    else
    {
        m_dT = 0.1; //default
    }
    m_startTime = startTime;
}

PosesPairInterpolator::~PosesPairInterpolator()
{
}

bool PosesPairInterpolator::computeMotionParameters()
{
    if (m_motionParamComputed)
    {
        //
        return true;
    }
    //Let's assume a linear uniform motion between two consecutive path poses at constant speed.
    m_distance = std::sqrt(pow(m_nextPose.position(0) - m_startPose.position(0), 2) +
                           pow(m_nextPose.position(1) - m_startPose.position(1), 2) 
                           );
    double slope = (m_nextPose.position(1) - m_startPose.position(1)) / 
                   (m_nextPose.position(0) - m_startPose.position(0));
    double slope_angle = atan(slope);
    m_cos_theta = std::abs(cos(slope_angle));    
    m_sin_theta = std::abs(sin(slope_angle)); 

    //if both poses aligned to the path = 0, if both poses aligned but not with the path > 0, if both poses perpendicular to the path = 1
    double proprotionFactor = (std::abs(slope_angle - m_nextPose.angle) + std::abs(slope_angle - m_startPose.angle))/ 2 / M_PI_2 ;
    //linear speed module that moves from pose i to pose i+1
    m_linearSpeed = std::abs(1 - proprotionFactor) * m_maxVelocity + proprotionFactor * m_maxLateralVelocity;

    //calculate the direction, on the plane, of the two consecutive poses on its components (x,y,theta)
    //(x, y, theta) direction sign components / versors
    m_localVersors(0) = (m_nextPose.position(0) - m_startPose.position(0) >= 0) ? 1 : -1;
    m_localVersors(1) = (m_nextPose.position(1) - m_startPose.position(1) >= 0) ? 1 : -1;
    
    //Angles vary between (-pi, pi]
    double angleDifference = m_nextPose.angle - m_startPose.angle;
    if (angleDifference > M_PI)
    {
        m_localVersors(2) = -1;
    }
    else if (angleDifference >=0 && angleDifference <= M_PI)
    {
        m_localVersors(2) = 1;
    }
    else if (angleDifference <0 && angleDifference >= -M_PI)
    {
        m_localVersors(2) = -1;
    }
    else    // angleDifference < -M_PI
    {
        m_localVersors(2) = 1;
    }
    m_motionParamComputed = true;
    return true;
}

bool PosesPairInterpolator::ETA_Computation()
{
    //Initial checks
    if (!m_motionParamComputed)
    {
        return false;
    }
    if (m_ETAsComputed)
    {
        return true;
    }
    //Computation
    m_linearETA = m_distance / m_linearSpeed;  //Time required for moving from (x_i, y_i) to (x_i+1, y_i+1)
    double deltaPosesAngle = std::abs(m_nextPose.angle - m_startPose.angle); //orientation difference between two consecutive path poses
    if (deltaPosesAngle >= M_PI)    //take into account angle periodicity (-pi, pi]
    {
        deltaPosesAngle = std::abs(deltaPosesAngle - 2* M_PI);
    }
    m_angularETA = deltaPosesAngle / m_maxAngVelocity;   //Time required for moving from theta_i to theta_i+1
    m_ETAsComputed = true;
    return true;
}

std::vector<PosesPairInterpolator::PoseStamped> PosesPairInterpolator::shimController(PosesPairInterpolator::PoseStamped startPose)
{
    std::vector<PoseStamped> interpolatedSegment;   //output of the interpolation of the ShimController
    m_time = startPose.time;    //initialize the moment from which start the 
    //SHIM CONTROLLER. rotation BEFORE the linear movement if I have a too big deltaPosesAngle.
    // The robot will rotate in-place until the extra agle will be compensated, then it will roto-translate to the next pose.
    if (m_angularETA > m_linearETA)     
    {
        double shimTime = 0;    //counter of how much time we take to complete the shim rotation
        //compute the interpolation for the rotation of the unicycle on itself untill is aligned and add this time to the elapsedTimeLinear
        //update also t for each dT passed
        int missingIterations = std::ceil((m_angularETA - m_linearETA) / m_dT) ;    //round up by excess to calculate the number of iterations for fulling the time gap
        UnicycleState shimState;
        //Rotation in place -> X and Y don't change
        shimState.position(0) = m_startPose.position(0);
        shimState.position(1) = m_startPose.position(1);
        for (size_t i = 1; i < missingIterations; i++)
        {
            m_time += m_dT;
            shimTime += m_dT;
            if (interpolatedSegment.empty())
            {
                shimState.angle = startPose.pose.angle + m_localVersors(2) * m_maxAngVelocity * m_dT;
            }
            else
            {
                shimState.angle = interpolatedSegment.back().pose.angle + m_localVersors(2) * m_maxAngVelocity * m_dT;
            }

            //deal with angle periodicity
            if (shimState.angle > M_PI) //overshoot in the positive domain
            {
                shimState.angle -= 2*M_PI;
            }
            else if (shimState.angle < -M_PI)   //overshoot in the negative domain
            {
                shimState.angle += 2*M_PI;
            }
            
            //save the pose
            PoseStamped ps {shimState, m_time};
            interpolatedSegment.push_back(ps);            
        }
        //now we have m_angularETA <= m_linearETA
        //so we add the missing time to m_linearETA and subtract it to m_angularETA
        m_linearETA += shimTime;
        m_angularETA -= shimTime;
    }

    return interpolatedSegment;
}

std::vector<PosesPairInterpolator::PoseStamped> PosesPairInterpolator::interpolate(PosesPairInterpolator::PoseStamped startPose)
{
    std::vector<PosesPairInterpolator::PoseStamped> interpolatedSegment;   //output
    double iterationEndTime = m_startTime + m_linearETA;    //add the time elapsed for all the previous poses in the path and the initial time (m_startTime)
    //interpolate between two path poses
    double sweptAngle = 0; //the cumulative angle which is being swept after each iter
    double missingAngle = m_angularETA * m_maxAngVelocity;    //angle missing to achieve the desired orientation of m_nextPose

    while (m_time < iterationEndTime)
    {
        //compute the next interpolated point
        m_time += m_dT;
        UnicycleState nextState;
        //check if we overshoot the final pose -> then we clamp it
        if (m_time >= iterationEndTime)
        {
            nextState.position(0) = m_nextPose.position(0);
            nextState.position(1) = m_nextPose.position(1);
            nextState.angle = m_nextPose.angle;
        }
        else
        {
            double angleIncrement = m_dT * m_maxAngVelocity;
            double next_angle = 0;
            if (interpolatedSegment.empty())
            {
                nextState.position(0) = startPose.pose.position(0) + m_localVersors(0) * m_dT * (m_linearSpeed * m_cos_theta);    //the first part of the equation computes the number of iteration of dT passed untill this time istant.
                                                                                                                                  //the second part is the x component of the maximum speed
                nextState.position(1) = startPose.pose.position(1) + m_localVersors(1) * m_dT * (m_linearSpeed * m_sin_theta);
                next_angle = startPose.pose.angle + m_localVersors(2) * angleIncrement;
            }
            else
            {
                //starting from the previous pose in the path, I add the increment on each motion component
                nextState.position(0) = interpolatedSegment.back().pose.position(0) + m_localVersors(0) * m_dT * (m_linearSpeed * m_cos_theta);    //the first part of the equation computes the number of iteration of dT passed untill this time istant.
                                                                                                                                    //the second part is the x component of the maximum speed
                nextState.position(1) = interpolatedSegment.back().pose.position(1) + m_localVersors(1) * m_dT * (m_linearSpeed * m_sin_theta);
                next_angle = interpolatedSegment.back().pose.angle + m_localVersors(2) * angleIncrement;
            }
            sweptAngle += angleIncrement;

            //angle overshoot check
            if (sweptAngle >= missingAngle)
            {
                nextState.angle = m_nextPose.angle;
            }
            else
            {
                nextState.angle = next_angle;
            }
        }

        //save the pose
        PoseStamped ps {nextState, m_time};
        interpolatedSegment.push_back(ps);
    }
    return interpolatedSegment;
}
