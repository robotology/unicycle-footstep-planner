/*
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia
 * Authors: Simone Micheletti
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef POSES_PAIR_INTERPOLATOR__H
#define POSES_PAIR_INTERPOLATOR__H

//Fix for compiling math constants
#define _USE_MATH_DEFINES
#include <math.h>
#include "UnicycleFoot.h"

//Computes the omnidirectional motion between two given poses
class PosesPairInterpolator
{
private:
    //Constraints on the motion given externally
    double m_maxVelocity;
    double m_maxLateralVelocity;
    double m_maxAngVelocity;
    //Poses between which the interpolation takes place
    UnicycleState m_startPose;
    UnicycleState m_nextPose;

    // Motion parameters
    iDynTree::Vector3 m_localVersors;   //(x, y, theta) direction sign components / versors
    double m_cos_theta;
    double m_sin_theta;
    double m_distance;
    double m_linearSpeed;
    bool m_motionParamComputed{false};

    //Time quantities
    bool m_ETAsComputed{false};
    double m_linearETA;
    double m_angularETA;
    double m_dT;        //time increment of the interpolation
    double m_time;      //quantity that gets incremented by m_dT until maxTime or the next pose is reached
    double m_startTime; //time istant from which we start interpolating
    double m_endTime;   //time horizon after which we end the computation
public:
    PosesPairInterpolator(UnicycleState &startPose, UnicycleState &nextPose, 
                          const double &maxVelocity, const double &maxLateralVelocity, const double &maxAngVelocity, double &timeIncrement,
                          double &startTime);
    ~PosesPairInterpolator();
    struct PoseStamped
    {
        UnicycleState pose;
        double time;
    };
    bool computeMotionParameters();
    bool ETA_Computation();
    std::vector<PoseStamped> shimController();
    std::vector<PoseStamped> interpolate();
};


#endif // POSES_PAIR_INTERPOLATOR__H