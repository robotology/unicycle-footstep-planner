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
#include <vector>

//Computes the omnidirectional motion between two given poses, at the maximum speed allowed
class PosesPairInterpolator
{
private:
    //Constraints on the motion (computed externally)
    double m_maxVelocity;               //max linear velocity
    double m_maxLateralVelocity;        //max pure lateral speed allowed (y)
    double m_maxAngVelocity;            //max angular velocity for yaw rotation

    //Poses between which the interpolation takes place
    UnicycleState m_startPose;          //start pose from which start interpolating
    UnicycleState m_nextPose;           //end pose of interpolation

    // Motion parameters
    iDynTree::Vector3 m_localVersors;   //(x, y, theta) direction sign components / versors from starting pose to next pose
    double m_cos_theta;                 //cosine of the slope angle between the two poses
    double m_sin_theta;                 //sine of the slope angle between the two poses
    double m_distance;                  //cartesian distance between the two poses
    double m_linearSpeed;               //constant translational speed from starting pose to next pose
    bool m_motionParamComputed{false};  //flag that states if the motion parameters have already been computed

    //Time quantities
    bool m_ETAsComputed{false};     //flag that states if the time quantities have already been computed
    double m_linearETA;             //time needed to reach the final X,Y pose
    double m_angularETA;            //time needed to reach the final angular pose
    double m_dT;                    //time increment of the interpolation
    double m_time;                  //quantity that gets incremented by m_dT until time horizon or the next pose is reached
    double m_startTime;             //time istant from which we start interpolating
    double m_endTime;               //time horizon after which we end the computation

public:

    /**
     * Default constructor of PosesPairInterpolator object
     * @param startPose             starting pose from where starting to interpolate
     * @param nextPose              final pose of the interpolation
     * @param maxVelocity           max linear velocity allowed
     * @param maxLateralVelocity    max pure lateral speed allowed (y)
     * @param maxAngVelocity        max angular velocity for yaw rotation
     * @param timeIncrement         time step of the interpolation
     * @param startTime             time istant at the startPose
     */
    PosesPairInterpolator(UnicycleState startPose, UnicycleState nextPose, 
                          const double maxVelocity, const double maxLateralVelocity, const double maxAngVelocity, double timeIncrement,
                          double startTime);

    /**
     * Default destructor
     */
    ~PosesPairInterpolator();

    //Struct containing the pose of the unicycle with its relative timestamp
    struct PoseStamped
    {
        UnicycleState pose;
        double time;
    };

    /**
     * Computes the Motion parameters of the motion between the two poses
     * @return true in case of correct computation
     */
    bool computeMotionParameters();

    /**
     * Computes the timings of the motion between the two poses
     * @return true in case of correct computation
     */
    bool ETA_Computation();

    /**
     * Rotation in place to compensate for a too big disalignment between the start and next pose
     * @param startPose Pose where the robot starts rotating in place
     * @return a vector of PoseStamped where X and Y state don't change, but only the angle
     */
    std::vector<PoseStamped> shimController(PosesPairInterpolator::PoseStamped startPose);

    /**
     * Interpolation using a roto-traslating motion from the startPose to the next pose
     * @param startPose Pose from where the robot starts roto-traslating
     * @return a vector of PoseStamped up until the next pose
     */
    std::vector<PoseStamped> interpolate(PosesPairInterpolator::PoseStamped startPose);
};


#endif // POSES_PAIR_INTERPOLATOR__H