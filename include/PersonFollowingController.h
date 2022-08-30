/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef PERSONFOLLOWINGCONTROLLER_H
#define PERSONFOLLOWINGCONTROLLER_H

#include "FreeSpaceEllipse.h"
#include "UnicycleBaseController.h"
#include <iDynTree/TimeRange.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <deque>

typedef struct{
    double initTime;
    iDynTree::Vector2 yDesired;
    iDynTree::Vector2 yDotDesired;
} TrajectoryPoint;

class PersonFollowingController : public UnicycleBaseController{
    iDynTree::Vector2 m_personDistance, m_y, m_personPosition, m_unicyclePosition;
    double m_personDistanceNorm;
    double m_theta;
    iDynTree::MatrixDynSize m_inverseB, m_R;
    std::deque<TrajectoryPoint> m_desiredTrajectory;
    double m_gain, m_time;
    double m_semiMajorInnerEllipseOffset;
    double m_semiMinorInnerEllipseOffset;
    FreeSpaceEllipse m_outerEllipse, m_innerEllipse;
    double m_conservativeFactor;

    void interpolateReferences(double time,
                               const std::deque<TrajectoryPoint>::reverse_iterator& point,
                               iDynTree::Vector2& yOutput, iDynTree::Vector2 &yDotOutput);

public:
    PersonFollowingController();

    //the state is [y, x, theta], i.e. the 2D position of the point to be followed, the 2D position of the cart and the angle wrt Z axis;
    //the controller is [u;w;v], i.e. forward speed, angular velocity, lateral speed.

    virtual bool doUnicycleControl(double& forwardSpeed, double& angularVelocity, double& lateralVelocity) override;

    virtual bool setUnicycleStateFeedback(const double t, const iDynTree::Vector2& unicyclePosition, double unicycleOrientation) override;

    bool setPersonDistance(double xPosition, double yPosition);

    const iDynTree::Vector2& getPersonDistance() const;

    const iDynTree::Vector2& getPersonPosition(const iDynTree::Vector2& unicyclePosition, double unicycleAngle);

    bool setGain(double controllerGain);

    bool setSaturations(double maxVelocity, double maxAngularVelocity);

    bool setDesiredPoint(const TrajectoryPoint &desiredPoint);

    bool getDesiredPoint(double time, iDynTree::Vector2& yDesired, iDynTree::Vector2& yDotDesired);

    bool getDesiredPointInFreeSpaceEllipse(double time, const iDynTree::Vector2& unicyclePosition, double unicycleAngle, iDynTree::Vector2& yDesired, iDynTree::Vector2& yDotDesired);

    bool getDesiredTrajectoryInitialTime(double& firstTime);

    void clearDesiredTrajectory();

    bool clearDesiredTrajectoryUpTo(double time);

    bool setFreeSpaceEllipse(const FreeSpaceEllipse& freeSpaceEllipse);

    bool setFreeSpaceEllipseConservativeFactor(double conservativeFactor);

    bool setInnerFreeSpaceEllipseOffset(double offset);

    bool setInnerFreeSpaceEllipseOffsets(double semiMajorAxisOffset, double semiMinorAxisOffset);
};

#endif // PERSONFOLLOWINGCONTROLLER_H
