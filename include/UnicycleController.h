/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLECONTROLLER_H
#define UNICYCLECONTROLLER_H

#include <iDynTree/Controller.h>
#include <iDynTree/TimeRange.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <memory>
#include <deque>

typedef struct{
    double initTime;
    iDynTree::Vector2 yDesired;
    iDynTree::Vector2 yDotDesired;
} TrajectoryPoint;

class UnicyleController : public iDynTree::optimalcontrol::Controller{
    iDynTree::Vector2 m_personDistance, m_y, m_personPosition;
    double m_theta;
    iDynTree::MatrixDynSize m_inverseB, m_R;
    std::deque<TrajectoryPoint> m_desiredTrajectory;
    double m_gain, m_maxVelocity, m_maxAngularVelocity, m_time;

    double saturate(double input, double saturation);

    void interpolateReferences(double time,
                               const std::deque<TrajectoryPoint>::reverse_iterator& point,
                               iDynTree::Vector2& yOutput, iDynTree::Vector2 &yDotOutput);

public:
    UnicyleController();

    //the state is [y, x, theta], i.e. the 2D position of the point to be followed, the 2D position of the cart and the angle wrt Z axis;
    //the controller is [u;w]

    bool doControl(iDynTree::VectorDynSize &controllerOutput) override;

    bool setStateFeedback(const double t, const iDynTree::VectorDynSize &stateFeedback) override;

    bool setPersonDistance(double xPosition, double yPosition);

    const iDynTree::Vector2& getPersonDistance() const;

    const iDynTree::Vector2& getPersonPosition(const iDynTree::Vector2& unicyclePosition, double unicycleAngle);

    bool setGain(double controllerGain);

    bool setSaturations(double maxVelocity, double maxAngularVelocity);

    bool setDesiredPoint(const TrajectoryPoint &desiredPoint);

    bool getDesiredPoint(double time, iDynTree::Vector2& yDesired, iDynTree::Vector2& yDotDesired);

    bool getDesiredTrajectoryInitialTime(double& firstTime);

    void clearDesiredTrajectory();

    bool clearDesiredTrajectoryUpTo(double time);
};

#endif // UNICYCLECONTROLLER_H
