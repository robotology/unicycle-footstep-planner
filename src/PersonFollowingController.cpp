/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "PersonFollowingController.h"
#include "iDynTree/EigenHelpers.h"
#include "Eigen/Core"
#include <iostream>
#include <cmath>
#include <algorithm>

void PersonFollowingController::interpolateReferences(double time,
                                              const std::deque<TrajectoryPoint>::reverse_iterator &point,
                                              iDynTree::Vector2 &yOutput, iDynTree::Vector2 &yDotOutput)
{
    //actually point-1 is the next point, we are using reverse pointers
    double ratio = ((point-1)->initTime - time)/((point-1)->initTime - point->initTime);

    iDynTree::toEigen(yOutput) = ratio * iDynTree::toEigen(point->yDesired) + (1-ratio) * iDynTree::toEigen((point-1)->yDesired);
    iDynTree::toEigen(yDotOutput) = ratio * iDynTree::toEigen(point->yDotDesired) + (1-ratio) * iDynTree::toEigen((point-1)->yDotDesired);

}

PersonFollowingController::PersonFollowingController()
    :m_theta(0)
    ,m_gain(10)
    ,m_time(0)
    ,m_semiMajorInnerEllipseOffset(0.0)
    ,m_semiMinorInnerEllipseOffset(0.0)
    ,m_conservativeFactor(2.0)
{
    m_personDistance(0) = 0.2;
    m_personDistance(1) = 0.0;
    m_personDistanceNorm = iDynTree::toEigen(m_personDistance).norm();
    m_personPosition.zero();

    m_y.zero();

    m_unicyclePosition.zero();

    m_inverseB.resize(2,2);
    m_inverseB.zero();

    m_R.resize(2,2);
    m_R.zero();

    TrajectoryPoint dummyReference;
    dummyReference.initTime = 0;
    dummyReference.yDesired.zero();
    dummyReference.yDotDesired.zero();
    m_desiredTrajectory.push_back(dummyReference);

}

bool PersonFollowingController::doUnicycleControl(double &forwardSpeed, double &angularVelocity, double &lateralVelocity)
{
    double s_theta = std::sin(m_theta);
    double c_theta =std::cos(m_theta);
    double dx = m_personDistance(0);
    double dy = m_personDistance(1);

    m_inverseB(0,0) = -dy/dx*s_theta + c_theta;
    m_inverseB(0,1) = dy/dx*c_theta + s_theta;
    m_inverseB(1,0) = -s_theta/dx;
    m_inverseB(1,1) = c_theta/dx;

    iDynTree::Vector2 yDesired, yDotDesired;

    if (!getDesiredPointInFreeSpaceEllipse(m_time, m_unicyclePosition, m_theta, yDesired, yDotDesired)){
        std::cerr << "Error while reading the desired point." << std::endl;
        return false;
    }

    Eigen::Vector2d controlOutput = iDynTree::toEigen(m_inverseB) * (iDynTree::toEigen(yDotDesired) -
                                    m_gain * (iDynTree::toEigen(m_y) - iDynTree::toEigen(yDesired)));

    forwardSpeed = controlOutput(0);
    angularVelocity = controlOutput(1);
    lateralVelocity = 0.0;

    return true;
}

bool PersonFollowingController::setUnicycleStateFeedback(const double t, const iDynTree::Vector2 &unicyclePosition, double unicycleOrientation)
{
    m_time = t;

    m_theta = unicycleOrientation;

    m_unicyclePosition = unicyclePosition;

    m_y = getPersonPosition(m_unicyclePosition, m_theta);

    return true;
}

bool PersonFollowingController::setPersonDistance(double xPosition, double yPosition)
{
    if(xPosition == 0){
        std::cerr << "The xPosition need to be greater than 0." << std::endl;
        return false;
    }
    m_personDistance(0) = xPosition;
    m_personDistance(1) = yPosition;

    m_personDistanceNorm = iDynTree::toEigen(m_personDistance).norm();
    return true;
}

const iDynTree::Vector2 &PersonFollowingController::getPersonDistance() const
{
    return m_personDistance;
}

const iDynTree::Vector2 &PersonFollowingController::getPersonPosition(const iDynTree::Vector2 &unicyclePosition, double unicycleAngle)
{
    double c_theta = std::cos(unicycleAngle);
    double s_theta = std::sin(unicycleAngle);

    m_R(0,0) = c_theta;
    m_R(0,1) = -s_theta;
    m_R(1,0) = s_theta;
    m_R(1,1) = c_theta;

    iDynTree::toEigen(m_personPosition) = iDynTree::toEigen(unicyclePosition) + iDynTree::toEigen(m_R)*iDynTree::toEigen(m_personDistance);

    return m_personPosition;
}

bool PersonFollowingController::setGain(double controllerGain)
{
    if (controllerGain <= 0){
        std::cerr << "The controller gain is supposed to be positive." << std::endl;
        return false;
    }
    m_gain = controllerGain;
    return true;
}

bool PersonFollowingController::setDesiredPoint(const TrajectoryPoint &desiredPoint)
{
    if (desiredPoint.initTime < 0){
        std::cerr << "The initTime parameter is supposed to be non-negative." << std::endl;
        return false;
    }

    m_desiredTrajectory.push_back(desiredPoint);

    std::sort(m_desiredTrajectory.begin(), m_desiredTrajectory.end(), [](const TrajectoryPoint &a, const TrajectoryPoint &b) { return a.initTime < b.initTime;}); //reorder the vector

    return true;
}

bool PersonFollowingController::getDesiredTrajectoryInitialTime(double &firstTime)
{
    if (m_desiredTrajectory.empty()){
        std::cerr << "No trajectory loaded yet." << std::endl;
        return false;
    }

    firstTime = m_desiredTrajectory.front().initTime;
    return true;
}

void PersonFollowingController::clearDesiredTrajectory()
{
    m_desiredTrajectory.clear();
}

bool PersonFollowingController::clearDesiredTrajectoryUpTo(double time)
{
    if (time < 0){
        std::cerr << "The time is expected to be non-negative" <<std::endl;
        return false;
    }

    if (m_desiredTrajectory.empty()){
        return true;
    }

    iDynTree::Vector2 yDesired, yDotDesired;
    if(!getDesiredPoint(time, yDesired, yDotDesired))
        return false;

    std::deque<TrajectoryPoint>::iterator element = m_desiredTrajectory.begin();

    while((element != m_desiredTrajectory.end()) && (element->initTime <= time)){
        element++;
        m_desiredTrajectory.pop_front();
    }
    TrajectoryPoint firstPoint;
    firstPoint.yDesired = yDesired;
    firstPoint.yDotDesired = yDotDesired;
    firstPoint.initTime = time;
    m_desiredTrajectory.push_front(firstPoint);

    return true;
}

bool PersonFollowingController::setFreeSpaceEllipse(const FreeSpaceEllipse &freeSpaceEllipse)
{
    m_outerEllipse = freeSpaceEllipse;
    m_innerEllipse = freeSpaceEllipse;
    if (m_outerEllipse.isSet())
    {
        double innerEllipseSemiMajorAxis = m_outerEllipse.semiMajorAxis() - m_semiMajorInnerEllipseOffset;
        double innerEllipseSemiMinorAxis = m_outerEllipse.semiMinorAxis() - m_semiMinorInnerEllipseOffset;

        if ((innerEllipseSemiMajorAxis <= 0.0) || (innerEllipseSemiMinorAxis <= 0.0))
        {
            std::cerr << "[ERROR][UnicyleController::setFreeSpaceEllipse] The specified free space ellipse is too small for the given inner ellipse offset." << std::endl;
            return false;
        }
        if (!m_innerEllipse.setEllipse(innerEllipseSemiMajorAxis, innerEllipseSemiMinorAxis, m_outerEllipse.angle(),
                                       m_outerEllipse.centerOffset()(0), m_outerEllipse.centerOffset()(1)))
        {
            return false;
        }
    }

    return true;
}

bool PersonFollowingController::setFreeSpaceEllipseConservativeFactor(double conservativeFactor)
{
    if (conservativeFactor < 0)
    {
        std::cerr << "The free space ellipse conservative factor is expected to be non-negative" <<std::endl;
        return false;
    }

    m_conservativeFactor = conservativeFactor;
    return true;
}

bool PersonFollowingController::setInnerFreeSpaceEllipseOffset(double offset)
{
    return setInnerFreeSpaceEllipseOffsets(offset, offset);
}

bool PersonFollowingController::setInnerFreeSpaceEllipseOffsets(double semiMajorAxisOffset, double semiMinorAxisOffset)
{
    if (semiMajorAxisOffset < 0)
    {
        std::cerr << "The inner free space ellipse semi major axis offset is expected to be non-negative" << std::endl;
        return false;
    }

    if (semiMinorAxisOffset < 0)
    {
        std::cerr << "The inner free space ellipse semi minor axis offset is expected to be non-negative" << std::endl;
        return false;
    }

    m_semiMajorInnerEllipseOffset = semiMajorAxisOffset;
    m_semiMinorInnerEllipseOffset = semiMinorAxisOffset;
    return setFreeSpaceEllipse(m_outerEllipse);
}

bool PersonFollowingController::getDesiredPoint(double time,
                                        iDynTree::Vector2 &yDesired, iDynTree::Vector2 &yDotDesired)
{
    if (time < 0){
        std::cerr << "The time is expected to be non-negative" <<std::endl;
        return false;
    }

    if (m_desiredTrajectory.empty()){
        std::cerr << "First you have to load a desired trajectory." <<std::endl;
        return false;
    }

    double initTime = time - 1.0;
    getDesiredTrajectoryInitialTime(initTime);

    if (time < initTime){
        std::cerr << "The time is before the beginning of the desired trajectory." << std::endl;
        return false;
    }

    std::deque<TrajectoryPoint>::reverse_iterator pointIterator =
            std::find_if(m_desiredTrajectory.rbegin(),
                         m_desiredTrajectory.rend(),
                         [time](const TrajectoryPoint & a) -> bool { return a.initTime <= time; }); //find the last element in the vector with init time lower than the specified time
    if (pointIterator == m_desiredTrajectory.rend()){ //something went wrong
        std::cerr << "Something went wrong while getting the reference." <<std::endl;
        return false;
    }
    if (pointIterator == m_desiredTrajectory.rbegin()){
        yDesired = pointIterator->yDesired;
        yDotDesired = pointIterator->yDotDesired;
    } else
        interpolateReferences(time, pointIterator, yDesired, yDotDesired);

    return true;

}

bool PersonFollowingController::getDesiredPointInFreeSpaceEllipse(double time, const iDynTree::Vector2 &unicyclePosition, double unicycleAngle, iDynTree::Vector2 &yDesired, iDynTree::Vector2 &yDotDesired)
{
    if (!getDesiredPoint(time, yDesired, yDotDesired))
    {
        return false;
    }

    if (m_innerEllipse.isSet() && m_innerEllipse.isPointInside(yDesired))
    {
        return true; //premature exit if the point is already inside the inner ellipse
    }

    Eigen::Vector2d desiredFromOuter, desiredFromInner;
    desiredFromInner = desiredFromOuter  = iDynTree::toEigen(yDesired);
    double blendingFactor = 0.0;

    if (m_outerEllipse.isSet())
    {
        desiredFromOuter = iDynTree::toEigen(m_outerEllipse.projectPointInsideEllipse(yDesired, unicyclePosition));
        desiredFromInner = desiredFromOuter;
    }

    if (m_innerEllipse.isSet())
    {
        desiredFromInner = iDynTree::toEigen(m_innerEllipse.projectPointInsideEllipse(yDesired, unicyclePosition));

        iDynTree::Vector2 closestIntersection;
        iDynTree::Vector2 personPosition = getPersonPosition(unicyclePosition, unicycleAngle);

        //Compute the intersections between inner ellipse and the line passing between the center of the unicycle and the person
        if (m_outerEllipse.getClosestIntersectionsWithLine(unicyclePosition, personPosition, closestIntersection))
        {
            Eigen::Vector2d ellipseTangentVector = iDynTree::toEigen(m_outerEllipse.getTangentVector(closestIntersection));
            Eigen::Vector2d desiredMotionVector = iDynTree::toEigen(yDesired) - iDynTree::toEigen(personPosition);
            double desiredMotionVectorModule = desiredMotionVector.norm();
            if (desiredMotionVectorModule > 1e-4)
            {
                blendingFactor = std::abs(desiredMotionVector.transpose() * ellipseTangentVector) / desiredMotionVectorModule; //1 if the desired motion vector is parallel to the tangent, 0 if perpendicular
            }
        }
    }

    blendingFactor = std::tanh(m_conservativeFactor * blendingFactor);
    iDynTree::toEigen(yDesired) = blendingFactor * desiredFromInner + (1.0 - blendingFactor) * desiredFromOuter;

    return true;
}
