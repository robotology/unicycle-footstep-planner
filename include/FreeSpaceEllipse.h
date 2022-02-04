/*
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef FREESPACEELLIPSE_H
#define FREESPACEELLIPSE_H

#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorFixSize.h>

class FreeSpaceEllipse
{

    // The ellipse is encoded as the image of a unit circle y = C * x + d, where ||x|| = 1.
    iDynTree::MatrixFixSize<2,2> m_C;
    iDynTree::MatrixFixSize<2,2> m_C_inverse;
    iDynTree::Vector2 m_d;
    double m_semiMajorAxis;
    double m_semiMinorAxis;
    double m_angle;
    bool m_isSet;

    double generatorsModule(const iDynTree::Vector2& generators) const;

    iDynTree::Vector2 computeGenerators(const iDynTree::Vector2& inputPoint) const;

public:

    FreeSpaceEllipse(const iDynTree::MatrixFixSize<2,2>& imageMatrix, const iDynTree::Vector2& centerOffset);

    // Define the ellipse given the principal axis half-length, the rotation wrt the inertial frame, and the center position
    FreeSpaceEllipse(double a, double b, double theta, double centerOffsetX, double centerOffsetY);

    FreeSpaceEllipse();

    ~FreeSpaceEllipse() = default;

    FreeSpaceEllipse(const FreeSpaceEllipse& other) = default;

    FreeSpaceEllipse(FreeSpaceEllipse&& other) = default;

    FreeSpaceEllipse& operator=(const FreeSpaceEllipse& other) = default;

    FreeSpaceEllipse& operator=(FreeSpaceEllipse&& other) = default;

    bool setEllipse(const iDynTree::MatrixFixSize<2,2>& imageMatrix, const iDynTree::Vector2& centerOffset);

    bool setEllipse(double a, double b, double theta, double centerOffsetX, double centerOffsetY);

    bool isSet();

    const iDynTree::Vector2& centerOffset() const;

    const iDynTree::MatrixFixSize<2,2>& imageMatrix() const;

    double semiMajorAxis() const;

    double semiMinorAxis() const;

    double angle() const;

    //Returns true if the point is inside the ellipse, boundary included.
    // If the ellipsoid has never been set, it returns true by default
    bool isPointInside(const iDynTree::Vector2& testPoint) const;

    iDynTree::Vector2 projectPointInsideEllipse(const iDynTree::Vector2& testPoint) const;

    bool getIntersectionsWithLine(const iDynTree::Vector2& linePoint1, const iDynTree::Vector2& linePoint2,
                                  iDynTree::Vector2& intersection1, iDynTree::Vector2& intersection2) const;

    iDynTree::Vector2 getTangentVector(const iDynTree::Vector2& intersectionPoint) const;

    std::string printInfo() const;

    void clear();

};



#endif // FREESPACEELLIPSE_H