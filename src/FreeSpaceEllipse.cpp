/*
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <FreeSpaceEllipse.h>
#include <cassert>
#include <iostream>
#include <sstream>
#include <cmath>

FreeSpaceEllipse::FreeSpaceEllipse(const iDynTree::MatrixFixSize<2, 2> &imageMatrix, const iDynTree::VectorFixSize<2> &centerOffset)
{
    bool ok = setEllipse(imageMatrix, centerOffset);
    assert(ok);

    if (!ok)
    {
        clear();
    }
}

FreeSpaceEllipse::FreeSpaceEllipse(double a, double b, double theta, double centerOffsetX, double centerOffsetY)
{
    bool ok = setEllipse(a, b, theta, centerOffsetX, centerOffsetY);
    assert(ok);

    if (!ok)
    {
        clear();
    }
}

bool FreeSpaceEllipse::isPointInside(const iDynTree::VectorFixSize<2> &testPoint) const
{
    if (!m_isSet)
    {
        return true; //Default case
    }

    return generatorsModule(computeGenerators(testPoint)) <= 1.0;
}

bool FreeSpaceEllipse::setEllipse(double a, double b, double theta, double centerOffsetX, double centerOffsetY)
{
    if (a < 1e-10 || b < 1e-10)
    {
        std::cerr << "[FreeSpaceEllipse::setEllipse] The input a and b are supposed to be non-zero." << std::endl;
        return false;
    }

    Eigen::Matrix2d rotationMatrix;
    rotationMatrix(0,0) = std::cos(theta);
    rotationMatrix(0,1) = -std::sin(theta);
    rotationMatrix(1,0) = std::sin(theta);
    rotationMatrix(1,1) =std::cos(theta);

    Eigen::Matrix2d defaultC;
    defaultC.setZero();
    defaultC(0,0) = a;
    defaultC(1,1) = b;

    iDynTree::toEigen(m_C) = rotationMatrix * defaultC;
    iDynTree::toEigen(m_C_inverse) = iDynTree::toEigen(m_C).inverse();
    m_d(0) = centerOffsetX;
    m_d(1) = centerOffsetY;

    m_semiMajorAxis = a;
    m_semiMinorAxis = b;
    m_angle = theta;

    m_isSet = true;

    return true;
}

bool FreeSpaceEllipse::isSet()
{
    return m_isSet;
}

const iDynTree::VectorFixSize<2> &FreeSpaceEllipse::centerOffset() const
{
    return m_d;
}

const iDynTree::MatrixFixSize<2, 2> &FreeSpaceEllipse::imageMatrix() const
{
    return m_C;
}

double FreeSpaceEllipse::semiMajorAxis() const
{
    return m_semiMajorAxis;
}

double FreeSpaceEllipse::semiMinorAxis() const
{
    return m_semiMinorAxis;
}

double FreeSpaceEllipse::angle() const
{
    return m_angle;
}

bool FreeSpaceEllipse::setEllipse(const iDynTree::MatrixFixSize<2, 2> &imageMatrix, const iDynTree::VectorFixSize<2> &centerOffset)
{
    double determinant = iDynTree::toEigen(imageMatrix).determinant();

    if (determinant < 1e-10)
    {
        std::cerr << "[FreeSpaceEllipse::setEllipse] The input image matrix is almost singular." << std::endl;
        return false;
    }

    //The image matrix should be in the form
    //  _                           _
    // |  c_theta * a  -s_theta * b  |
    // |_ s_theta * a   c_theta * b _|

    double theta = std::atan2(imageMatrix(1,0), imageMatrix(0,0));
    double stheta = std::sin(theta);
    double ctheta = std::cos(theta);

    double a,b;
    if (std::abs(stheta) > std::abs(ctheta))
    {
        a = imageMatrix(1, 0) / stheta;
        b = imageMatrix(0, 1) / stheta;
    }
    else
    {
        a = imageMatrix(0, 0) / ctheta;
        b = imageMatrix(1, 1) / ctheta;
    }

    if (std::abs(determinant - a * b) > 1e-10)
    {
        std::cerr << "[FreeSpaceEllipse::setEllipse] The specified imagematrix does not seem to correspond to an ellipse." << std::endl;
        return false;
    }

    m_angle = theta;
    m_semiMajorAxis = a;
    m_semiMinorAxis = b;

    m_C = imageMatrix;
    iDynTree::toEigen(m_C_inverse) = iDynTree::toEigen(m_C).inverse();
    m_d = centerOffset;

    m_isSet = true;

    return true;
}

iDynTree::Vector2 FreeSpaceEllipse::computeGenerators(const iDynTree::VectorFixSize<2> &inputPoint) const
{
    iDynTree::Vector2 output;
    iDynTree::toEigen(output) = iDynTree::toEigen(m_C_inverse) * (iDynTree::toEigen(inputPoint) - iDynTree::toEigen(m_d));
    return output;
}

double FreeSpaceEllipse::generatorsModule(const iDynTree::VectorFixSize<2> &generators) const
{
    return (iDynTree::toEigen(generators)).norm();
}

FreeSpaceEllipse::FreeSpaceEllipse()
{
    clear();
}

iDynTree::Vector2 FreeSpaceEllipse::projectPointInsideEllipse(const iDynTree::VectorFixSize<2> &testPoint) const
{
    if (!m_isSet)
    {
        return testPoint; //Default case
    }

    iDynTree::Vector2 generators = computeGenerators(testPoint);
    double module = generatorsModule(generators);
    if (module <= 1.0)
    {
        return testPoint;
    }

    iDynTree::VectorFixSize<2> normalizedGenerators;

    iDynTree::toEigen(normalizedGenerators) = iDynTree::toEigen(generators) / module;

    iDynTree::VectorFixSize<2> output;

    iDynTree::toEigen(output) = iDynTree::toEigen(m_C) * iDynTree::toEigen(normalizedGenerators) + iDynTree::toEigen(m_d);

    return output;
}

bool FreeSpaceEllipse::getIntersectionsWithLine(const iDynTree::VectorFixSize<2> &linePoint1, const iDynTree::VectorFixSize<2> &linePoint2,
                                                iDynTree::VectorFixSize<2> &intersection1, iDynTree::VectorFixSize<2> &intersection2) const
{
    if (!m_isSet)
    {
        return false; //No intersections
    }

    iDynTree::VectorFixSize<2> linePoint1InCircle, linePoint2InCircle;
    linePoint1InCircle.zero();
    linePoint2InCircle.zero();

    // Move the points in the "ellipse space"
    iDynTree::toEigen(linePoint1InCircle) = iDynTree::toEigen(m_C_inverse) *  (iDynTree::toEigen(linePoint1) - iDynTree::toEigen(m_d));
    iDynTree::toEigen(linePoint2InCircle) = iDynTree::toEigen(m_C_inverse) *  (iDynTree::toEigen(linePoint2) - iDynTree::toEigen(m_d));

    double alpha = linePoint1InCircle(1) - linePoint2InCircle(1);
    double beta = linePoint2InCircle(0) - linePoint1InCircle(0);
    double gamma = linePoint1InCircle(0)*linePoint2InCircle(1) - linePoint2InCircle(0) * linePoint1InCircle(1); //alpha * x + beta * y + gamma = 0 is the line equation passing via linePoint1 and linePoint2

    // Compute the intersection between the line and the unit circle centered in zero
    if (std::abs(alpha) < 1e-15 && std::abs(beta) < 1e-15)
    {
        std::cerr << "[ERROR][FreeSpaceEllipse::getIntersectionsWithLine] The line points coincide." << std::endl;;
        return false;
    }

    iDynTree::VectorFixSize<2> intersection1InCircle, intersection2InCircle;
    intersection1InCircle.zero();
    intersection2InCircle.zero();

    if (std::abs(alpha) < 1e-15) // The line is horizontal, y = -gamma / beta. There is an intersection is y <= 1
    {

        intersection1InCircle(1) = -gamma / beta;
        intersection2InCircle(1) = intersection1InCircle(1);

        if (std::abs(intersection1InCircle(1)) <= 1.0)
        {
            intersection1InCircle(0) = sqrt(1 - intersection1InCircle(1) * intersection1InCircle(1));
            intersection2InCircle(0) = -intersection1InCircle(0);
        }
        else
        {
            return false; //No intersections
        }
    }

    else if (std::abs(beta) < 1e-15) // The line is vertical, x = -gamma / alpha. There is an intersection is x <= 1
    {

        intersection1InCircle(0) = -gamma / alpha;
        intersection2InCircle(0) = intersection1InCircle(0);

        if (std::abs(intersection1InCircle(0)) <= 1.0)
        {
            intersection1InCircle(1) = sqrt(1 - intersection1InCircle(0) * intersection1InCircle(0));
            intersection2InCircle(1) = -intersection1InCircle(1);
        }
        else
        {
            return false; //No intersections
        }
    }
    else
    {
        //We rewrite the line as y = sigma * x + delta
        double sigma = -alpha / beta;
        double delta = -gamma / beta;
        //We inject the line equation into x^2 + y^2 = 1 obtaining
        //(1 + sigma^2) * x^2 + 2 * delta * sigma * x + delta^2 - 1 = aPoly * x^2 + bPoly * x + cPoly
        double aPoly = 1.0 + sigma * sigma;
        double bPoly = 2.0 * delta * sigma;
        double cPoly = delta * delta - 1.0;

        double determinant = bPoly * bPoly - 4.0 * aPoly * cPoly;

        if (determinant < 0)
        {
            return false; //No intersections
        }
        else
        {
            intersection1InCircle(0) = (-bPoly + std::sqrt(determinant)) / (2.0 * aPoly);
            intersection2InCircle(0) = (-bPoly - std::sqrt(determinant)) / (2.0 * aPoly);
            intersection1InCircle(1) = sigma * intersection1InCircle(0) + delta;
            intersection2InCircle(1) = sigma * intersection2InCircle(0) + delta;
        }
    }

    //Port the points back to the initial space
    iDynTree::toEigen(intersection1) = iDynTree::toEigen(m_C) * iDynTree::toEigen(intersection1InCircle) + iDynTree::toEigen(m_d);
    iDynTree::toEigen(intersection2) = iDynTree::toEigen(m_C) * iDynTree::toEigen(intersection2InCircle) + iDynTree::toEigen(m_d);
    return true;
}

std::string FreeSpaceEllipse::printInfo() const
{
    std::stringstream stream;
    stream << "Image matrix:" << std::endl << imageMatrix().toString() << "Center offset: " << centerOffset().toString() << std::endl;

    return stream.str();
}

void FreeSpaceEllipse::clear()
{
    iDynTree::toEigen(m_C).setIdentity();
    m_C_inverse = m_C;
    m_d.zero();
    m_angle = 0;
    m_semiMajorAxis = 1.0;
    m_semiMinorAxis = 1.0;
    m_isSet = false;
}
