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
