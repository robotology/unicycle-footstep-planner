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

FreeSpaceEllipse::FreeSpaceEllipse(const iDynTree::MatrixFixSize<2, 2> &imageMatrix, const iDynTree::VectorFixSize<2> &centerOffset)
{
    bool ok = setEllipse(imageMatrix, centerOffset);
    assert(ok);
}

FreeSpaceEllipse::FreeSpaceEllipse(double a, double b, double theta, const iDynTree::VectorFixSize<2> &centerOffset)
{
    bool ok = setEllipse(a, b, theta, centerOffset);
    assert(ok);
}

bool FreeSpaceEllipse::isPointInside(const iDynTree::VectorFixSize<2> &testPoint) const
{
    if (!m_isSet)
    {
        return true; //Default case
    }

    return generatorsModule(computeGenerators(testPoint)) <= 1.0;
}

bool FreeSpaceEllipse::setEllipse(double a, double b, double theta, const iDynTree::VectorFixSize<2> &centerOffset)
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
    m_d = centerOffset;
    m_isSet = true;

    return true;
}

bool FreeSpaceEllipse::setEllipse(const iDynTree::MatrixFixSize<2, 2> &imageMatrix, const iDynTree::VectorFixSize<2> &centerOffset)
{
    if (iDynTree::toEigen(imageMatrix).determinant() < 1e-10)
    {
        std::cerr << "[FreeSpaceEllipse::setEllipse] The input image matrix is almost singular." << std::endl;
        return false;
    }

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
    return (iDynTree::toEigen(generators)).squaredNorm();
}

FreeSpaceEllipse::FreeSpaceEllipse()
{
    iDynTree::toEigen(m_C).setIdentity();
    m_C_inverse = m_C;
    m_d.zero();
    m_isSet = false;
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
