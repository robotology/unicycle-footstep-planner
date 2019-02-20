/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <FeetGenerator.h>
FeetGenerator::~FeetGenerator()
{ }

iDynTree::Matrix3x3 FeetGenerator::RPYRightTrivializedSecondDerivative(const double /*roll*/, const double pitch, const double yaw, const double rollDot, const double pitchDot, const double /*yawDot*/)
    {
        iDynTree::Matrix3x3 map;

        double sp = std::sin(pitch);
        double cp = std::cos(pitch);
        double sy = std::sin(yaw);
        double cy = std::cos(yaw);

        map(0, 0) = 0.0;
        map(1, 0) = 0.0;
        map(2, 0) = 0.0;
        map(0, 1) = -sp * cy * rollDot;
        map(1, 1) = -sy * sp * rollDot;
        map(2, 1) = -cp * rollDot;
        map(0, 2) = -(cp * sy * rollDot + cy * pitchDot);
        map(1, 2) = cy * cp * rollDot - sy * pitchDot;
        map(2, 2) = 0.0;

        return map;
    }
