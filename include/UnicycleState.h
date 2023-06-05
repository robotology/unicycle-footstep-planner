/*
 * Copyright (C) 2023 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLE_STATE_H
#define UNICYCLE_STATE_H
#include "iDynTree/Core/VectorFixSize.h"

struct UnicycleState
{
    iDynTree::Vector2 position;
    double angle;
};

#endif // UNICYCLE_STATE_H
