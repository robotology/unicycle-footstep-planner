/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the BSD-3-Clause license, see LICENSE
 *
 */

#ifndef CONTROLLEDUNICYCLE_H
#define CONTROLLEDUNICYCLE_H

#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Controller.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <memory>

class ControlledUnicycle : public iDynTree::optimalcontrol::DynamicalSystem{
    iDynTree::VectorDynSize m_controllerOutput, m_initialState;
    std::shared_ptr<iDynTree::optimalcontrol::Controller> m_controller_ptr;

public:

    ControlledUnicycle();

    //the state is [x, theta], i.e. the 2D position of the cart and the angle wrt Z axis;
    //the controller is [u;w;v], i.e. forward speed, angular velocity, lateral speed.

    bool dynamics(const iDynTree::VectorDynSize &state, double time, iDynTree::VectorDynSize &stateDynamics) override;

    bool setInitialState(const iDynTree::Vector2& unicyclePosition, double angle);

    const iDynTree::VectorDynSize& initialState() const override;

    bool setController(std::shared_ptr<iDynTree::optimalcontrol::Controller> controller);
};


#endif // CONTROLLEDUNICYCLE_H
