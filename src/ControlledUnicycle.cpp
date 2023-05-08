/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "ControlledUnicycle.h"
#include <cmath>
#include <memory>
#include <iostream>

ControlledUnicycle::ControlledUnicycle()
:iDynTree::optimalcontrol::DynamicalSystem(3, 3)
, m_controller_ptr(nullptr)
, m_navigationMode(false)
{
    m_initialState.resize(3);
    m_initialState.zero();
    m_controllerOutput.resize(3);
    m_controllerOutput.zero();
}

//the state is [x, theta], i.e. the 2D position of the point to be followed, the 2D position of the cart and the angle wrt Z axis;
//the controller is [u;w;v]

bool ControlledUnicycle::dynamics(const iDynTree::VectorDynSize &state, double time, iDynTree::VectorDynSize &stateDynamics)
{
    if(!m_controller_ptr){
        std::cerr << "Unicycle controller not set." << std::endl;
        return false;
    }

    if(state.size() != this->stateSpaceSize()){
        std::cerr << "Wrong state dimensions." <<std::endl;
        return false;
    }

    if(stateDynamics.size() != this->stateSpaceSize())
        stateDynamics.resize(this->stateSpaceSize());

    if(!m_controller_ptr->setStateFeedback(time, state)){
        std::cerr << "Error while setting feedback to the controller." << std::endl;
        return false;
    }

    if(!m_controller_ptr->doControl(m_controllerOutput)){
        std::cerr << "Error while getting the control output." << std::endl;
        return false;
    }

    double theta = state(2);
    double c_theta = std::cos(theta);
    double s_theta = std::sin(theta);
    if (m_navigationMode)
    {
        stateDynamics(0) = m_controllerOutput(0);
        stateDynamics(1) = m_controllerOutput(2);
        stateDynamics(2) = m_controllerOutput(1);
    }
    else
    {
        stateDynamics(0) = c_theta * m_controllerOutput(0) - s_theta * m_controllerOutput(2);
        stateDynamics(1) = s_theta * m_controllerOutput(0) + c_theta * m_controllerOutput(2);
        stateDynamics(2) = m_controllerOutput(1);
    }
    
    
    

    return true;
}

bool ControlledUnicycle::setInitialState(const iDynTree::Vector2 &unicyclePosition, double angle)
{

    m_initialState(0) = unicyclePosition(0);
    m_initialState(1) = unicyclePosition(1);
    m_initialState(2) = angle;

    return true;
}

const iDynTree::VectorDynSize &ControlledUnicycle::initialState() const
{
    return m_initialState;
}

bool ControlledUnicycle::setController(std::shared_ptr<iDynTree::optimalcontrol::Controller> controller)
{
    if (controller->controlSpaceSize() != controlSpaceSize()){
        std::cerr << "The controller dimension is not coherent with the controlSpaceSize." << std::endl;
        return false;
    }
    m_controller_ptr = controller;
    return true;
}

bool ControlledUnicycle::setNavigationMode(bool mode)
{
    m_navigationMode = mode;
    return true;
}
