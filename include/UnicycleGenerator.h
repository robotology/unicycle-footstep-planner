/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef UNICYCLEGENERATOR_H
#define UNICYCLEGENERATOR_H

#include <UnicyclePlanner.h>
#include <FeetCubicSplineGenerator.h>
#include <ZMPTrajectoryGenerator.h>
#include <CoMHeightTrajectoryGenerator.h>
#include <memory>

class UnicycleGenerator {
    class UnicycleGeneratorImplementation;
   std::unique_ptr<UnicycleGeneratorImplementation> m_pimpl;

public:

    UnicycleGenerator();

    ~UnicycleGenerator();

    std::shared_ptr<UnicyclePlanner> unicyclePlanner();

    std::shared_ptr<FootPrint> getLeftFootPrint();

    std::shared_ptr<FootPrint> getRightFootPrint();

    bool generateFromFootPrints(std::shared_ptr<FootPrint> left, std::shared_ptr<FootPrint> right, double initTime, double dT); //here the planner is not called

    bool generate(double initTime, double dT, double endTime); //here the planner is called

    bool reGenerate(double initTime, double dT, double endTime); //here the planner is called

    bool reGenerate(double initTime, double dT, double endTime, const Step &measuredLeft, const Step &measuredRight); //automatically sets previous steps if zmp trajectory generation is used

    bool reGenerate(double initTime, double dT, double endTime, bool correctLeft,
                    const iDynTree::Vector2 &measuredPosition, double measuredAngle); //automatically sets previous steps if zmp trajectory generation is used

    bool reGenerate(double initTime, double dT, double endTime,
                    const iDynTree::Vector2 &measuredLeftPosition, double measuredLeftAngle,
                    const iDynTree::Vector2 &measuredRightPosition, double measuredRightAngle); //automatically sets previous steps if zmp trajectory generation is used

    //Settings
    bool setSwitchOverSwingRatio(double ratio); //indeed the swing time cannot be null, while the switch time can be very close to zero (but not zero)

    bool setTerminalHalfSwitchTime(double lastHalfSwitchTime); //if not set, it won't bring the ZMP at the center of the feet at the end

    bool setPauseConditions(double maxStepTime, double nominalStepTime);

    //Getters
    void getFeetStandingPeriods(std::vector<bool>& lFootContacts, std::vector<bool>& rFootContacts) const;

    void getWhenUseLeftAsFixed(std::vector<bool>& leftIsFixed) const;

    void getMergePoints(std::vector<size_t>& mergePoints) const; //indexes in which is suitable to perform a merge of trajectories. The weight percentage is discontinuos in velocity


    //Plugins

    std::shared_ptr<FeetCubicSplineGenerator> addFeetCubicSplineGenerator();

    std::shared_ptr<ZMPTrajectoryGenerator> addZMPTrajectoryGenerator();

    std::shared_ptr<CoMHeightTrajectoryGenerator> addCoMHeightTrajectoryGenerator();

};

#endif // UNICYCLEGENERATOR_H
