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
#include <memory>

class UnicycleGenerator {
    class UnicycleGeneratorImplementation;
    UnicycleGeneratorImplementation *m_pimpl;

public:

    UnicycleGenerator();

    ~UnicycleGenerator();

    std::shared_ptr<UnicyclePlanner> unicyclePlanner();

    bool interpolate(const FootPrint &left, const FootPrint &right, double initTime, double dT,
                     const Step &previousLeft, const Step &previousRight); //both feet are supposed to start on the ground at zero velocity. The initTime must be greater than the maximum of the first impactTime of the two feet. The first step has half switch time. The FootPrints needs to be ordered! previousLeft and previouRight are needed to compensate eventual discontinuities on the ZMP when the foot lands not at the specified point

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




};

#endif // UNICYCLEGENERATOR_H
