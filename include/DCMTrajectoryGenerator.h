/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi, Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef DCMTRAJECTORYGENERATOR_H
#define DCMTRAJECTORYGENERATOR_H

#include <StepPhase.h>
#include <FootPrint.h>
#include <iDynTree/Core/VectorFixSize.h>

#include <vector>
#include <memory>

typedef struct{
    iDynTree::Vector2 initialPosition;
    iDynTree::Vector2 initialVelocity;
} DCMInitialState;

class DCMTrajectoryGenerator {

    friend class UnicycleGenerator;

    class DCMTrajectoryGeneratorImplementation;
    std::unique_ptr<DCMTrajectoryGeneratorImplementation> m_pimpl;

    DCMTrajectoryGenerator();

    /**
     * Generete a DCM trajectory.
     * @param left is a vector containing all the left footprints;
     * @param right is a vector containing all the left footprints;
     * @param initTime is the trajectory initial time;
     * @param dT sampling time;
     * @return true/false in case of success/failure.
     */
    bool computeNewTrajectories(double initTime, double dT, double switchPercentage, double maxStepTime,
                                double nominalStepTime, bool pauseActive, const std::vector<const Step *> &orderedSteps,
                                const std::vector<size_t> &phaseShift, const std::vector<StepPhase> &lFootPhases,
                                const FootPrint &left, const FootPrint &right);

public:

    ~DCMTrajectoryGenerator();

    bool setDCMInitialState(const DCMInitialState &initialState);

    /**
     * Set the time constant of the 3D-LIPM.
     * @param omega is the time constant of the 3D-LIPM.
     * @return true / false in case of success / failure.
     */
    bool setOmega(const double &omega);

    /**
     * @brief Specifies an offset from the origin of the foot frame to be used when generating trajectories
     * @param offsetInLeftFootFrame Offset with respect the origin of the left foot
     * @param offsetInRightFootFrame Offset with respect the origin of the right foot
     * @return true/false in case of success/failure.
     */
    bool setFootOriginOffset(const iDynTree::Vector2 &offsetInLeftFootFrame, const iDynTree::Vector2 &offsetInRightFootFrame);

    /**
     * Get the position of the DCM along the entire trajectory.
     * @return a vector containing the position of the DCM
     */
    const std::vector<iDynTree::Vector2>& getDCMPosition() const;

    /**
     * Get the velocity of the DCM along the entire trajectory.
     * @return a vector containing the velocity of the DCM
     */
    const std::vector<iDynTree::Vector2>& getDCMVelocity() const;

    /**
     * Output the weight percentage carried by each foot while walking, according to the DCM trajectory.
     */
    void getWeightPercentage(std::vector<double>& weightInLeft, std::vector<double>& weightInRight) const;

};

#endif // DCMTRAJECTORYGENERATOR_H
