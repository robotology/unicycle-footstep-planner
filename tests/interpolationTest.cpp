/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicycleTrajectoryGenerator.h"
#include "iDynTree/Core/TestUtils.h"
#include <memory>
#include "iDynTree/Core/VectorDynSize.h"
#include "iDynTree/Core/VectorFixSize.h"
#include <fstream>
#include <ctime>


typedef struct {
    double initTime = 0.0, endTime = 10.0, dT = 0.016, K = 10, dX = 0.05, dY = 0.0;
    double maxL = 0.05, minL = 0.005, minW = 0.03, maxAngle = iDynTree::deg2rad(40), minAngle = iDynTree::deg2rad(5);
    double nominalW = 0.04, maxT = 8, minT = 2.9, nominalT = 4, timeWeight = 2.5, positionWeight = 1;

    double nominalSwitchTime = 1.0, endSwitchTime = 0.5, maxSwitch = 3.0;
    double stepHeight = 0.005;
    double comHeight = 0.137;
    double comHeightDelta = 0.002;
    double switchOverSwing = 0.6;
    bool swingLeft = true;
} Configuration;

bool configurePlanner(UnicyclePlanner& planner,const Configuration &conf){
    bool ok = true;
    ok = ok && planner.setDesiredPersonDistance(conf.dX, conf.dY);
    ok = ok && planner.setControllerGain(conf.K);
    ok = ok && planner.setEndTime(conf.endTime);
    ok = ok && planner.setMaximumIntegratorStepSize(conf.dT);
    ok = ok && planner.setMaxStepLength(conf.maxL);
    ok = ok && planner.setWidthSetting(conf.minW, conf.nominalW);
    ok = ok && planner.setMaxAngleVariation(conf.maxAngle);
    ok = ok && planner.setCostWeights(conf.positionWeight, conf.timeWeight);
    ok = ok && planner.setStepTimings(conf.minT, conf.maxT, conf.nominalT);
    ok = ok && planner.setPlannerPeriod(conf.dT);
    ok = ok && planner.setMinimumAngleForNewSteps(conf.minAngle);
    ok = ok && planner.setMinimumStepLength(conf.minL);
    planner.addTerminalStep(true);
    planner.startWithLeft(conf.swingLeft);
    return ok;
}

bool configureInterpolator(FeetInterpolator& interpolator, const Configuration &conf){
    iDynTree::assertTrue(interpolator.setSwitchOverSwingRatio(conf.switchOverSwing));
    iDynTree::assertTrue(interpolator.setTerminalHalfSwitchTime(conf.endSwitchTime));
    iDynTree::assertTrue(interpolator.setStepHeight(conf.stepHeight));
    iDynTree::assertTrue(interpolator.setPauseConditions(conf.maxT, conf.nominalT));
    iDynTree::assertTrue(interpolator.setCoMHeightSettings(conf.comHeight, conf.comHeightDelta));
    iDynTree::Vector2 leftZMPstance, rightZMPstance;
    leftZMPstance.zero();
    leftZMPstance(0) = 0.01; //1cm forward
    rightZMPstance = leftZMPstance;

    iDynTree::assertTrue(interpolator.setStanceZMPDelta(leftZMPstance, rightZMPstance));

    iDynTree::Vector2 leftZMPswing, rightZMPswing;
    leftZMPswing(0) = 0.01;
    leftZMPswing(1) = -0.005; //5mm toward right
    rightZMPswing(0) = 0.01;
    rightZMPswing(1) = 0.005; //5mm toward left

    iDynTree::assertTrue(interpolator.setInitialSwitchZMPDelta(leftZMPswing, rightZMPswing));

    return true;
}

void printSteps(const std::deque<Step>& leftSteps, const std::deque<Step>& rightSteps){
    std::cerr << "Left foot "<< leftSteps.size() << " steps:"<< std::endl;
    for (auto step : leftSteps){
        std::cerr << "Position "<< step.position.toString() << std::endl;
        std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
        std::cerr << "Time  "<< step.impactTime << std::endl;
    }


    std::cerr << std::endl << "Right foot "<< rightSteps.size() << " steps:" << std::endl;
    for (auto step : rightSteps){
        std::cerr << "Position "<< step.position.toString() << std::endl;
        std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
        std::cerr << "Time  "<< step.impactTime << std::endl;
    }
}
template<class object>
void print_iDynTree(object& objectName){
    for (auto pose : objectName){
        std::cerr << pose.toString() << std::endl;
    }
}

template<class object>
void print_iDynTree(object& objectName, std::ofstream& file){
    for (auto pose : objectName){
        file << pose.toString() << std::endl;
    }
}

template<class object>
void printVector(object& objectName){
    for (auto pose : objectName){
        std::cerr << pose << std::endl;
    }
}

template<class object>
void printVector(object& objectName, std::ofstream& file){
    for (auto pose : objectName){
        file << pose << std::endl;
    }
}

void printTrajectories(const FeetInterpolator& interpolator, size_t& newMergePoint, InitialState& newAlpha, size_t mergePoint){
    std::ofstream posLeft, posRight, zmp, leftZmp, rightZmp, weightLeft, weightRight, height, heightAcceleration, zmpVel, zmpAcc;
    posLeft.open("pL.txt");
    posRight.open("pR.txt");
    zmp.open ("zmp.txt");
    zmpVel.open ("zmpVel.txt");
    zmpAcc.open("zmpAcc.txt");
    leftZmp.open("zmpL.txt");
    rightZmp.open("zmpR.txt");
    weightLeft.open ("leftW.txt");
    weightRight.open ("rightW.txt");
    height.open("height.txt");
    heightAcceleration.open("heightAcc.txt");

    static std::vector<iDynTree::Transform> lFootTrajectory, rFootTrajectory;

    std::vector<iDynTree::Transform> lFootTrajectoryInput, rFootTrajectoryInput;
    interpolator.getFeetTrajectories(lFootTrajectoryInput, rFootTrajectoryInput);

    lFootTrajectory.insert(lFootTrajectory.begin() + mergePoint, lFootTrajectoryInput.begin(), lFootTrajectoryInput.end());
    lFootTrajectory.resize(mergePoint + lFootTrajectoryInput.size());
    rFootTrajectory.insert(rFootTrajectory.begin() + mergePoint, rFootTrajectoryInput.begin(), rFootTrajectoryInput.end());
    rFootTrajectory.resize(mergePoint + rFootTrajectoryInput.size());

    std::cerr << "--------------------------------------------->Left Trajectory." << std::endl;
    //print_iDynTree(lFootTrajectory);
    for (auto pose : lFootTrajectory){
        posLeft << pose.getPosition()(0) << "    " << pose.getPosition()(1) << "    " <<
                "    " << pose.getPosition()(2)<<"    "<< std::endl;
    }

    std::cerr << "--------------------------------------------->Right Trajectory." << std::endl;
    //print_iDynTree(rFootTrajectory);
    for (auto pose : rFootTrajectory){
        posRight << pose.getPosition()(0) << " " << pose.getPosition()(1) << " " <<
                    " " << pose.getPosition()(2)<<"    "<< std::endl;
    }

    static std::vector<double> weightInLeft, weightInRight;
    std::vector<double> weightInLeftInput, weightInRightInput;
    interpolator.getWeightPercentage(weightInLeftInput, weightInRightInput);
    weightInLeft.insert(weightInLeft.begin() + mergePoint, weightInLeftInput.begin(), weightInLeftInput.end());
    weightInLeft.resize(mergePoint + weightInLeftInput.size());
    weightInRight.insert(weightInRight.begin() + mergePoint, weightInRightInput.begin(), weightInRightInput.end());
    weightInRight.resize(mergePoint + weightInRightInput.size());

    std::cerr << "--------------------------------------------->Left Weight." << std::endl;
    printVector(weightInLeft, weightLeft);

    std::cerr << "--------------------------------------------->Right Weight." << std::endl;
    printVector(weightInRight, weightRight);

    static std::vector< iDynTree::Vector2 > lZMPTrajectory, rZMPTrajectory;
    std::vector< iDynTree::Vector2 > lZMPTrajectoryIn, rZMPTrajectoryIn;
    interpolator.getLocalZMPTrajectories(lZMPTrajectoryIn, rZMPTrajectoryIn);
    lZMPTrajectory.insert(lZMPTrajectory.begin()+ mergePoint, lZMPTrajectoryIn.begin(), lZMPTrajectoryIn.end());
    lZMPTrajectory.resize(mergePoint + lZMPTrajectoryIn.size());
    rZMPTrajectory.insert(rZMPTrajectory.begin()+ mergePoint, rZMPTrajectoryIn.begin(), rZMPTrajectoryIn.end());
    rZMPTrajectory.resize(mergePoint + rZMPTrajectoryIn.size());

    std::cerr << "--------------------------------------------->Left ZMP." << std::endl;
    print_iDynTree(lZMPTrajectory, leftZmp);

    std::cerr << "--------------------------------------------->Right ZMP." << std::endl;
    print_iDynTree(rZMPTrajectory, rightZmp);

    static std::vector< iDynTree::Vector2 > ZMPTrajectory, ZMPVel, ZMPAcc;
    std::vector< iDynTree::Vector2 > ZMPTrajectoryIn, ZMPVelIn, ZMPAccIn;
    interpolator.getZMPTrajectory(ZMPTrajectoryIn, ZMPVelIn, ZMPAccIn);
    ZMPTrajectory.insert(ZMPTrajectory.begin()+ mergePoint, ZMPTrajectoryIn.begin(), ZMPTrajectoryIn.end());
    ZMPTrajectory.resize(mergePoint + ZMPTrajectoryIn.size());
    ZMPVel.insert(ZMPVel.begin()+ mergePoint, ZMPVelIn.begin(), ZMPVelIn.end());
    ZMPVel.resize(mergePoint + ZMPVelIn.size());
    ZMPAcc.insert(ZMPAcc.begin()+ mergePoint, ZMPAccIn.begin(), ZMPAccIn.end());
    ZMPAcc.resize(mergePoint + ZMPAccIn.size());

    std::cerr << "--------------------------------------------->Global ZMP Trajectory." << std::endl;
    print_iDynTree(ZMPTrajectory, zmp);
    print_iDynTree(ZMPVel, zmpVel);
    print_iDynTree(ZMPAcc, zmpAcc);

    static std::vector< double > CoMHeightTrajectory;
    std::vector< double > CoMHeightTrajectoryIn;
    interpolator.getCoMHeightTrajectory(CoMHeightTrajectoryIn);
    CoMHeightTrajectory.insert(CoMHeightTrajectory.begin()+ mergePoint, CoMHeightTrajectoryIn.begin(), CoMHeightTrajectoryIn.end());
    CoMHeightTrajectory.resize(mergePoint + CoMHeightTrajectoryIn.size());

    std::cerr << "--------------------------------------------->CoM Height Trajectory." << std::endl;
    printVector(CoMHeightTrajectory, height);


    static std::vector< double > CoMHeightAccelerationProfile;
    std::vector< double > CoMHeightAccelerationProfileIn;
    interpolator.getCoMHeightAccelerationProfile(CoMHeightAccelerationProfileIn);
    CoMHeightAccelerationProfile.insert(CoMHeightAccelerationProfile.begin()+ mergePoint, CoMHeightAccelerationProfileIn.begin(), CoMHeightAccelerationProfileIn.end());
    CoMHeightAccelerationProfile.resize(mergePoint + CoMHeightAccelerationProfileIn.size());

    std::cerr << "--------------------------------------------->CoM Height Acceleration." << std::endl;
    printVector(CoMHeightAccelerationProfile, heightAcceleration);

    static std::vector < bool > lFootContacts, rFootContacts;
    std::vector < bool > lFootContactsIn, rFootContactsIn;
    interpolator.getFeetStandingPeriods(lFootContactsIn, rFootContactsIn);
    lFootContacts.insert(lFootContacts.begin()+ mergePoint, lFootContactsIn.begin(), lFootContactsIn.end());
    lFootContacts.resize(mergePoint + lFootContactsIn.size());
    rFootContacts.insert(rFootContacts.begin()+ mergePoint, rFootContactsIn.begin(), rFootContactsIn.end());
    rFootContacts.resize(mergePoint + rFootContactsIn.size());


//    std::cerr << "--------------------------------------------->Left Contacts." << std::endl;
//    printVector(lFootContacts);

//    std::cerr << "--------------------------------------------->Right Contacts." << std::endl;
//    printVector(rFootContacts);

    static std::vector < bool > lFootFixed;
    std::vector < bool > lFootFixedIn;
    interpolator.getWhenUseLeftAsFixed(lFootFixedIn);
    lFootFixed.insert(lFootFixed.begin()+ mergePoint, lFootFixedIn.begin(), lFootFixedIn.end());
    lFootFixed.resize(mergePoint + lFootFixedIn.size());
    std::cerr << "--------------------------------------------->Left fixed." << std::endl;
//    printVector(lFootFixed);

    std::vector<InitialState> initPoints;
    interpolator.getInitialStatesAtMergePoints(initPoints);

    std::vector<size_t> mergePoints;
    interpolator.getMergePoints(mergePoints);
    newMergePoint = mergePoints[1] + mergePoint;

    newAlpha = initPoints[1];

    std::cerr << "--------------------------------------------->Merge Points." << std::endl;
    printVector(mergePoints);

    posLeft.close();
    posRight.close();
    zmp.close();
    zmpVel.close();
    zmpAcc.close();
    leftZmp.close();
    rightZmp.close();
    weightLeft.close();
    weightRight.close();
    height.close();
    heightAcceleration.close();
}


bool interpolationTest(){
    Configuration conf;
    UnicycleTrajectoryGenerator unicycle;
    //FeetInterpolator unicycle;

    iDynTree::assertTrue(configurePlanner(unicycle, conf));
    iDynTree::assertTrue(configureInterpolator(unicycle, conf));

    std::shared_ptr<FootPrint> leftFoot, rightFoot;

    leftFoot = std::make_shared<FootPrint>();
    rightFoot = std::make_shared<FootPrint>();

    iDynTree::Vector2 initPosition, initVelocity, finalPosition, finalVelocity;

    initPosition(0) = conf.dX;
    initPosition(1) = conf.dY;

    initVelocity.zero();

    finalPosition(0) = initPosition(0) + 0.5;
    finalPosition(1) = initPosition(1) + 0.3;

    finalVelocity.zero();

    iDynTree::assertTrue(unicycle.addDesiredTrajectoryPoint(conf.initTime, initPosition, initVelocity));
    iDynTree::assertTrue(unicycle.addDesiredTrajectoryPoint(conf.endTime, finalPosition, finalVelocity));

    clock_t total = clock();
    //iDynTree::assertTrue(unicycle.computeNewSteps(leftFoot, rightFoot));
    //clock_t interpolatorTime = clock();
    iDynTree::assertTrue(unicycle.generateAndInterpolate(leftFoot, rightFoot, conf.initTime, conf.dT));
    //iDynTree::assertTrue(unicycle.interpolate(*leftFoot, *rightFoot, conf.initTime, conf.dT, 0.5, false));
    clock_t end = clock();

    //std::cerr <<"Interpolator has Finished in " << (static_cast<double>(end - interpolatorTime) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

    std::cerr <<"Total time " << (static_cast<double>(end - total) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

    std::cerr << "Output from unicycle." << std::endl;
    printSteps(leftFoot->getSteps(), rightFoot->getSteps());

    size_t newMergePoint;
    InitialState newAlpha;
    printTrajectories(unicycle, newMergePoint, newAlpha, 0);
    double newInitTime;
    newInitTime = newMergePoint*conf.dT + conf.initTime;
    std::cerr << "New run at " << newInitTime << std::endl;
    iDynTree::assertTrue(unicycle.setEndTime(conf.endTime + 50));
    iDynTree::assertTrue(unicycle.getPersonPosition(newInitTime, finalPosition));
    unicycle.clearDesiredTrajectory();
    iDynTree::assertTrue(unicycle.addDesiredTrajectoryPoint(conf.endTime + 10, finalPosition));

    iDynTree::assertTrue(leftFoot->keepOnlyPresentStep(newInitTime));

    iDynTree::assertTrue(rightFoot->keepOnlyPresentStep(newInitTime));

    printSteps(leftFoot->getSteps(), rightFoot->getSteps());


    iDynTree::assertTrue(unicycle.generateAndInterpolate(leftFoot, rightFoot, newInitTime, conf.dT, newAlpha));

    printTrajectories(unicycle, newMergePoint, newAlpha, newMergePoint);

    printSteps(leftFoot->getSteps(), rightFoot->getSteps());



    return true;
}


int main(){
    iDynTree::assertTrue(interpolationTest());
    return EXIT_SUCCESS;
}
