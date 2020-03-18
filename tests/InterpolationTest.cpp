/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicycleGenerator.h"
#include "iDynTree/Core/TestUtils.h"
#include <memory>
#include "iDynTree/Core/VectorDynSize.h"
#include "iDynTree/Core/VectorFixSize.h"
#include "iDynTree/Core/Twist.h"
#include "iDynTree/Core/SpatialAcc.h"
#include <fstream>
#include <ctime>


typedef struct {
    double initTime = 0.0, endTime = 10.0, dT = 0.01, K = 10, dX = 0.05, dY = 0.0;
    double maxL = 0.05, minL = 0.005, minW = 0.03, maxAngle = iDynTree::deg2rad(45), minAngle = iDynTree::deg2rad(5);
    double nominalW = 0.05, maxT = 8, minT = 2.9, nominalT = 4, timeWeight = 2.5, positionWeight = 1;

    double nominalSwitchTime = 1.0, endSwitchTime = 0.5, maxSwitch = 3.0;
    double stepHeight = 0.005;
    double comHeight = 0.137;
    double comHeightDelta = 0.002;
    double switchOverSwing = 0.6;
    bool swingLeft = true;
} Configuration;

bool configurePlanner(std::shared_ptr<UnicyclePlanner> planner,const Configuration &conf){
    bool ok = true;
    ok = ok && planner->setDesiredPersonDistance(conf.dX, conf.dY);
    ok = ok && planner->setControllerGain(conf.K);
    ok = ok && planner->setMaximumIntegratorStepSize(conf.dT);
    ok = ok && planner->setMaxStepLength(conf.maxL);
    ok = ok && planner->setWidthSetting(conf.minW, conf.nominalW);
    ok = ok && planner->setMaxAngleVariation(conf.maxAngle);
    ok = ok && planner->setCostWeights(conf.positionWeight, conf.timeWeight);
    ok = ok && planner->setStepTimings(conf.minT, conf.maxT, conf.nominalT);
    ok = ok && planner->setPlannerPeriod(conf.dT);
    ok = ok && planner->setMinimumAngleForNewSteps(conf.minAngle);
    ok = ok && planner->setMinimumStepLength(conf.minL);
    planner->addTerminalStep(true);
    planner->startWithLeft(conf.swingLeft);
    return ok;
}

bool configureGenerator(UnicycleGenerator& generator, const Configuration &conf){
    std::shared_ptr<FeetCubicSplineGenerator> feetGenerator = generator.addFeetCubicSplineGenerator();
    std::shared_ptr<CoMHeightTrajectoryGenerator> heightGenerator = generator.addCoMHeightTrajectoryGenerator();
    std::shared_ptr<ZMPTrajectoryGenerator> zmpGenerator = generator.addZMPTrajectoryGenerator();

    iDynTree::assertTrue(generator.setSwitchOverSwingRatio(conf.switchOverSwing));
    iDynTree::assertTrue(generator.setTerminalHalfSwitchTime(conf.endSwitchTime));
    iDynTree::assertTrue(feetGenerator->setStepHeight(conf.stepHeight));
    iDynTree::assertTrue(generator.setPauseConditions(conf.maxT, conf.nominalT));
    iDynTree::assertTrue(heightGenerator->setCoMHeightSettings(conf.comHeight, conf.comHeightDelta));
    iDynTree::Vector2 leftZMPstance, rightZMPstance;
    leftZMPstance.zero();
    leftZMPstance(0) = 0.01; //1cm forward
    rightZMPstance = leftZMPstance;

    iDynTree::assertTrue(zmpGenerator->setStanceZMPDelta(leftZMPstance, rightZMPstance));

    iDynTree::Vector2 leftZMPswing, rightZMPswing;
    leftZMPswing(0) = 0.01;
    leftZMPswing(1) = -0.005; //5mm toward right
    rightZMPswing(0) = 0.01;
    rightZMPswing(1) = 0.005; //5mm toward left

    iDynTree::assertTrue(zmpGenerator->setInitialSwitchZMPDelta(leftZMPswing, rightZMPswing));

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

void printTrajectories(UnicycleGenerator& interpolator, size_t& newMergePoint, InitialState& newAlpha, size_t mergePoint){
    std::shared_ptr<FeetCubicSplineGenerator> feetGenerator = interpolator.addFeetCubicSplineGenerator();
    std::shared_ptr<CoMHeightTrajectoryGenerator> heightGenerator = interpolator.addCoMHeightTrajectoryGenerator();
    std::shared_ptr<ZMPTrajectoryGenerator> zmpGenerator = interpolator.addZMPTrajectoryGenerator();

    std::ofstream posLeft, posRight, twistLeft, twistRight, accLeft, accRight, zmp, leftZmp, rightZmp, weightLeft, weightRight, height, heightAcceleration, zmpVel, zmpAcc;
    posLeft.open("pL.txt");
    posRight.open("pR.txt");
    twistLeft.open("twistL.txt");
    twistRight.open("twistR.txt");
    accLeft.open("accL.txt");
    accRight.open("accR.txt");
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
    static std::vector<iDynTree::Twist> lFootTwist, rFootTwist;
    static std::vector<iDynTree::SpatialAcc> lFootAcceleration, rFootAcceleration;

    std::vector<iDynTree::Transform> lFootTrajectoryInput, rFootTrajectoryInput;
    std::vector<iDynTree::Twist> lFootTwistInput, rFootTwistInput;
    std::vector<iDynTree::SpatialAcc> lFootAccelerationInput, rFootAccelerationInput;
    feetGenerator->getFeetTrajectories(lFootTrajectoryInput, rFootTrajectoryInput);
    feetGenerator->getFeetTwistsInMixedRepresentation(lFootTwistInput, rFootTwistInput);
    feetGenerator->getFeetAccelerationInMixedRepresentation(lFootAccelerationInput, rFootAccelerationInput);

    std::cerr << rFootTrajectoryInput.size() << " " << rFootTwistInput.size() << " " << rFootAccelerationInput.size() << "\n";

    lFootTrajectory.insert(lFootTrajectory.begin() + mergePoint, lFootTrajectoryInput.begin(), lFootTrajectoryInput.end());
    lFootTrajectory.resize(mergePoint + lFootTrajectoryInput.size());
    rFootTrajectory.insert(rFootTrajectory.begin() + mergePoint, rFootTrajectoryInput.begin(), rFootTrajectoryInput.end());
    rFootTrajectory.resize(mergePoint + rFootTrajectoryInput.size());

    lFootTwist.insert(lFootTwist.begin() + mergePoint, lFootTwistInput.begin(), lFootTwistInput.end());
    lFootTwist.resize(mergePoint + lFootTwistInput.size());
    rFootTwist.insert(rFootTwist.begin() + mergePoint, rFootTwistInput.begin(), rFootTwistInput.end());
    rFootTwist.resize(mergePoint + rFootTwistInput.size());

    lFootAcceleration.insert(lFootAcceleration.begin() + mergePoint, lFootAccelerationInput.begin(), lFootAccelerationInput.end());
    lFootAcceleration.resize(mergePoint + lFootAccelerationInput.size());
    rFootAcceleration.insert(rFootAcceleration.begin() + mergePoint, rFootAccelerationInput.begin(), rFootAccelerationInput.end());
    rFootAcceleration.resize(mergePoint + rFootAccelerationInput.size());

    std::cerr << "--------------------------------------------->Left Trajectory." << std::endl;
    //print_iDynTree(lFootTrajectory);
    for (auto pose : lFootTrajectory){
        posLeft << pose.getPosition()(0) << "    " << pose.getPosition()(1) << "    " <<
            "    " << pose.getPosition()(2)<<"    "<< std::endl;
    }

    for (auto twist : lFootTwist){
        twistLeft << twist(0) << "    " << twist(1) << "    " << twist(2) << "    "
                  << twist(3) << "    " << twist(4) << "    " << twist(5) <<  std::endl;
    }

    for (auto acc : lFootAcceleration){
        accLeft << acc(0) << "    " << acc(1) << "    " << acc(2) << "    "
                << acc(3) << "    " << acc(4) << "    " << acc(5) <<  std::endl;
    }

    std::cerr << "--------------------------------------------->Right Trajectory." << std::endl;
    //print_iDynTree(rFootTrajectory);
    for (auto pose : rFootTrajectory){
        posRight << pose.getPosition()(0) << " " << pose.getPosition()(1) << " " <<
                    " " << pose.getPosition()(2)<<"    "<< std::endl;
    }

    for (auto twist : rFootTwist){
        twistRight << twist(0) << "    " << twist(1) << "    " << twist(2) << "    "
                   << twist(3) << "    " << twist(4) << "    " << twist(5) <<  std::endl;
    }

    for (auto acc : rFootAcceleration){
        accRight << acc(0) << "    " << acc(1) << "    " << acc(2) << "    "
                 << acc(3) << "    " << acc(4) << "    " << acc(5) <<  std::endl;
    }

    static std::vector<double> weightInLeft, weightInRight;
    std::vector<double> weightInLeftInput, weightInRightInput;
    zmpGenerator->getWeightPercentage(weightInLeftInput, weightInRightInput);
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
    zmpGenerator->getLocalZMPTrajectories(lZMPTrajectoryIn, rZMPTrajectoryIn);
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
    zmpGenerator->getZMPTrajectory(ZMPTrajectoryIn, ZMPVelIn, ZMPAccIn);
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
    heightGenerator->getCoMHeightTrajectory(CoMHeightTrajectoryIn);
    CoMHeightTrajectory.insert(CoMHeightTrajectory.begin()+ mergePoint, CoMHeightTrajectoryIn.begin(), CoMHeightTrajectoryIn.end());
    CoMHeightTrajectory.resize(mergePoint + CoMHeightTrajectoryIn.size());

    std::cerr << "--------------------------------------------->CoM Height Trajectory." << std::endl;
    printVector(CoMHeightTrajectory, height);


    static std::vector< double > CoMHeightAccelerationProfile;
    std::vector< double > CoMHeightAccelerationProfileIn;
    heightGenerator->getCoMHeightAccelerationProfile(CoMHeightAccelerationProfileIn);
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
    zmpGenerator->getInitialStatesAtMergePoints(initPoints);

    std::vector<size_t> mergePoints;
    interpolator.getMergePoints(mergePoints);
    newMergePoint = mergePoints[1] + mergePoint;

    newAlpha = initPoints[1];

    std::cerr << "--------------------------------------------->Merge Points." << std::endl;
    printVector(mergePoints);

    posLeft.close();
    posRight.close();
    twistLeft.close();
    twistRight.close();
    accLeft.close();
    accRight.close();
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
    UnicycleGenerator unicycle;
    //FeetInterpolator unicycle;

    iDynTree::assertTrue(configurePlanner(unicycle.unicyclePlanner(), conf));
    iDynTree::assertTrue(configureGenerator(unicycle, conf));

    std::shared_ptr<FootPrint> leftFoot, rightFoot;

    leftFoot = unicycle.getLeftFootPrint();
    rightFoot = unicycle.getRightFootPrint();


    iDynTree::Vector2 initPosition, initVelocity, finalPosition, finalVelocity;

    initPosition(0) = conf.dX;
    initPosition(1) = conf.dY;

    initVelocity.zero();

    finalPosition(0) = initPosition(0) + 0.5;
    finalPosition(1) = initPosition(1) + 0.3;

    finalVelocity.zero();

    iDynTree::assertTrue(unicycle.unicyclePlanner()->addDesiredTrajectoryPoint(conf.initTime, initPosition, initVelocity));
    iDynTree::assertTrue(unicycle.unicyclePlanner()->addDesiredTrajectoryPoint(conf.endTime, finalPosition, finalVelocity));

    clock_t total = clock();
    //iDynTree::assertTrue(unicycle.computeNewSteps(leftFoot, rightFoot));
    //clock_t interpolatorTime = clock();
    iDynTree::assertTrue(unicycle.generate(conf.initTime, conf.dT, conf.endTime));
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
    iDynTree::assertTrue(unicycle.unicyclePlanner()->getPersonPosition(newInitTime, finalPosition));
    unicycle.unicyclePlanner()->clearDesiredTrajectory();
    iDynTree::assertTrue(unicycle.unicyclePlanner()->addDesiredTrajectoryPoint(conf.endTime + 10, finalPosition));

    iDynTree::assertTrue(leftFoot->keepOnlyPresentStep(newInitTime));

    iDynTree::assertTrue(rightFoot->keepOnlyPresentStep(newInitTime));

    printSteps(leftFoot->getSteps(), rightFoot->getSteps());


    iDynTree::assertTrue(unicycle.addZMPTrajectoryGenerator()->setWeightInitialState(newAlpha));
    iDynTree::assertTrue(unicycle.reGenerate(newInitTime, conf.dT, conf.endTime + 50));

    printTrajectories(unicycle, newMergePoint, newAlpha, newMergePoint);

    printSteps(leftFoot->getSteps(), rightFoot->getSteps());



    return true;
}


int main(){
    iDynTree::assertTrue(interpolationTest());
    return EXIT_SUCCESS;
}
