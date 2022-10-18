/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra, Simone Micheletti
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "UnicyclePlanner.h"
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Utils.h>
#include <Eigen/Core>
#include "iDynTree/Core/EigenHelpers.h"
#include "iDynTree/Core/MatrixDynSize.h"
#include <cmath>
#include <memory>
#include <iostream>
#include <ctime>

// Custom ROS2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RosNode : public rclcpp::Node
{
private:
    /* consts */
    const std::string m_sub_topic_name = "/plan";
    const std::string m_pathPub_topic_name = "/unicycle_path_follower/dcm_path";
    const std::string m_footprints_topic_name = "/unicycle_path_follower/footprints";
    //pubs and subs
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_dcm_pub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_footprint_markers_pub;
    // TFs
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;

    void sub_callback(nav_msgs::msg::Path::ConstPtr &msg_in)
    {

    }

public:
    RosNode() : Node("unicycle_path_follower_node")
    {
        // pubs and subs
        m_dcm_pub = this->create_publisher<nav_msgs::msg::Path>(m_pathPub_topic_name, 10);
        m_footprint_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_footprints_topic_name, 10);
        m_path_sub = this->create_subscription<nav_msgs::msg::Path>(m_sub_topic_name,
                                                                    10,
                                                                    std::bind(&RosNode::sub_callback, this, _1)
                                                                    );
        //TFs
        m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);

    }
    ~RosNode();
};



struct Configuration {
    double initTime = 0.0, endTime = 50.0, dT = 0.01, K = 10, dX = 0.2, dY = 0.0;
    double maxL = 0.2, minL = 0.05, minW = 0.08, maxAngle = iDynTree::deg2rad(45), minAngle = iDynTree::deg2rad(5);
    double nominalW = 0.14, maxT = 10, minT = 3, nominalT = 4, timeWeight = 2.5, positionWeight = 1;
    bool swingLeft = true;
    double slowWhenTurnGain = 0.5;
};

bool printSteps(std::deque<Step> leftSteps, std::deque<Step> rightSteps){
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

    return true;
}

bool checkConstraints(std::deque<Step> leftSteps, std::deque<Step> rightSteps, Configuration conf){
    //Checking constraints

    conf.swingLeft = (leftSteps.front().impactTime >= rightSteps.front().impactTime);

    if (leftSteps.front().impactTime == rightSteps.front().impactTime)
        leftSteps.pop_front(); //this is a fake step!

    bool result = true;
    double distance = 0.0, deltaAngle = 0.0, deltaTime = 0.0, c_theta, s_theta;
    iDynTree::MatrixDynSize rTranspose(2,2);
    iDynTree::Vector2 rPl;

    while (!leftSteps.empty() && !rightSteps.empty()){
        distance = (iDynTree::toEigen(leftSteps.front().position) - iDynTree::toEigen(rightSteps.front().position)).norm();

        if (distance > conf.maxL){
            std::cerr <<"[ERROR] Distance constraint not satisfied" << std::endl;
            result = false;
        }

        deltaAngle = std::abs(leftSteps.front().angle - rightSteps.front().angle);

        if (deltaAngle > conf.maxAngle){
            std::cerr <<"[ERROR] Angle constraint not satisfied" << std::endl;
            result = false;
        }

        deltaTime = std::abs(leftSteps.front().impactTime - rightSteps.front().impactTime);

        if (deltaTime < conf.minT){
            std::cerr <<"[ERROR] Min time constraint not satisfied" << std::endl;
            result = false;
        }

        c_theta = std::cos(rightSteps.front().angle);
        s_theta = std::sin(rightSteps.front().angle);

        rTranspose(0,0) = c_theta;
        rTranspose(1,0) = -s_theta;
        rTranspose(0,1) = s_theta;
        rTranspose(1,1) = c_theta;

        iDynTree::toEigen(rPl) =
                iDynTree::toEigen(rTranspose)*(iDynTree::toEigen(leftSteps.front().position) - iDynTree::toEigen(rightSteps.front().position));

        if (rPl(1) < conf.minW){
            std::cerr <<"[ERROR] Width constraint not satisfied" << std::endl;
            result = false;
        }

        if(conf.swingLeft)
            rightSteps.pop_front();
        else leftSteps.pop_front();

        conf.swingLeft = !conf.swingLeft;

    }

    if(!result)
        return false;

    return true;
}

bool populateDesiredTrajectory(UnicyclePlanner& planner, double initTime, double endTime, double dT){

    double t = initTime;
    iDynTree::Vector2 yDes, yDotDes;
    while (t <= endTime){
        yDes(0) = 0.01*t;
        yDotDes(0) = 0.01;
        yDes(1) = 0.5*std::sin(0.1*t);
        yDotDes(1) = 0.5*0.1*std::cos(0.1*t);
        if(!planner.addPersonFollowingDesiredTrajectoryPoint(t,yDes, yDotDes))
            return false;
        t += dT;
    }
    return true;
}

bool plannerTest(){

    Configuration conf;
    conf.initTime = 0.0;
    conf.endTime = 50.0;
    conf.dT = 0.01;
    conf.K = 10;
    conf.dX = 0.2;
    conf.dY = 0.0;
    conf.maxL = 0.2;
    conf.minL = 0.05;
    conf.minW = 0.08;
    conf.maxAngle = iDynTree::deg2rad(45);
    conf.minAngle = iDynTree::deg2rad(5);
    conf.nominalW = 0.14;
    conf.maxT = 10;
    conf.minT = 3;
    conf.nominalT = 4;
    conf.timeWeight = 2.5;
    conf.positionWeight = 1;
    conf.swingLeft = true;
    conf.slowWhenTurnGain = 0.5;

    UnicyclePlanner planner;

    //Initialization (some of these calls may be avoided)
    iDynTree::assertTrue(planner.setDesiredPersonDistance(conf.dX, conf.dY));
    iDynTree::assertTrue(planner.setPersonFollowingControllerGain(conf.K));
    iDynTree::assertTrue(planner.setMaximumIntegratorStepSize(conf.dT));
    iDynTree::assertTrue(planner.setMaxStepLength(conf.maxL));
    iDynTree::assertTrue(planner.setWidthSetting(conf.minW, conf.nominalW));
    iDynTree::assertTrue(planner.setMaxAngleVariation(conf.maxAngle));
    iDynTree::assertTrue(planner.setCostWeights(conf.positionWeight, conf.timeWeight));
    iDynTree::assertTrue(planner.setStepTimings(conf.minT, conf.maxT, conf.nominalT));
    iDynTree::assertTrue(planner.setPlannerPeriod(conf.dT));
    iDynTree::assertTrue(planner.setMinimumAngleForNewSteps(conf.minAngle));
    iDynTree::assertTrue(planner.setMinimumStepLength(conf.minL));
    iDynTree::assertTrue(planner.setSlowWhenTurnGain(conf.slowWhenTurnGain));

    planner.addTerminalStep(true);
    planner.startWithLeft(conf.swingLeft);

    //Generate desired trajectory
    clock_t start = clock();
    iDynTree::assertTrue(populateDesiredTrajectory(planner, conf.initTime, conf.endTime, conf.dT));
    std::cerr <<"Populating the trajectory took " << (static_cast<double>(clock() - start) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

    std::shared_ptr<FootPrint> left, right;
    left = std::make_shared<FootPrint>();
    right = std::make_shared<FootPrint>();
    iDynTree::Vector2 initPosition;
    initPosition(0) = 0.3;
    initPosition(1) = -0.5;
    //left->addStep(initPosition, iDynTree::deg2rad(15), 25); //fake initialization

    start = clock();
    iDynTree::assertTrue(planner.computeNewSteps(left, right, conf.initTime, conf.endTime));
    std::cerr <<"Test Finished in " << (static_cast<double>(clock() - start) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

    StepList leftSteps = left->getSteps();
    StepList rightSteps = right->getSteps();

    std::cerr << "First test." << std::endl;
    iDynTree::assertTrue(printSteps(leftSteps, rightSteps));
    iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));



    std::cerr << std::endl << "------------------------------------------------------------------" << std::endl;
    std::cerr << "Second test." << std::endl;

    left->clearSteps();
    iDynTree::assertTrue(right->dropPastSteps());
    iDynTree::assertTrue(right->numberOfSteps() == 1);
    Step lastStep;
    iDynTree::assertTrue(right->getLastStep(lastStep));
    planner.clearPersonFollowingDesiredTrajectory();
    iDynTree::Vector2 dummyVector, newDesired;
    dummyVector.zero();
    newDesired(0) = lastStep.position(0) + 0.5;
    newDesired(1) = lastStep.position(1) + 0.5;
    iDynTree::assertTrue(planner.addPersonFollowingDesiredTrajectoryPoint(lastStep.impactTime+10, newDesired, dummyVector));

    iDynTree::assertTrue(planner.computeNewSteps(left, right, lastStep.impactTime, lastStep.impactTime+10));

    leftSteps = left->getSteps();
    rightSteps = right->getSteps();

    iDynTree::assertTrue(printSteps(leftSteps, rightSteps));

    iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));

    return true;
}

bool directControlTest(){

    Configuration conf;
    conf.initTime = 0.0;
    conf.endTime = 50.0;
    conf.dT = 0.01;
    conf.K = 10;
    conf.dX = 0.2;
    conf.dY = 0.0;
    conf.maxL = 0.2;
    conf.minL = 0.05;
    conf.minW = 0.08;
    conf.maxAngle = iDynTree::deg2rad(45);
    conf.minAngle = iDynTree::deg2rad(5);
    conf.nominalW = 0.14;
    conf.maxT = 10;
    conf.minT = 3;
    conf.nominalT = 4;
    conf.timeWeight = 2.5;
    conf.positionWeight = 1;
    conf.swingLeft = true;
    conf.slowWhenTurnGain = 0.5;

    UnicyclePlanner planner;

    //Initialization (some of these calls may be avoided)
    iDynTree::assertTrue(planner.setUnicycleController(UnicycleController::DIRECT));
    iDynTree::assertTrue(planner.setMaximumIntegratorStepSize(conf.dT));
    iDynTree::assertTrue(planner.setMaxStepLength(conf.maxL));
    iDynTree::assertTrue(planner.setWidthSetting(conf.minW, conf.nominalW));
    iDynTree::assertTrue(planner.setMaxAngleVariation(conf.maxAngle));
    iDynTree::assertTrue(planner.setCostWeights(conf.positionWeight, conf.timeWeight));
    iDynTree::assertTrue(planner.setStepTimings(conf.minT, conf.maxT, conf.nominalT));
    iDynTree::assertTrue(planner.setPlannerPeriod(conf.dT));
    iDynTree::assertTrue(planner.setMinimumAngleForNewSteps(conf.minAngle));
    iDynTree::assertTrue(planner.setMinimumStepLength(conf.minL));
    iDynTree::assertTrue(planner.setSlowWhenTurnGain(conf.slowWhenTurnGain));
    iDynTree::assertTrue(planner.setSlowWhenSidewaysFactor(0.2));
    iDynTree::assertTrue(planner.setSaturationsConservativeFactors(0.7, 0.7));

    planner.addTerminalStep(true);
    planner.startWithLeft(conf.swingLeft);

    //Generate desired trajectory

    planner.setDesiredDirectControl(10.0, 0.0, 0.0);

    std::shared_ptr<FootPrint> left, right;
    left = std::make_shared<FootPrint>();
    right = std::make_shared<FootPrint>();
    iDynTree::Vector2 initPosition;
    initPosition(0) = 0.3;
    initPosition(1) = -0.5;
    //left->addStep(initPosition, iDynTree::deg2rad(15), 25); //fake initialization

    clock_t start = clock();
    iDynTree::assertTrue(planner.computeNewSteps(left, right, conf.initTime, conf.endTime));
    std::cerr <<"Test Finished in " << (static_cast<double>(clock() - start) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

    StepList leftSteps = left->getSteps();
    StepList rightSteps = right->getSteps();

    std::cerr << "First test." << std::endl;
    iDynTree::assertTrue(printSteps(leftSteps, rightSteps));
    iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));



    std::cerr << std::endl << "------------------------------------------------------------------" << std::endl;
    std::cerr << "Second test." << std::endl;

    left->clearSteps();
    iDynTree::assertTrue(right->dropPastSteps());
    iDynTree::assertTrue(right->numberOfSteps() == 1);
    Step lastStep;
    iDynTree::assertTrue(right->getLastStep(lastStep));

    planner.setDesiredDirectControl(0.0, 10.0, 0.0);


    iDynTree::assertTrue(planner.computeNewSteps(left, right, lastStep.impactTime, lastStep.impactTime+10));

    leftSteps = left->getSteps();
    rightSteps = right->getSteps();

    iDynTree::assertTrue(printSteps(leftSteps, rightSteps));

    iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));


    std::cerr << std::endl << "------------------------------------------------------------------" << std::endl;
    std::cerr << "Third test." << std::endl;

    left->clearSteps();
    iDynTree::assertTrue(right->dropPastSteps());
    iDynTree::assertTrue(right->numberOfSteps() == 1);
    iDynTree::assertTrue(right->getLastStep(lastStep));

    planner.setDesiredDirectControl(0.0, 0.0, 10.0);


    iDynTree::assertTrue(planner.computeNewSteps(left, right, lastStep.impactTime, lastStep.impactTime+10));

    leftSteps = left->getSteps();
    rightSteps = right->getSteps();

    iDynTree::assertTrue(printSteps(leftSteps, rightSteps));

    iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));

    std::cerr << std::endl << "------------------------------------------------------------------" << std::endl;
    std::cerr << "Fourth test." << std::endl;

    left->clearSteps();
    iDynTree::assertTrue(right->dropPastSteps());
    iDynTree::assertTrue(right->numberOfSteps() == 1);
    iDynTree::assertTrue(right->getLastStep(lastStep));

    planner.setDesiredDirectControl(0.0, 0.0, -10.0);


    iDynTree::assertTrue(planner.computeNewSteps(left, right, lastStep.impactTime, lastStep.impactTime+10));

    leftSteps = left->getSteps();
    rightSteps = right->getSteps();

    iDynTree::assertTrue(printSteps(leftSteps, rightSteps));

    iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));

    return true;
}

int main(int argc, char **argv) {
    iDynTree::assertTrue(plannerTest());
    std::cerr << "----------------Direct Control Test -------------------" << std::endl;
    iDynTree::assertTrue(directControlTest());
    return EXIT_SUCCESS;
}
