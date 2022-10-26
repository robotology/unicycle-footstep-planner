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

#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

/*****************************************************************************************************************************/

class RosNode : public rclcpp::Node
{
private:
    /* consts */
    const std::string m_sub_topic_name = "/plan";
    const std::string m_pathPub_topic_name = "/unicycle_path_follower/dcm_path";
    const std::string m_ritgh_footprints_topic_name = "/unicycle_path_follower/right_footprints";
    const std::string m_left_footprints_topic_name = "/unicycle_path_follower/left_footprints";
    const std::string m_robot_frame = "projection";
    //const std::string m_world_frame = "map";
    //pubs and subs
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_dcm_pub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_right_footprint_markers_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_left_footprint_markers_pub;
    // TFs
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    
    //debug
    bool debug_once = false;

    void sub_callback(const nav_msgs::msg::Path::ConstPtr &msg_in)
    {
        try
        {
            if (debug_once)
            {
                RCLCPP_INFO(this->get_logger(), "Quitting callback");
                return;
            }
            
            // Each time a path is published I need to transform it to the robot frame
            geometry_msgs::msg::TransformStamped tf = m_tf_buffer_in->lookupTransform(m_robot_frame, msg_in->header.frame_id, rclcpp::Time(0));
            nav_msgs::msg::Path transformed_path = transformPlan(tf, msg_in, false);


            plannerTest(transformed_path);


            debug_once = true;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform global path to robot frame");
        }
    }

    nav_msgs::msg::Path transformPlan(geometry_msgs::msg::TransformStamped & tf_, const nav_msgs::msg::Path::ConstPtr &untransformed_path, bool prune_plan = true){
        if (untransformed_path->poses.empty()) {
            std::cerr << "Received plan with zero length" << std::endl;
            throw std::runtime_error("Received plan with zero length");
        }
        
        // let's get the pose of the robot in the frame of the plan

        // Transform the near part of the global plan into the robot's frame of reference.
        nav_msgs::msg::Path transformed_plan_ = *untransformed_path;
        transformed_plan_.header.frame_id = m_robot_frame;
        transformed_plan_.header.stamp = untransformed_path->header.stamp;  //could be removed

        //Transform the whole path (we could transform the path up to a certain point to save resources)
        //std::cout << "Transform the whole path for loop" << std::endl;
        for (int i = 0; i < untransformed_path->poses.size(); ++i)
        {
            tf2::doTransform(untransformed_path->poses.at(i), transformed_plan_.poses.at(i), tf_);
            std::cout << "Transformed X: " << transformed_plan_.poses.at(i).pose.position.x << "Transformed Y: " << transformed_plan_.poses.at(i).pose.position.y <<std::endl;
        }
        
        // Remove the portion of the global plan that is behind the robot
        if (prune_plan) {
            // Helper predicate lambda function to see what is the positive x-element in a vector of poses
            // Warning: The robot needs to have a portion of the path that goes forward (positive X)
            auto greaterThanZero = [](const geometry_msgs::msg::PoseStamped &i){
                return i.pose.position.x > 0.0;
            };

            transformed_plan_.poses.erase(begin(transformed_plan_.poses), 
                                        std::find_if(transformed_plan_.poses.begin(),
                                                transformed_plan_.poses.end(),
                                                greaterThanZero));
        }
        
        if (transformed_plan_.poses.empty()) {
            std::cerr << "Resulting plan has 0 poses in it." << std::endl;
            throw std::runtime_error("Resulting plan has 0 poses in it");
        }
        return transformed_plan_;
    }

    bool populateDesiredPath(UnicyclePlanner& planner, double initTime, double endTime, double dT, const nav_msgs::msg::Path &path, const double granularity=0.001){

        double t = initTime;    //0.0
        iDynTree::Vector2 yDes, yDotDes, polarCoordinates;    //yDes is the x, y pose 
        size_t index = 1;   //skip the first pose (should be too close to the robot origin and could be ignored)
        std::vector<iDynTree::Vector2> poses_history;
        // In theory, the first pose should be the reference frame in (0,0)
        double slope_angle = std::atan2(path.poses.at(index).pose.position.y, path.poses.at(index).pose.position.x);
        double speed = granularity/dT;

        while (t <= endTime){ 
            //Calculate the points separated by a fixed granularity
            if (poses_history.empty())
            {
                yDes(0) = granularity * std::cos(slope_angle);
                yDes(1) = granularity * std::sin(slope_angle);
                poses_history.push_back(yDes);
            }
            else
            {
                yDes(0) = granularity * std::cos(slope_angle) + poses_history.back()(0);
                yDes(1) = granularity * std::sin(slope_angle) + poses_history.back()(1);
                poses_history.push_back(yDes);
            }
            // Velocities
            yDotDes(0) = speed * std::cos(slope_angle);
            yDotDes(1) = speed * std::sin(slope_angle);
            
            // check if the next goal pose of the path is reached
            if (nextPoseReached(path, yDes, index, slope_angle))
            {
                ++index;
                //check if last posed is reached
                if (index >= path.poses.size() - 1)
                {
                    if(!planner.addPersonFollowingDesiredTrajectoryPoint(t, yDes, yDotDes))
                        return false;
                    break;
                }
                //update the slope angle with the direction of the next pose in the path
                //TODO check if use the latest pose from path or from poses_history
                slope_angle = std::atan2(path.poses.at(index + 1).pose.position.y - path.poses.at(index).pose.position.y, 
                                         path.poses.at(index + 1).pose.position.x - path.poses.at(index).pose.position.x);
            }
            
            if(!planner.addPersonFollowingDesiredTrajectoryPoint(t, yDes, yDotDes))
                return false;

            t += dT;
        }
        return true;
    }

    bool nextPoseReached (const nav_msgs::msg::Path &path, const iDynTree::Vector2 &pose, const size_t &current_index, const double &slope_angle){
        double distance_x = std::abs(path.poses.at(current_index).pose.position.x * std::cos(slope_angle)) -
                            std::abs(pose(0) * std::cos(slope_angle));
        double distance_y = std::abs(path.poses.at(current_index).pose.position.y * std::sin(slope_angle)) -
                            std::abs(pose(1) * std::sin(slope_angle));
        // condition if I am matching the pose or overshooting it
        if (distance_x >= 0 && distance_y >=0)
            return true;
        else
            return false;
    }

    bool setWaypoints(UnicyclePlanner& planner, double initTime, double endTime, const nav_msgs::msg::Path &path, const double freq=0.5){
        
        for (double i = initTime ; i < path.poses.size(); ++i)
        {
            if (i*freq > endTime)   // exit condition
            {
                break;
            }
            //do i need to transform each pose to the previous one? (shouldn't)
            iDynTree::Vector2 yDes;
            yDes(0) = path.poses.at(i).pose.position.x;
            yDes(1) = path.poses.at(i).pose.position.y; 
            planner.addPersonFollowingDesiredTrajectoryPoint(i * freq, yDes);   //scales each point
        }
        return true;
    }


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
    
    bool plannerTest(const nav_msgs::msg::Path &path){

        Configuration conf;
        conf.initTime = 0.0;
        conf.endTime = 10.0;    //50.0
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
        //iDynTree::assertTrue(populateDesiredTrajectory(planner, conf.initTime, conf.endTime, conf.dT));
        iDynTree::assertTrue(setWaypoints(planner, conf.initTime, conf.endTime, path));
        std::cerr <<"Populating the trajectory took " << (static_cast<double>(clock() - start) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

        std::shared_ptr<FootPrint> left, right;
        left = std::make_shared<FootPrint>();
        right = std::make_shared<FootPrint>();
        iDynTree::Vector2 initPosition;
        initPosition(0) = 0.0;      //0.3
        initPosition(1) = 0.0;      //-0.5
        //left->addStep(initPosition, iDynTree::deg2rad(15), 25); //fake initialization

        start = clock();
        iDynTree::assertTrue(planner.computeNewSteps(left, right, conf.initTime, conf.endTime));
        std::cerr <<"Test Finished in " << (static_cast<double>(clock() - start) / CLOCKS_PER_SEC) << " seconds."<<std::endl;

        StepList leftSteps = left->getSteps();
        StepList rightSteps = right->getSteps();

        std::cerr << "First test." << std::endl;
        iDynTree::assertTrue(printSteps(leftSteps, rightSteps));
        iDynTree::assertTrue(checkConstraints(leftSteps, rightSteps, conf));

        // publish markers on ros2
        iDynTree::assertTrue(publishMarkers(leftSteps, rightSteps));

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

    bool publishMarkers(std::deque<Step> leftSteps, std::deque<Step> rightSteps){
        visualization_msgs::msg::MarkerArray right_marker_array, left_marker_array;
        
        //LEFT
        visualization_msgs::msg::Marker tmp_marker_msg;
        tmp_marker_msg.header.frame_id = "/virtual_unicycle_base";
        tmp_marker_msg.header.stamp = rclcpp::Time(0);
        tmp_marker_msg.scale.x = 1.0;
        tmp_marker_msg.scale.y = 1.0;
        tmp_marker_msg.scale.z = 1.0;
        // Color for left foot
        tmp_marker_msg.color.r = 0.0;
        tmp_marker_msg.color.g = 1.0;
        tmp_marker_msg.color.b = 0.0;
        tmp_marker_msg.color.a = 1.0;
        tmp_marker_msg.type = tmp_marker_msg.ARROW;
        tmp_marker_msg.pose.position.z = 0.0;

        for (auto it = leftSteps.begin(); it != leftSteps.end(); ++it)
        {
            tmp_marker_msg.id = std::distance(leftSteps.begin(), it); 
            tmp_marker_msg.pose.position.x = it->position(0);
            tmp_marker_msg.pose.position.y = it->position(1);
            tf2::Quaternion q;
            q.setRPY(0, 0, it->angle);
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);

            left_marker_array.markers.push_back(tmp_marker_msg);
        }

        //RIGHT
        //change colour
        tmp_marker_msg.color.r = 0.0;
        tmp_marker_msg.color.g = 0.0;
        tmp_marker_msg.color.b = 1.0;
        tmp_marker_msg.color.a = 1.0;
        for (auto it = rightSteps.begin(); it != rightSteps.end(); ++it)
        {
            tmp_marker_msg.id = std::distance(rightSteps.begin(), it); 
            tmp_marker_msg.pose.position.x = it->position(0);
            tmp_marker_msg.pose.position.y = it->position(1);
            tf2::Quaternion q;
            q.setRPY(0, 0, it->angle);
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);

            right_marker_array.markers.push_back(tmp_marker_msg);
        }

        //publish
        m_left_footprint_markers_pub->publish(left_marker_array);
        m_right_footprint_markers_pub->publish(right_marker_array);
    }

public:
    RosNode() : rclcpp::Node("unicycle_path_follower_node")
    {
        // pubs and subs
        m_dcm_pub = this->create_publisher<nav_msgs::msg::Path>(m_pathPub_topic_name, 10);
        m_right_footprint_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_ritgh_footprints_topic_name, 10);
        m_left_footprint_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_left_footprints_topic_name, 10);
        m_path_sub = this->create_subscription<nav_msgs::msg::Path>(m_sub_topic_name,
                                                                    10,
                                                                    std::bind(&RosNode::sub_callback, this, _1)
                                                                    );
        //TFs
        m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);

    }
};


/*
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
*/
/*****************************************************************************************************************************/





int main(int argc, char **argv) {
    //iDynTree::assertTrue(plannerTest());
    //std::cerr << "----------------Direct Control Test -------------------" << std::endl;
    //iDynTree::assertTrue(directControlTest());
    // ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosNode>();
    if (rclcpp::ok()) 
    {
        std::cout << "Spinning odometry_standalone node" << std::endl;
        rclcpp::spin(node);
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    return EXIT_SUCCESS;
}
