/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Giulio Romualdi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "UnicycleTrajectoryGenerator.h"
 #include <fstream> 

/**
 * Print left and right footsteps into a file
 * @param leftFootsteps is a vector containing the left footsteps 
 * @param rightFootsteps is a vector containing the right footsteps 
 * @param leftFootstepsFileName is the name of the file that will contain the left footsteps
 * @param rightFootstepsFileName is the name of the file that will contain the right footsteps
 */
void printSteps(const std::deque<Step>& leftFootsteps, const std::deque<Step>& rightFootsteps,
		const std::string& leftFootstepsFileName, const std::string& rightFootstepsFileName);

/**
 * Print and iDynTree vector in the shell
 * @param objectName iDynTree vector
 */
template<class object>
void print_iDynTree(object& objectName){
  for (auto pose : objectName){
    std::cerr << pose.toString() << std::endl;
  }
}

/**
 * Print and iDynTree vector in a file
 * @param objectName is an iDynTree vector
 * @param file is the output stream class
 */
template<class object>
void print_iDynTree(object& objectName, std::ofstream& file){
  for (auto pose : objectName){
    file << pose.toString() << std::endl;
  }
}

/**
 * Print and iDynTree vector in the shell
 * @param objectName is a v vector
 */
template<class object>
void printVector(object& objectName){
  for (auto pose : objectName){
    std::cerr << pose << std::endl;
  }
}

/**
 * Print and iDynTree vector in the shell
 * @param objectName is a v vector
 * @param file is the output stream class
 */
template<class object>
void printVector(object& objectName, std::ofstream& file){
  for (auto pose : objectName){
    file << pose << std::endl;
  }
}
