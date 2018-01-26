/**
 * @file utils.h
 * @author Giulio Romualdi
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <ostream>
#include <fstream>

#include "UnicycleTrajectoryGenerator.h"

/**
 * Color namespace
 */
namespace Color {
  enum Code {
    FG_RED      = 31,
    FG_GREEN    = 32,
    FG_BLUE     = 34,
    FG_DEFAULT  = 39,
    BG_RED      = 41,
    BG_GREEN    = 42,
    BG_BLUE     = 44,
    BG_DEFAULT  = 49
  };

  /**
   * Modifier class
   */
  class Modifier
  {
    Code code /** < Color code. */;

  public:

    /**
     * Modifier constructor
     * @param pCode is the color code
     */
  Modifier(Code pCode):
    code(pCode)
    {}

    /**
     * Overloading << operator.
     */
    friend std::ostream& operator<<(std::ostream& os, const Modifier& mod);
  };
}

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
  std::cerr << "[ " ;
  for (auto pose : objectName){
    std::cerr << pose << " ";
  }
  std::cerr << "]" << std::endl;
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

#endif
