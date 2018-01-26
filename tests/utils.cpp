/**
 * @file utils.cpp
 * @author Giulio Romualdi
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include "utils.h"

namespace Color {
  std::ostream& operator<<(std::ostream& os, const Color::Modifier& mod)
  {
    return os << "\033[" << mod.code << "m";
  }
}

void printStep(const std::deque<Step>& steps, std::ofstream& footsteps)
{
  for (auto step : steps){
    footsteps << step.position.toString() << " " << step.angle <<std::endl;

    std::cerr << "Position: [ "<< step.position.toString() <<"]" << std::endl;
    std::cerr << "Angle: "<< iDynTree::rad2deg(step.angle) << " degree"  << std::endl;
    std::cerr << "Impact time:  "<< step.impactTime << " seconds" << std::endl;
  }
}

void printSteps(const std::deque<Step>& leftSteps, const std::deque<Step>& rightSteps,
		const std::string& footstepsL, const std::string& footstepsR){

  Color::Modifier green(Color::FG_GREEN);
  Color::Modifier def(Color::FG_DEFAULT);

  std::ofstream footstepsLeftStream, footstepsRightStream;

  // open the streams
  footstepsLeftStream.open(footstepsL.c_str());
  footstepsRightStream.open(footstepsR.c_str());

  // stream the left foot position and attitude
  std::cerr << green << "Left foot "<< def << leftSteps.size() << " steps:"<< std::endl;
  footstepsLeftStream << "xL yL angleLeft"<<std::endl;
  printStep(leftSteps, footstepsLeftStream);

  // stream the right foot position and attitude
  std::cerr << green << "Right foot "<< def << rightSteps.size() << " steps:" << std::endl;
  footstepsRightStream << "xR yR angleRight"<<std::endl;
  printStep(rightSteps, footstepsRightStream);

  // close the streams
  footstepsLeftStream.close();
  footstepsRightStream.close();
}
