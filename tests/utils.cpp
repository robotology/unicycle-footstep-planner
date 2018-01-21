#include "utils.h"

void printSteps(const std::deque<Step>& leftSteps, const std::deque<Step>& rightSteps,
		const std::string& footstepsL, const std::string& footstepsR){

  std::ofstream footstepsLeftStream, footstepsRightStream;
  
  footstepsLeftStream.open(footstepsL.c_str());
  footstepsRightStream.open(footstepsR.c_str());

  footstepsLeftStream << "xL yL angleL"<<std::endl;
  
  std::cerr << "Left foot "<< leftSteps.size() << " steps:"<< std::endl;
  for (auto step : leftSteps){
    footstepsLeftStream << step.position.toString() << " " << step.angle <<std::endl;

    std::cerr << "Position "<< step.position.toString() << std::endl;
    std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
    std::cerr << "Time  "<< step.impactTime << std::endl;
  }

  footstepsRightStream << "xR yR angleR"<<std::endl;
  std::cerr << std::endl << "Right foot "<< rightSteps.size() << " steps:" << std::endl;
  for (auto step : rightSteps){
    footstepsRightStream << step.position.toString() << " " << step.angle <<std::endl;
    
    std::cerr << "Position "<< step.position.toString() << std::endl;
    std::cerr << "Angle "<< iDynTree::rad2deg(step.angle) << std::endl;
    std::cerr << "Time  "<< step.impactTime << std::endl;
  }

  footstepsLeftStream.close();
  footstepsRightStream.close();
}


