// File:          	webots_undulatory_swimming.cpp
// Date:          	23-11-2015
// Description:		controller to explore undulatory swimming with CPG / proprioceptive and exterceptive feedback
// Author:        	Robin Thandiackal, Florin Dzeladini
// Modifications: 	-

#include "anguilliformrobot.hpp"

unsigned int N_SERVOS;
unsigned int N_SERVOS_TOT;
float PHI_LAG;	// [rad]		
float PHI_LAG_PERCENTAGE;// [%]// [rad]


// MAIN -----------------------------------------------------------------------
int main(int argc, char **argv)
{
  AnguilliformRobot* controller = new AnguilliformRobot();
  controller->run();
  delete controller;
  return 0;
}