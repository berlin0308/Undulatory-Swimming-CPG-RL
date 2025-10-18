// File:          	webots_undulatory_swimming.cpp
// Date:          	23-11-2015
// Description:		controller to explore undulatory swimming with CPG / proprioceptive and exterceptive feedback
// Author:        	Robin Thandiackal, Florin Dzeladini
// Modifications: 	-

#include "anguilliformrobot.hpp"

void 
AnguilliformRobot :: initOptimizationParameters()
{
	// create the optimization handler
	optimization::Webots &opti = optimization::Webots::Instance();
	if(opti)
		optihand = new optihandler(opti,N_OPT_PARAMS,N_OPT_SETTINGS);

	opti_flag = 0;
	
	// set optimization variables
	if(opti)
	{
		control_mode_enable = optihand->settings[0];		// enable choice of control mode
		lesion_mode_enable = optihand->settings[1];			// enable choice of lesion mode

		control_mode = optihand->settings[2];		// control mode
		lesion_mode = optihand->settings[3];		// lesion mode
		maxSimTime = optihand->settings[4];			// max simulation time 

		// parameter search for different control modes		
		if(control_mode == 0)
		{
			// open loop
			frequency = optihand->params[0]; 			// Network frequency
			parameter[0] = optihand->params[1];			// Torque gain: ALPHA (related to amplitude)
			parameter[1] = optihand->params[2];			// Stiffness: GAMMA 
			PHI_LAG_PERCENTAGE = optihand->params[3];	// nominal phase lag
		}
		else if(control_mode == 1)
		{
			// combined
			frequency = optihand->params[0]; 			// Network frequency
			for (int i=0; i<N_SERVOS; i++)
			{
				ec_fb_gain[i] = optihand->params[1];	// fb gain 
			}
			parameter[0] = optihand->params[2];			// Torque gain: ALPHA 
			parameter[1] = optihand->params[3];			// Stiffness: GAMMA 
			PHI_LAG_PERCENTAGE = optihand->params[4];	// nominal phase lag
		}
		else if(control_mode == 2)
		{
			// closed loop decoupled
			frequency = optihand->params[0]; 			// Network frequency
			for (int i=0; i<N_SERVOS; i++)
			{
				ec_fb_gain[i] = optihand->params[1];	// fb gain 
			} 
			parameter[0] = optihand->params[2];			// Torque gain: ALPHA
			parameter[1] = optihand->params[3];			// Stiffness: GAMMA 
		}
		else if(control_mode == 3)
		{
			// closed loop omegazero
			for (int i=0; i<N_SERVOS; i++)
			{
				ec_fb_gain[i] = optihand->params[0];	// fb gain 
			}
			parameter[0] = optihand->params[1];			// Torque gain: ALPHA 
			parameter[1] = optihand->params[2];			// Stiffness: GAMMA
			PHI_LAG_PERCENTAGE = optihand->params[3];	// nominal phase lag 
		}
	}
	else
	{
		PHI_LAG_PERCENTAGE = DEFAULT_PHI_LAG_PERCENTAGE;
	}

	cout << "frequency: " << frequency << endl;
	cout << "fb gain: " << ec_fb_gain[0] << endl;
	cout << "alpha: " << parameter[0] << endl;
	cout << "gamma: " << parameter[1] << endl;
	cout << "delta: " << parameter[2] << endl;
	cout << "nominal phase lag: " << PHI_LAG_PERCENTAGE << endl;
	cout << "noise level: " << noise_level << endl;
	cout << "maxSimTime: " << maxSimTime << endl;
	cout << "simulationID: " << simuID << endl;

	if(lesion_mode_enable)
	{
		if(lesion_mode==0)
		{
			cout << "oscillations knocked-off" << "\t";
		}
		else if(lesion_mode==1)
		{
			cout << "couplings knocked-off" << "\t";
		}
		else if(lesion_mode==2)
		{
			cout << "feedback knocked-off" << "\t";
		}
		else if(lesion_mode==3)
		{
			cout << "motoneurons knocked-off" << "\t";	
		}
		cout << "number of lesions: " << n_lesion_locations << endl;
	}
	else
	{
		cout << "lesion mode disabled" << endl;
	}

	cout << "control_mode: " << control_mode << " (0:open loop/1:combined/2:decoupled/3:sensor-driven)" << endl;

	// set optimization fitness names
	fitness["speed"] = -1000.0;
	fitness["meanSpeed"] = -1000.0;
	fitness["stdSpeed"] = -1000.0;
	fitness["meanStdSpeed"] = -1000.0;
	fitness["randSeed"]=-1000.0;

	fitness["absEnergy"] = 0.0;
	fitness["posEnergy"] = 0.0;
	fitness["negEnergy"] = 0.0;
	fitness["avgFrequency"] = 0.0;
	fitness["avgPhaseLag"] = 0.0;
	fitness["avgPhaseLagKin"] = -1000;
	fitness["avgPhaseLagAct"] = -1000;
	fitness["avgZCStd"] = -1000.0;
	fitness["straightness"] = -1000.0;
}

void 
AnguilliformRobot :: returnFitness()
{
	// return fitness values to optimizer
	optimization::Webots &opti = optimization::Webots::Instance();
	if(opti)
	{
		optihand->returnFitness(opti, fitness);
	}

	// cout if not in optimization
	if(opti.Setting("optiextractor") || !opti)
	{
		cout << "speed" << " : " << fitness["speed"] << endl;
	}

	// quit simulation when optimizing
	if(opti && !opti.Setting("optiextractor"))
	{
		simulationQuit(0);
	}

}