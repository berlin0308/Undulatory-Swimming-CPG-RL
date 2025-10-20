// File:          	webots_undulatory_swimming.cpp
// Date:          	23-11-2015
// Description:		controller to explore undulatory swimming with CPG / proprioceptive and exterceptive feedback
// Author:        	Robin Thandiackal, Florin Dzeladini
// Modifications: 	-

#include "anguilliformrobot.hpp"

void 
AnguilliformRobot :: initOptimizationParameters(int opti_type)
{
	// create the optimization handler
	optimization::Webots &opti = optimization::Webots::Instance();
	if(opti)
		optihand = new optihandler(opti,N_OPT_PARAMS,N_OPT_SETTINGS);

	opti_flag = 0;
	
	// set optimization variables
	if(opti)
	{
		// systematic search related to lesion study and feedback analysis
		if(opti_type==0) 
		{
			opti_flag = 1;
			control_mode_enable = 1;		// enable choice of control mode
			lesion_mode_enable = 1;			// enable choice of lesion mode
			motor_control_mode = 0;			// set motors to torque control

			maxSimTime = optihand->settings[0];			// max simulation time 
			control_mode = optihand->settings[1];		// control mode
			lesion_mode = optihand->settings[2];		// lesion mode
			// section_start_ind = optihand->settings[3];	// starting joint index of lesion section
			// section_end_ind = optihand->settings[4]; 	// ending joint index of lesion section
			section_start_ind = 0;
			section_end_ind = N_SERVOS-1;
			num_section_joints = section_end_ind - section_start_ind + 1;

			n_lesion_locations = optihand->params[0];	// number of lesions

			// parameter search for different control modes
			frequency = optihand->params[1]; 			// Network frequency
			parameter[0] = optihand->params[2];			// Torque gain: ALPHA (related to amplitude)
			parameter[1] = optihand->params[3];			// Stiffness: GAMMA 
			PHI_LAG_PERCENTAGE = optihand->params[4];	// nominal phase lag
			for (int i=0; i<N_SERVOS; i++)
			{
				ec_fb_gain[i] = optihand->params[5];	// fb gain 
			} 

			// for logging
			n_opti_params = 6;
			char *names[100] = {"nLesions","freq","alpha","gamma","phlag","fbgain"};
			for(uint i =0; i<n_opti_params;i++)
			{
				opti_params[i] = optihand->params[i];
				opti_params_names[i] = names[i];
			}
		}
		// systematic search related to position control and force profile characterization
		else if(opti_type==1)
		{
			opti_flag = 1;
			motor_control_mode = 1;		// set motors to position control
			control_mode_enable = 1;	// enable choice of control mode
			control_mode = 0;			// open loop control without feedback
			lesion_mode_enable = 0;		// disable lesion mode

			maxSimTime = optihand->settings[0];			// max simulation time
			pos_gradient_enable = optihand->settings[1];// gradient enabled/disabled			

			// parameter search
			frequency = optihand->params[0];			// network frequency
			pos_amplitude = optihand->params[1]; 		// amplitude (with gradient > tail amplitude) 
			PHI_LAG_PERCENTAGE = optihand->params[2];	// phase lag
			pos_gradient_start = optihand->params[3];	// head amplitude

			// for logging
			n_opti_params = 4;
			char *names[100] = {"freq","ampl","phlag","headampl"};
			for(uint i =0; i<n_opti_params;i++)
			{
				opti_params[i] = optihand->params[i];
				opti_params_names[i] = names[i];
			}
		}
		// systematic search related to resonance experiments 
		else if (opti_type==2)
		{
			opti_flag = 1;
			motor_control_mode = 0;		// set motors to torque control
			control_mode_enable = 1;	// enable choice of control mode
			control_mode = 5; 			// resonance experiments
			lesion_mode_enable = 0; 	// disable lesion mode

			maxSimTime = optihand->settings[0];				// max simulation time
			res_time_until_release = optihand->settings[1]; // time to settle before release
			res_gradient_enable = optihand->settings[2]; 	// 1: enable gradient in initial act

			// parameter search
			parameter[1] = optihand->params[0];				// Stiffness: GAMMA
			res_activation_level = optihand->params[1];		// Amount of initial bending via activation level

			// for logging
			n_opti_params = 3;
			char *names[100] = {"stiffness","activation_level","gradient_start"};
			for(uint i =0; i<n_opti_params;i++)
			{
				opti_params[i] = optihand->params[i];
				opti_params_names[i] = names[i];
			}
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

	cout << "opti type: " << OPTI_TYPE << " (0:lesion sys search / 1:pos cntrl sys search)" << endl;

	cout << "motor control mode: " << motor_control_mode << " (0:torque control / 1:position control)" << endl;  
	if(motor_control_mode==1)
	{
		cout << "position control amplitude: " << pos_amplitude << " [deg]" << endl;
		
		if(pos_gradient_enable==1)
		{
			cout << "gradient enabled with start gradient: " << pos_gradient_start << " [deg]" <<endl;
		}
	}

	if(lesion_mode_enable)
	{
		cout << "lesion_mode: " << lesion_mode << "(0: oscillations / 1: coupling / 2: feedback / 3: motoneuron / 4: combined lesions)" << endl;
		cout << "lesion mode type: " << lesion_mode_type << " (0: random / 1: systematic)" << endl;
		cout << "number of lesions: " << n_lesion_locations << endl;
		cout << "lesion section -> start: " << section_start_ind << " | end: " << section_end_ind << endl;
		cout << "lesion comb id: " << lesion_comb_id << endl;
	}
	else
	{
		cout << "lesion mode disabled" << endl;
	}

	if(control_mode_enable)
	{
		cout << "control_mode: " << control_mode << " (0:open loop/1:combined/2:decoupled/3:sensor-driven)" << endl;
	}
	else
	{
		cout << "control mode disabled" << endl;
	}
	
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