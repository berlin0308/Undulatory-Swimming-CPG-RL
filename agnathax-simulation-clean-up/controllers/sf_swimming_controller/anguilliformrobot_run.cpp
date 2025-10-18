// File:          	webots_undulatory_swimming.cpp
// Date:          	23-11-2015
// Description:		controller to explore undulatory swimming with CPG / proprioceptive and exterceptive feedback
// Author:        	Robin Thandiackal, Florin Dzeladini
// Modifications: 	-

#include "anguilliformrobot.hpp"

void 
AnguilliformRobot :: run()
{
	// initial simulation step
	step(TIME_STEP); // to omit problems with sensor data in the first time step

	// general variables
	t = 0;
	double fb[N_SERVOS];
	double ef[N_SERVOS];
	double nb_fb[N_SERVOS];
	// double output[N_SERVOS_MAX] = {0};
	double normal_dir[N_SERVOS_MAX][3] = {{0}};
	// double external_force[N_SERVOS_MAX][3]= {{0}};
	// double external_normal_force[N_SERVOS_MAX]= {0};
	double work = 0.0;
	double kappa = 0;//-M_PI/2.0+8.0*M_PI/8.0;//-M_PI/2.0+8.0*M_PI/8.0;//0.0;
	bool averageFrequencyComputed = false;
	bool periodicSolution = true; 
	bool initial_theta_flag = false;
	
	// double meanSpeedVal, stdSpeedVal, meanStdSpeed, stdStdSpeed;

	int key = 0; // keyboard control 

	vector <double> activations;
	for(uint i=0; i<N_SERVOS; i++)
	{
		activations.push_back(0.0);
	}
	vector <double> angles;
	for(uint i=0; i<N_SERVOS_TOT; i++)
	{
		angles.push_back(0.0);
	}

	// prepare packets for physics plugin
	packet_controller_to_physics *emitterPacket = new packet_controller_to_physics();
	// packet_physics_to_controller *receiverPacket;

	emitterPacket->cntrl = CNTRL;

	for(uint i=0; i<N_MUSCLE_PARAMS; i++)
	{
		emitterPacket->pmtr[i] = parameter[i];
	}

	if(sizeof(*emitterPacket)>=1024)
	{
		cerr << " PACKET SIZE EXCEEDS BUFFER SIZE: LIMITED TO 1024 !!!! " << endl;
	}
	else
	{
		emitter->send(emitterPacket, sizeof(*emitterPacket));
	}

	// initialize position measurements
	posX.push_back(bodies[0]->getPosition()[0]); 
	posZ.push_back(bodies[0]->getPosition()[2]);

	do
	{

		// update oscillator network
	    cdn_network_step (d_cdn_network, TIME_STEP * 0.001);

		if (receiver->getQueueLength()>0){

			receiverPacket = (packet_physics_to_controller*)(receiver->getData());

			// key = keyboardGetKey();
			// if(key==KEYBOARD_UP){
			// 	kappa-=0.1;
			// 	cout << kappa << endl;
			// }
			// if(key==KEYBOARD_DOWN){
			// 	kappa+=0.1;
			// 	cout << kappa << endl;
			// }
			// if(key==KEYBOARD_HOME){
			// 	kappa=0.0;
			// 	cout << kappa << endl;
			// }	
		
			// kappa = min(kappa,M_PI/2);
			// kappa = max(kappa,-M_PI/2);

			// set passive joints in the tail to zero torque
			for(uint i=N_SERVOS; i<N_SERVOS_TOT;i++)
			{
				//cout << "controller AA" << i << endl;
				servo[i]->setForce(0.0);
			}
			// set feedback
		    for (uint i = 0; i < N_SERVOS; i++)
		    {
		    	if(control_mode==6) // turning experiments
		    	{
		    		double turning_duration = 1;
		    		if(t<=turning_duration)
		    		{
			    		double grad_start = 0;
			    		double grad_max = 1;
			    		double amp = grad_start+(grad_max-grad_start)*i/(N_SERVOS-1.0);
			    		if(i>=3 && i<10)
			    		{
			    			output[i] = 1;//amp * sin(2*M_PI/turning_duration*t-i/(N_SERVOS-1.0)*2*M_PI*0.2);
			    		}
		    		}
		    		else
		    		{
		    			output[i] = 0.0;
		    		} 
		    	}
		    	else if (control_mode==5) // resonance experiments
				{

						if(t<res_time_until_release)
						{
							output[i] = res_activation_level;
							if(pos_gradient_enable==1)
							{
								output[i] = res_gradient_start+(res_activation_level-res_gradient_start)*i/(N_SERVOS-1.0);
							}
							else
							{
								output[i] = res_activation_level;
							}
						}
						else
						{
							output[i] = 0.0;
						}
				}
				else if (control_mode == 7)
				{
					if(t<twitch_duration)
					{
						if(i==twitch_joint)
						{
							output[i] = twitch_activation_level;
						}
					}
					else
					{
						output[i] = 0.0;
					}
				}
				else
		    	{
		    		// phase oscillator model
		    		if(cpg_model_type==0){
			    		output[i] = AMP*cdn_variable_get_value(d_cdn_x[i]);

			    		// proprioceptive sensory feedback
				    	// fb[i] = (1.0+i*0.5/N_SERVOS) * servo[i]->getPosition();

				    	fb[i] = servo[i]->getPosition();
				    	cdn_variable_set_value(d_cdn_sf[i],PC_FB_GAIN*fb[i]);

				    	// feedback phase shift
				    	cdn_variable_set_value(d_cdn_kappa[i],kappa);
			    	}
			    	else if(cpg_model_type==1){
			    		output[i] = AMP*cdn_variable_get_value(d_cdn_xx_a[i])-AMP*cdn_variable_get_value(d_cdn_xx_b[i]);
			    		output[i] = 0.1*output[i];
			    	}

			    	// exteroceptive sensory feedback
			    	if(t>0)
			    	{
			    		//ef[i] = external_normal_force[i];
			    		
			    		//ef[i] = external_normal_force[i+1]; 
			    		
			    		//ef[i] = (external_normal_force[i]+external_normal_force[i+1])/2.0;
			    		// if(i<N_SERVOS-2){
			    		// 	ef[i] = external_normal_force[i+3];
			    		// }
			    		// else{
			    		// 	ef[i] = 0.0;//external_normal_force[i+1];
			    		// }
			    		// if(i<N_SERVOS-1){
			    		// 	ef[i] = external_normal_force[i+2];//external_normal_force[i+1];
			    		// }	
			    		// else{
			    		// 	ef[i] = 0.0;
			    		// }

			    		double force_sum = 0;
			    		for(int k=0;k<N_SERVOS+1;k++)
			    		{
			    			force_sum += fmat[i][k]*external_normal_force[k];
			    			//cout << fmat[i][k] << " ";
			    		}
			    		//cout << endl;
			    		ef[i] = force_sum;


			    		// double inhib_gain = -0.1;

			    		// if(i==0){
			    		// 	nb_fb[i] = inhib_gain*external_normal_force[i+2]*cdn_variable_get_value(d_cdn_x[i+1]);
			    		// }
			    		// else if(i>0 && i<N_SERVOS-1){
			    		// 	nb_fb[i] = inhib_gain*(external_normal_force[i]*cdn_variable_get_value(d_cdn_x[i-1])
			    		// 						+external_normal_force[i+2]*cdn_variable_get_value(d_cdn_x[i+1]));
			    		// }
			    		// else if(i==N_SERVOS-1){
			    		// 	nb_fb[i] = inhib_gain*external_normal_force[i]*cdn_variable_get_value(d_cdn_x[i-1]);
			    		// }
			    	}
			    	else
			    	{
			    		ef[i] = 0;
			    	}
			    	
			    	if(decay_enable==0)
			    	{
			    		cdn_variable_set_value(d_cdn_ef[i],ec_fb_gain[i]*ef[i]);
			    	}
			    	else
			    	{
				    	// feedback decay 
				    	if(t>0 && t<=decay_start_time)
				    	{
				    		cdn_variable_set_value(d_cdn_ef[i],ec_fb_gain[i]*ef[i]);
				    	}
				    	else if(t>decay_start_time && t<(decay_start_time+decay_duration))
				    	{
				    		// linear decay
				    		cout << "decay!" << endl;
				    		//cdn_variable_set_value(d_cdn_ef[i],(ec_fb_gain[i]-ec_fb_gain[i]/decay_duration*(t-decay_start_time))*ef[i]);
				    		// exponential decay
				    		cdn_variable_set_value(d_cdn_ef[i],(ec_fb_gain[i]*exp(-decay_param*(t-decay_start_time)))*ef[i]);
				    	}
				    	else
				    	{
				    		// cout << "zero fdbgain" << endl;
				    		cdn_variable_set_value(d_cdn_ef[i],0);
				    	}
			    	}	
		    		// cdn_variable_set_value(d_cdn_nb_fb[i],0 * ec_fb_gain[i]*nb_fb[i]);

		    		// cout << i << ": " << cdn_variable_get_value(d_cdn_freq[i]) << endl;

		    		// if (i==5)
		    		// {
		    		// 	cout << "fb term 5-th joint: "<< ef[i]*ec_fb_gain[i]*output[i]<< endl;
		    		// }
	    		}

		    }
			// set activations
			for(uint i=0; i<N_SERVOS; i++)
			{
				if(CNTRL==0) // torque control
				{
					servo[i]->setForce(0);

					emitterPacket->actv[i][0] = output[i];
					emitterPacket->actv[i][1] = 0.0;

				}
				else if(CNTRL==1) // position control
				{
					servo[i]->setControlP(50);
					if(pos_gradient_enable==1)
					{
						servo[i]->setPosition(-output[i]/AMP*(pos_gradient_start+(pos_amplitude-pos_gradient_start)*i/(N_SERVOS-1.0))/180.0*M_PI);
					}
					else
					{
						servo[i]->setPosition(-output[i]/AMP*pos_amplitude/180.0*M_PI);
					}
					emitterPacket->actv[i][0] = 0.0;
					emitterPacket->actv[i][1] = 0.0;
				}
				else
				{
					cout << "control mode unknown ..." << endl;
				}

				activations[i] = output[i];

			}

			// average frequency
			if(t>T_FREQ_EST_START+T_FREQ_EST_DURATION)
			{
				if(averageFrequencyComputed == false)
				{
					periodicSolution = averageFrequency();
					averageFrequencyComputed = true;
					for(uint i=0; i<N_SERVOS; i++)
					{
						jointAngles[i].clear();	 // clear so that it can be used again for phase lag estimation in last cycles
					}
				}
			}

			// joint angles for frequency estimation
			// and head position for straightness measure
			if(t>T_FREQ_EST_START && t<T_FREQ_EST_START+T_FREQ_EST_DURATION) // assuming slowest frequencies of 1/tFreqEstDuration
			{
				for(uint i=0; i<N_SERVOS; i++)
				{
					jointAngles[i].push_back(angles[i]);
				}
				headPosX.push_back(bodies[0]->getPosition()[0]);
				headPosZ.push_back(bodies[0]->getPosition()[2]);
			}

			// save theta initial condition
			if(initial_theta_flag==false && cpg_model_type==0)
			{
				initialCondition();
				initial_theta_flag = true;
			}

			if(periodicSolution==true){
				// update angles
				for(uint i=0; i<N_SERVOS_TOT; i++)
				{
					angles[i] = servo[i]->getPosition();
					// in the last cycle
					if(t>maxSimTime-1.0/avgFrequency)
					{
						end_cycle_ang[i].push_back(angles[i]);
					}
				}

				// update energy in last cycle
				for(uint i=0; i<N_SERVOS; i++)
				{	
					if(t>maxSimTime-1.0/avgFrequency)
					{
						work = (receiverPacket->torq[i])*(receiverPacket->speed[i]);
						absEnergy += fabs(work);
						(work>=0 ? posEnergy += work : negEnergy += work);
					}
				}

				// phase lags within 2 last cycles
				if(t>maxSimTime-3.0/avgFrequency)
				{
					for(uint i=0; i<N_SERVOS; i++)
					{
						jointAngles[i].push_back(angles[i]);
						jointActivs[i].push_back(activations[i]);
					}
				}

				// compute performance metrics
				// speed
				instantenousSpeed(N_SPEED_MEASURE_CYCLES/avgFrequency, SPEED_TIME_CONSTANT);
				meanStdVector(speed,meanSpeedVal,stdSpeedVal);
				meanStdVector(stdSpeed,meanStdSpeed,stdStdSpeed);	


			}
			else{
				//break;
			} // end periodic solution

		    // communication with physics plugin (activate muscles and apply torque)
			if(sizeof(*emitterPacket)>=1024)
			{
				cerr << " PACKET SIZE EXCEEDS BUFFER SIZE: LIMITED TO 1024 !!!! " << endl;
				break;
			}
			else
			{
				emitter->send(emitterPacket, sizeof(*emitterPacket));
			}

			// compute external hydrodynamics forces in normal direction
			for(uint i=0; i<N_SERVOS_TOT+1; i++)
			{
				external_force[i][0] = receiverPacket->forces[i][0];
				external_force[i][1] = receiverPacket->forces[i][1];
				external_force[i][2] = receiverPacket->forces[i][2];

				if(i==0)
				{
					normal_dir[i][0] = head_body->getOrientation()[0];
					normal_dir[i][1] = head_body->getOrientation()[3];
					normal_dir[i][2] = head_body->getOrientation()[6];

					// cout << "thrust force: " << sqrt((external_force[i][0])*(external_force[i][0]) + (external_force[i][1])*(external_force[i][1]) +(external_force[i][2])*(external_force[i][2])) << endl;					
				}
				else
				{
					normal_dir[i][0] = bodies[i-1]->getOrientation()[0];
					normal_dir[i][1] = bodies[i-1]->getOrientation()[3];
					normal_dir[i][2] = bodies[i-1]->getOrientation()[6];
				}
				projection(normal_dir[i],external_force[i],external_normal_force[i]);

				// add noise
				int rnd = rand();
				external_normal_force[i]*=(1.0+((rnd%101)*2-100.0)*0.01*noise_level/100.0);
				// if(i==5){
				// 	cout << external_normal_force[i] << endl;
				// }
				// cout << "random number: " << (1.0+((rnd%101)*2-100.0)*0.01*noise_level/100.0) << endl;
			}

			// logging
			logging();

			receiver->nextPacket(); // very important!!!!! (deletes the head packet, the next packet becomes new head!)
		}

		t += TIME_STEP*0.001;

	} while (step(TIME_STEP) != -1 && t<=maxSimTime);

	delete emitterPacket;

	// stop logging
	closeLogFiles();

	// stop movie recording
	if(REC_MOVIE==1)
	{
		stopMovieRec();
	}

	if(periodicSolution)
	{
		// compute amplitudes
		amplitudes();

		// compute average phase lags
		// averagePhaseLag();
		periodicSolution = individualPhaseLags();

		// straightness
		straightness();

		// return fitness
		meanStdVector(speed,meanSpeedVal,stdSpeedVal);
		meanStdVector(stdSpeed,meanStdSpeed,stdStdSpeed);

	}
	fitness["isNAN"] = 0;

	// return fitness
	fitness["speed"] = (isnan(instSpeed)) ? -1000.0: instSpeed;
	fitness["meanSpeed"]  = (isnan(meanSpeedVal)) ? -1000.0: meanSpeedVal;
	fitness["stdSpeed"] = (isnan(stdSpeedVal)) ? -1000.0: stdSpeedVal;
	fitness["meanStdSpeed"] = (isnan(meanStdSpeed)) ? -1000.0: meanStdSpeed;
	fitness["randSeed"] = (isnan(d_cdn_seed)) ? -1000.0: d_cdn_seed;
	fitness["absEnergy"] = (isnan(absEnergy)) ? -1000.0: absEnergy;
	fitness["posEnergy"] = (isnan(posEnergy)) ? -1000.0: posEnergy;
	fitness["negEnergy"] = (isnan(negEnergy)) ? -1000.0: negEnergy;
	fitness["avgFrequency"] = (isnan(avgFrequency)) ? -1000.0: avgFrequency;
	// fitness["avgPhaseLag"] = (isnan(avgPhaseLag)) ? -1000.0: avgPhaseLag;
	fitness["avgPhaseLagAct"] = (isnan(avgPhaseLagAct)) ? -1000.0: avgPhaseLagAct;
	fitness["avgPhaseLagKin"] = (isnan(avgPhaseLagKin)) ? -1000.0: avgPhaseLagKin;
	fitness["avgZCStd"] = (isnan(avgZCStd)) ? -1000.0: avgZCStd;
	fitness["straightness"] = (isnan(straigthnessMeas)) ? -1000.0: straigthnessMeas;
	fitness["periodicSolution"] = periodicSolution;
	

	if(isnan(instSpeed) || isnan(meanSpeedVal) || isnan(stdSpeedVal) || isnan(meanStdSpeed) || isnan(d_cdn_seed) || isnan(absEnergy) || isnan(posEnergy) || isnan(negEnergy) || isnan(avgFrequency) || isnan(avgPhaseLag) || isnan(avgPhaseLagAct) || isnan(avgPhaseLagKin) || isnan(straigthnessMeas) )
		fitness["isNAN"] = 1;	

	std::stringstream ss;
	for(uint i=0; i<N_SERVOS; i++)
	{
		ss << std::string("ampl") << i;
		fitness[ss.str()] = (isnan(ampl[i])) ? -1000.0: ampl[i];
		ss.str("");
		if(fitness["isNAN"] == 0 && (isnan(ampl[i])) )
			fitness["isNAN"] = 1;
	}
	for(uint i=0; i<N_SERVOS-1; i++)
	{
		ss << std::string("phiLAct") << i;
		fitness[ss.str()] = (isnan(phaseLagAct[i])) ? -1000.0: phaseLagAct[i]; //avgIndividualPhaseLag[i];
		ss.str("");
		if(fitness["isNAN"] == 0 && (isnan(phaseLagAct[i])) )
			fitness["isNAN"] = 1;
	}
	for(uint i=0; i<N_SERVOS-1; i++)
	{
		ss << std::string("phiLKin") << i;
		fitness[ss.str()] = (isnan(phaseLagKin[i])) ? -1000.0: phaseLagKin[i]; //avgIndividualPhaseLag[i];
		ss.str("");
		if(fitness["isNAN"] == 0 && (isnan(phaseLagKin[i])) )
			fitness["isNAN"] = 1;
	}
	for(uint i=0; i<N_SERVOS-1; i++)
	{
		ss << std::string("initTheta") << i;
		fitness[ss.str()] = (isnan(initTheta[i])) ? -1000.0: initTheta[i];
		ss.str("");
		if(fitness["isNAN"] == 0 && (isnan(initTheta[i])) )
			fitness["isNAN"] = 1;
	}
	for(uint i=0; i<N_SERVOS-1; i++)
	{
		ss << std::string("zcStd") << i;
		fitness[ss.str()] = (isnan(zcStd[i])) ? -1000.0: zcStd[i];
		ss.str("");
		if(fitness["isNAN"] == 0 && (isnan(zcStd[i])) )
			fitness["isNAN"] = 1;
	}

	cout << "fitness: " << endl;
	cout << "inst speed: " << fitness["speed"] << endl;
	cout << "mean speed: " << fitness["meanSpeed"] << endl;
	cout << "std speed: " << fitness["stdSpeed"] << endl;
	cout << "mean std speed: " << fitness["meanStdSpeed"] << endl;
	cout << "random seed: " << fitness["randSeed"] << endl;
	cout << "energy: " << absEnergy << "(abs) " << posEnergy << "(pos) " << negEnergy << "(neg)" << endl;
	cout << "average frequency: " << avgFrequency << endl;
	cout << "amplitudes: ";
	for(uint i=0; i<N_SERVOS;i++)
	{
		cout << ampl[i] << "//"; 
	}
	cout << endl;
	cout << "phase lags (act): ";
	for(uint i=0; i<N_SERVOS-1;i++)
	{
		cout << phaseLagAct[i] << "//"; 
	}
	cout << endl;
	cout << "phase lags (kin): ";
	for(uint i=0; i<N_SERVOS-1;i++)
	{
		cout << phaseLagKin[i] << "//"; 
	}
	cout << endl;
	cout << "initial thetas: ";
	for(uint i=0; i<N_SERVOS-1;i++)
	{
		cout << initTheta[i] << "//"; 
	}
	cout << endl;
	cout << "zero crossings std: ";
	for(uint i=0; i<N_SERVOS-1;i++)
	{
		cout << zcStd[i] << "//"; 
	}
	cout << endl;

	cout << "average phase lag (Act): " << fitness["avgPhaseLagAct"] << endl; 
	cout << "average phase lag (Kin): " << fitness["avgPhaseLagKin"] << endl;
	cout << "average zero crossing std: " << fitness["avgZCStd"] << endl;
	cout << "straightness: " << fitness["straightness"] << endl;
	if(periodicSolution==false)
		cout << "non-periodic solution !!!!!" << endl;

	returnFitness();
}

void
AnguilliformRobot :: logging()
{

	if(LOG==1 && !opti_flag)
	{
		// time
		log_time << t << endl;
		// external forces (3D)
		for (uint i = 0; i < N_SERVOS_TOT; i++)
		{
			for (uint j = 0; j < 3; j++)
			{
				log_ext_forces << external_force[i][j] <<"\t";  	
			}
		}
		// external normal forces
		for (uint i = 0; i < N_SERVOS_TOT; i++)
		{
			log_norm_forces << external_normal_force[i] <<"\t";	
		}
		// torque patterns
		for (uint i = 0; i < N_SERVOS; i++)
		{
			log_torques << receiverPacket->torq[i] <<"\t";  	
		}
		// muscle activation patterns
		for (uint i = 0; i < N_SERVOS; i++)
		{
			log_act << output[i] <<"\t";  	
		}
		// joint angles 
		for (uint i = 0; i < N_SERVOS; i++)
		{
			log_joint_ang << servo[i]->getPosition() <<"\t";  	
		}
		// oscillator: theta
		for (uint i = 0; i < N_SERVOS; i++)
		{
			log_theta << cdn_variable_get_value(d_cdn_theta[i]) << "\t";
		}
		// segment (incl. tail fin) position
		for (uint j = 0; j < 3; j++)
		{
			log_body_pos << head_body->getPosition()[j] << "\t";
		}
		for (uint i = 0; i < N_SERVOS_TOT; i++)
		{
			for (uint j = 0; j < 3; j++)
			{
				log_body_pos << bodies[i]->getPosition()[j] << "\t";
			}
		}	
		// segment (incl. tail fin) orientation
		for (uint j = 0; j < 3; j++)
		{
			log_body_orient << head_body->getOrientation()[3*j] << "\t";
		}
		for (uint i = 0; i < N_SERVOS_TOT; i++)
		{
			for (uint j = 0; j < 3; j++)
			{
				log_body_orient << bodies[i]->getOrientation()[3*j] << "\t";
			}
		}
		// speed
		log_speed << instSpeed << "\t" << meanSpeedVal << "\t" << stdSpeedVal<< "\t" << meanStdSpeed << "\t";// << steadyState << "\t";

		// next line
		log_ext_forces << endl;
		log_norm_forces << endl;
		log_torques << endl;
		log_act << endl;
		log_joint_ang << endl;
		log_theta << endl;
		log_body_pos << endl;
		log_body_orient << endl;
		log_speed << endl;
	}
	else if(LOG==2)
	{
		// alternative logging
		int t_int = (int)(1000*t);
		fprintf(log_all,"%d\t",t_int);
		// fprintf(log_all,"%.3f\t",t);
		// external normal forces
		for (uint i = 0; i < N_SERVOS_TOT+1; i++)
		{
			fprintf(log_all,"%.5f\t",external_normal_force[i]);  	
		}
		// torque patterns
		for (uint i = 0; i < N_SERVOS; i++)
		{
			if(motor_control_mode==0)
			{
				fprintf(log_all,"%.5f\t",receiverPacket->torq[i]);
			}
			else if(motor_control_mode==1)
			{
				fprintf(log_all,"%.5f\t",servo[i]->getMotorForceFeedback());
			}
			else
			{
				fprintf(log_all,"%.5f\t",0);
			}
		}
		// muscle activation patterns
		for (uint i = 0; i < N_SERVOS; i++)
		{
			fprintf(log_all,"%.5f\t",output[i]);  	
		}
		// joint angles 
		for (uint i = 0; i < N_SERVOS_TOT; i++)
		{
			fprintf(log_all,"%.5f\t",servo[i]->getPosition());  	
		}
		// oscillator: theta
		if(cpg_model_type==0){
			for (uint i = 0; i < N_SERVOS; i++)
			{
				fprintf(log_all,"%.3f\t",cdn_variable_get_value(d_cdn_theta[i]));
			}
		}
		else if(cpg_model_type==1){
			for (uint i = 0; i < N_SERVOS; i++)
			{
				fprintf(log_all,"%.3f\t",cdn_variable_get_value(d_cdn_xx_a[i]));
			}
			for (uint i = 0; i < N_SERVOS; i++)
			{
				fprintf(log_all,"%.3f\t",cdn_variable_get_value(d_cdn_xx_b[i]));
			}
		}
		// head position
		for (uint j = 0; j < 3; j++)
		{
			fprintf(log_all,"%.1f\t",head_body->getPosition()[j]*1000);
			// fprintf(log_all,"%.5f\t",head_body->getPosition()[j]);
		}
		// head orientation
		for (uint j = 0; j < 3; j++)
		{
			fprintf(log_all,"%.5f\t",head_body->getOrientation()[3*j]);
		}
		// 3D external forces
		for (uint i = 0; i < N_SERVOS_TOT+1; i++)
		{
			for (uint j = 0; j < 3; j++)
			{
				fprintf(log_all,"%.5f\t",external_force[i][j]);	
			}
		}
		// next line
		fprintf(log_all,"\n");
	}
}

void 
AnguilliformRobot :: closeLogFiles()
{
	if(LOG==1 &&!opti_flag)
	{
		log_time.close();
		log_ext_forces.close();
		log_norm_forces.close();
		log_torques.close();
		log_act.close();
		log_joint_ang.close();
		log_theta.close();
		log_body_pos.close();
		log_body_orient.close();
		log_speed.close();
		log_peaks.close();
		log_periods.close();
		log_amplitude.close();
	}
	// alternative logging
	if(LOG==2)
	{
		fclose(log_all);
	}
}

void 
AnguilliformRobot :: stopMovieRec()
{
	// stop movie recording
	stopMovie();

	// quit simulation or restart simulation
	//simulationRevert();
}
