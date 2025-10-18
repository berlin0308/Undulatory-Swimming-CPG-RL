// File:          	webots_undulatory_swimming.cpp
// Date:          	23-11-2015
// Description:		controller to explore undulatory swimming with CPG / proprioceptive and exterceptive feedback
// Author:        	Robin Thandiackal, Florin Dzeladini
// Modifications: 	-

#include "anguilliformrobot.hpp"



// CONSTRUCTOR ----------------------------------------------------------------
AnguilliformRobot :: AnguilliformRobot()
{
	// network parameters
	frequency = FREQ;
	
	// viscoelastic parameters
	for(uint i=0; i<N_SERVOS_MAX; i++)
	{
		parameter[0] = ALPHA;
		parameter[1] = GAMMA;//s_gain*1.2;//1.2
		parameter[2] = DELTA;//0.2;//0.8;
	}

	// fbgain and motoneuron strength
	for(uint i=0; i<N_SERVOS_MAX; i++)
	{
		ec_fb_gain[i] = EC_FB_GAIN;
	}

	// max time (default)
	maxSimTime = MAX_TIME;

	// noise level 
	noise_level = DEFAULT_NOISE_LEVEL;

	// simulation ID (fake parameter)
	simuID = -1000;

	// default phase lag
	PHI_LAG_PERCENTAGE = DEFAULT_PHI_LAG_PERCENTAGE;

	// default modes
	control_mode_enable = CONTROL_MODE_ENABLE;
	control_mode = CONTROL_MODE;
	lesion_mode_enable = LESION_MODE_ENABLE;
	lesion_mode = LESION_MODE;
	n_lesion_locations = DEFAULT_N_LESIONS;

	// initializiation
	// cout << "updating structural parameters ..." << endl;
	updateStructuralParameters();
	//stiffnessGradient();
	// cout << "init opti parameters ..." << endl;
	initOptimizationParameters();
	// cout << "init servos ..." << endl;
	initServos();
	// cout << "init Bodies ..." << endl;
	initBodies();
	// cout << "init GPS ..." << endl;
	initGPS();
	// cout << "init CPG ..." << endl;
	initCPG();
	// enable keyboard
	keyboardEnable(TIME_STEP);

	// initialize random number seed for noise generation
	srand(d_cdn_seed);

	// lesion initialization
	if (lesion_mode_enable)
	{
		lesionInit(n_lesion_locations);
	}

	// mode initialization
	if (control_mode_enable)
	{
		modeInit();
	}
	
	// logging
	if(LOG)
	{
		log_time.open("log_time.txt");
		log_ext_forces.open("log_ext_forces.txt");
		log_norm_forces.open("log_norm_forces.txt");
		log_torques.open("log_torques.txt");	
		log_act.open("log_act.txt");
		log_joint_ang.open("log_joint_ang.txt");
		log_theta.open("log_theta.txt");
		log_body_pos.open("log_body_pos.txt");
		log_body_orient.open("log_body_orient.txt");
		log_speed.open("log_speed.txt");
		log_peaks.open("log_peaks.txt");
		log_periods.open("log_periods.txt");
		log_amplitude.open("log_amplitude.txt");
	}

	// energy
	absEnergy = 0.0;
	posEnergy = 0.0;
	negEnergy = 0.0;

	// avgFrequency
	avgFrequency = 20;//FREQ;
	avgPhaseLag = -1000;
	avgPhaseLagKin = -1000;	
	avgPhaseLagAct = -1000;
	avgZCStd = -1000;
	// phase lag and amplitude
	for(uint i=0; i<N_SERVOS-1;i++)
	{
		avgIndividualPhaseLag[i] = -1000;
		phaseLagKin[i] = -1000.0;
		phaseLagAct[i] = -1000.0;
		ampl[i] = -1000;
		zcStd[i] = -1000;
	}

	// straightness
	straigthnessMeas = -1000;

}

//----------------------------------------------------------------
string AnguilliformRobot :: getJointNameLowerCase(uint i){
	const char * JOINT_NAME_PREFIX = "joint_servo_";
	std::stringstream ss;
	ss << std::string(JOINT_NAME_PREFIX) << i;
	return ss.str();
}
string AnguilliformRobot :: getJointNameUpperCase(uint i){
	std::string NAME = getJointNameLowerCase(i);
	std::transform(NAME.begin(), NAME.end(), NAME.begin(), ::toupper);
	return NAME;
}
string AnguilliformRobot :: getJointPhysicsNameUpperCase(uint i){
	std::string NAME = getJointNameUpperCase(i);
	std::stringstream ss;
	ss << "PHYSICS_" << NAME;
	return ss.str();
}

bool AnguilliformRobot :: hasJoint(uint i){
	if(this->getFromDef(getJointNameUpperCase(i)) == NULL )
		return false;
	return true;
}

void AnguilliformRobot :: updateStructuralParameters() {
	uint i;
	for(i=1;i<=N_SERVOS_MAX;i++){
		if(!hasJoint(i))
			break;
	}

	N_SERVOS_TOT = i-1;
 	N_SERVOS = N_SERVOS_TOT - N_SERVOS_TAIL;//i-1-10; // THE -10 is to remove the tail joint
}

void AnguilliformRobot :: initServos() {
	uint i;
	if(N_SERVOS_TOT == 0) 
		cerr << "Error N_SERVOS_TOT=0, did you run updateStructuralParameters() ?" << endl;
	for(i=1;i<=N_SERVOS_TOT;i++)
	{
		if(hasJoint(i)){
			servo[i-1] = new webots::Servo(getJointNameLowerCase(i));
			servo[i-1]->enablePosition(TIME_STEP);
			servo[i-1]->enableMotorForceFeedback(TIME_STEP);
		}
	}

	cout << "N_SERVOS: " << N_SERVOS << endl;
	cout << "N_SERVOS_TAIL: " << N_SERVOS_TAIL << endl;
	cout << "N_SERVOS_TOT: " << N_SERVOS_TOT << endl;
	cout << "N_SERVOS_MAX: " << N_SERVOS_MAX << endl;
}

void AnguilliformRobot :: initBodies() {
	uint i;
	if(N_SERVOS_TOT == 0) 
		cerr << "Error N_SERVOS_TOT=0, did you run updateStructuralParameters() ?" << endl;
	for(i=1;i<=N_SERVOS_TOT+1;i++)
	{
		if(hasJoint(i)){
			bodies[i-1] = this->getFromDef(getJointPhysicsNameUpperCase(i));
		}
	}
	head_body = this->getFromDef("PHYSICS_ROBOT");
}

void AnguilliformRobot :: initGPS(){
	// get gps
  	gps1 = new webots::GPS("gps1");
  	gps1->enable(TIME_STEP);
  
  	// get emitter and receiver
	emitter = new webots::Emitter("emitter");
	receiver = new webots::Receiver("receiver");
	receiver->enable(TIME_STEP);

}

void AnguilliformRobot :: initCPG(){

	// fixed phase lag for CPG
 	PHI_LAG = PHI_LAG_PERCENTAGE/100.0*2*M_PI/N_SERVOS;

	// CPG network (implemented with codyn)
	// initialize CpgNetwork (with coupling weights, frequency, phase lags)
	char const *filename = "cpg_single_chain.cdn";
	stringstream ss; 
	ss << "defines { N = " << N_SERVOS << " } include \"" << filename << "\"";
	d_cdn_network = cdn_network_new_from_string(ss.str().c_str(), NULL);

	(RAND_FIXED_SEED==false) ? d_cdn_seed = rand()%100000 : d_cdn_seed = RAND_FIXED_SEED_VALUE;
	cout << "random seed: " << d_cdn_seed << endl;
	cdn_network_set_random_seed	(d_cdn_network,d_cdn_seed);

	if(d_cdn_network == NULL) 
	{
		fprintf(stderr, "Could not open network '%s'\n", filename);
	}
	
	if (!cdn_object_compile (CDN_OBJECT (d_cdn_network), NULL, NULL)) 
	{
		fprintf(stderr, "Could not compile network '%s'\n", filename);
	}
	
	// oscillators
	// x
	char *selector = g_strdup_printf("\"osc{1:%d}\".x", N_SERVOS);
	GSList *list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
	free(selector);
	GSList *item = list;
	uint i;
	for (i=0; item && i<N_SERVOS; i++) 
	{
		d_cdn_x[i] = CDN_VARIABLE (item->data);
		item = item->next;
	}
	printf("Found %d axial oscillators\n", i);
	g_slist_free(list);


	// theta
	selector = g_strdup_printf("\"osc{1:%d}\".theta", N_SERVOS);
	list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
	free(selector);
	item = list;
	for (i=0; item && i<N_SERVOS; i++)
	{
		d_cdn_theta[i] = CDN_VARIABLE (item->data);
		item = item->next;
	}
	g_slist_free(list); 

	// theta_dot
	selector = g_strdup_printf("\"osc{1:%d}\".theta_dot", N_SERVOS);
	list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
	free(selector);
	item = list;
	for (i=0; item && i<N_SERVOS; i++)
	{
		d_cdn_theta_dot[i] = CDN_VARIABLE (item->data);
		item = item->next;
	}
	g_slist_free(list); 

	// proprioceptive feedback
	selector = g_strdup_printf("\"osc{1:%d}\".s", N_SERVOS);

	list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
	free(selector);
	item = list;
	for (i=0; item && i<N_SERVOS; i++)
	{
		d_cdn_sf[i] = CDN_VARIABLE (item->data);
		item = item->next;
	}
	g_slist_free(list); 

	// exteroceptive sensory feedback
	selector = g_strdup_printf("\"osc{1:%d}\".e", N_SERVOS);
	list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
	free(selector);
	item = list;
	for (i=0; item && i<N_SERVOS; i++)
	{
		d_cdn_ef[i] = CDN_VARIABLE (item->data);
		item = item->next;
	}
	g_slist_free(list);

	// kappa parameter
	selector = g_strdup_printf("\"osc{1:%d}\".kappa", N_SERVOS);
	list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
	free(selector);
	item = list;
	for (i=0; item && i<N_SERVOS; i++)
	{
		d_cdn_kappa[i] = CDN_VARIABLE (item->data);
		item = item->next;
	}
	g_slist_free(list);

	// freq
	selector = g_strdup_printf("\"osc{1:%d}\".f", N_SERVOS);
	list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
	free(selector);
	item = list;
	for (i=0; item && i<N_SERVOS; i++)
	{
		d_cdn_freq[i] = CDN_VARIABLE (item->data);
		item = item->next;
	}
	g_slist_free(list);

	// motoneuron output strength (r)
	selector = g_strdup_printf("\"osc{1:%d}\".r", N_SERVOS);
	list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
	free(selector);
	item = list;
	for (i=0; item && i<N_SERVOS; i++)
	{
		d_cdn_r[i] = CDN_VARIABLE (item->data);
		item = item->next;
	}
	g_slist_free(list); 

	// couplings
	selector = g_strdup_printf("\"edge_{1:%d}\".w", N_SERVOS*2);
	list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
	free(selector);
	item = list;
	for (i=0; item && i<N_SERVOS; i++)
	{
		d_cdn_couplings[i][0] = CDN_VARIABLE (item->data);
		item = item->next;
		d_cdn_couplings[i][1] = CDN_VARIABLE (item->data);
		item = item->next;
	}
	g_slist_free(list);

	// freq, phlag, weights
	d_cdn_init_freq = cdn_node_find_variable (CDN_NODE (d_cdn_network), "FREQ");
	d_cdn_phlag = cdn_node_find_variable (CDN_NODE (d_cdn_network), "PHI_LAG");
	d_cdn_wup = cdn_node_find_variable (CDN_NODE (d_cdn_network), "W_UP");
	d_cdn_wdown = cdn_node_find_variable (CDN_NODE (d_cdn_network), "W_DOWN");

	cdn_variable_set_value(d_cdn_init_freq, frequency);
	cout << "frequency: " << frequency << endl;
	cdn_variable_set_value(d_cdn_phlag, PHI_LAG);
	cdn_variable_set_value(d_cdn_wup, W_UP);
	cdn_variable_set_value(d_cdn_wdown, W_DOWN);

}

void
AnguilliformRobot::lesionInit(int nLocations) // lesion in nLocations
{

	int idx = 0;
	int segment_list[N_SERVOS];
	int coupling_list[N_SERVOS-1];

	for(int i=0; i<N_SERVOS; i++)
	{
		segment_list[i] = i;
		if (i<N_SERVOS-1) 
			coupling_list[i] = i;
		cout << segment_list[i] << "\t";
	}	
	cout << endl;
	random_shuffle(segment_list, segment_list + N_SERVOS);
	random_shuffle(coupling_list, coupling_list + N_SERVOS-1);

	for(int i=0; i<N_SERVOS; i++)
	{
		cout << segment_list[i] << "\t";
	}	
	cout << endl;


	if (lesion_mode==0)
	{
		// oscillation knock-off
		for(int i=0; i<nLocations; i++)
		{
			idx = segment_list[i];
			cdn_variable_set_value(d_cdn_freq[idx],0);
		}
	}
	else if (lesion_mode==1)
	{
		// coupling knock-off
		for(int i=0; i<nLocations; i++)
		{
			if(i<N_SERVOS-1) // one less coupling then segments
			{
				idx = coupling_list[i];
				cdn_variable_set_value(d_cdn_couplings[idx][0],0);
				cdn_variable_set_value(d_cdn_couplings[idx][1],0);
			}
		}
	}
	else if (lesion_mode==2)
	{
		// feedback knock-off
		for(int i=0; i<nLocations; i++)
		{
			idx = segment_list[i];
			ec_fb_gain[idx]=0;
		}
	}
	else if (lesion_mode==3)
	{
		// motoneuron knock-off
		for(int i=0; i<nLocations; i++)
		{
			idx = segment_list[i];
			cdn_variable_set_value(d_cdn_r[idx],0);
		}
	}
	else if (lesion_mode==4)
	{
		int segment_list2[N_SERVOS], segment_list3[N_SERVOS];
		for(int i=0; i<N_SERVOS; i++)
		{
			segment_list2[i] = segment_list[i];
			segment_list3[i] = segment_list[i];
		}
		random_shuffle(segment_list2, segment_list2 + N_SERVOS);
		random_shuffle(segment_list3, segment_list3 + N_SERVOS);

		// combined lesion (oscillation, coupling and feedback)
		cout << "combined lesion:" << endl;
		for(int i=0; i<nLocations; i++)
		{
			switch(control_mode)
			{
				// open loop model
				case 0:
					switch(rand()%2) 
					{
						case 0:
							// oscillations
							idx = segment_list[i];
							cdn_variable_set_value(d_cdn_freq[idx],0);
							cout << "oscillator lesion: " << idx << endl;
							break;
						case 1:
							// coupling
							idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
							cdn_variable_set_value(d_cdn_couplings[idx][0],0);
							cdn_variable_set_value(d_cdn_couplings[idx][1],0);
							cout << "coupling lesion: " << idx << endl;
							break;
					}
					break;

				// combined model 
				case 1:
					switch(rand()%3) 
					{
						case 0:
							// oscillations
							idx = segment_list[i];
							cdn_variable_set_value(d_cdn_freq[idx],0);
							cout << "oscillator lesion: " << idx << endl;
							break;
						case 1:
							// coupling
							idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
							cdn_variable_set_value(d_cdn_couplings[idx][0],0);
							cdn_variable_set_value(d_cdn_couplings[idx][1],0);
							cout << "coupling lesion: " << idx << endl;
							break;
						case 2:
							// feedback
							idx = segment_list3[i];
							ec_fb_gain[idx]=0;
							cout << "feedback lesion: " << idx << endl;
							break;
					}
					break;

				// decoupled model
				case 2:
					switch(rand()%2) 
					{
						case 0:
							// oscillations
							idx = segment_list[i];
							cdn_variable_set_value(d_cdn_freq[idx],0);
							cout << "oscillator lesion: " << idx << endl;
							break;
						case 1:
							// feedback
							idx = segment_list3[i];
							ec_fb_gain[idx]=0;
							cout << "feedback lesion: " << idx << endl;
							break;
					}
					break;

				// sensory-driven model
				case 3:
					switch(rand()%2) 
					{
						case 0:
							// coupling
							idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
							cdn_variable_set_value(d_cdn_couplings[idx][0],0);
							cdn_variable_set_value(d_cdn_couplings[idx][1],0);
							cout << "coupling lesion: " << idx << endl;
							break;
						case 1:
							// feedback
							idx = segment_list3[i];
							ec_fb_gain[idx]=0;
							cout << "feedback lesion: " << idx << endl;
							break;
					}
					break;
			}
		}
	}
	else if (lesion_mode==5)
	{
		// combined lesion all at same location (oscillation, coupling and feedback)
		for(int i=0; i<nLocations; i++)
		{
			idx = segment_list[i];
			// oscillations
			cdn_variable_set_value(d_cdn_freq[idx],0);	
			// coupling
			cdn_variable_set_value(d_cdn_couplings[(int)round((float)idx/N_SERVOS*(N_SERVOS-1))][0],0);
			cdn_variable_set_value(d_cdn_couplings[idx][1],0);
			// feedback
			ec_fb_gain[idx]=0;

			cout << "lesion location: " << idx << endl;
		}
	}

}

void
AnguilliformRobot :: modeInit()
{
	// open loop
	if (control_mode==0)
	{
		for(int i=0; i<N_SERVOS; i++)
		{
			ec_fb_gain[i]=0;
		}
	}
	// combined model
	else if (control_mode==1)
	{
		// take as is
	}
	// decoupled model
	else if (control_mode==2)
	{
		for(int i=0; i<N_SERVOS-1; i++)
		{
			cdn_variable_set_value(d_cdn_couplings[i][0],0);
			cdn_variable_set_value(d_cdn_couplings[i][1],0);
		}
	}
	// sensory-driven
	else if (control_mode==3)
	{
		for(int i=0; i<N_SERVOS; i++)
		{
			cdn_variable_set_value(d_cdn_freq[i],0);
		}
	}
}
