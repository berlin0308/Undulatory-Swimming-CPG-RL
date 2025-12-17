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

	// default position control amplitude
	pos_amplitude = POS_AMPLITUDE;

	// default resonance experiments parameters
	res_time_until_release = RES_TIME_UNTIL_RELEASE;
	res_activation_level = RES_ACTIVATION_LEVEL;
	res_gradient_enable = RES_GRADIENT_ENABLE;
	res_gradient_start = RES_GRADIENT_START;

	// default twitching parameters
	twitch_duration = TWITCH_DURATION;
	twitch_activation_level = TWITCH_ACTIVATION_LEVEL; 
	twitch_joint = TWITCH_JOINT;

	// default modes
	control_mode_enable = CONTROL_MODE_ENABLE;
	control_mode = CONTROL_MODE;
	lesion_mode_enable = LESION_MODE_ENABLE;
	lesion_mode_type = LESION_MODE_TYPE;
	lesion_mode = LESION_MODE;
	section_start_ind = SECTION_START_IND;
	section_end_ind = SECTION_END_IND;
	num_section_joints = section_end_ind - section_start_ind + 1;
	n_lesion_locations = min(DEFAULT_N_LESIONS,num_section_joints);
	lesion_comb_id = LESION_COMB_ID;
	motor_control_mode = CNTRL;
	pos_gradient_enable = POS_GRADIENT_ENABLE;
	pos_gradient_start = POS_GRADIENT_START;
	cpg_model_type = CPG_MODEL_TYPE;
	rand_fixed_seed_value = RAND_FIXED_SEED_VALUE;

	// feedback decay
	decay_enable=DECAY_ENABLE;
	decay_start_time=DECAY_START_TIME;
	decay_duration=DECAY_DURATION;
	decay_param=DECAY_PARAM;

	if(REC_MOVIE==1)
	{
		readFile();
	}

	// initializiation
	// cout << "updating structural parameters ..." << endl;
	updateStructuralParameters();
	// force coupling matrix
	readForceCouplingMatrixFromFile();
	//stiffnessGradient();
	// cout << "init opti parameters ..." << endl;
	initOptimizationParameters(OPTI_TYPE);
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
		switch(lesion_mode_type)
		{		
			case 0:
				initLesion();
				break;
			case 1:
				initSystematicLesion();
				break;
		}
	}

	// mode initialization
	if (control_mode_enable)
	{
		initMode();
	}

	if(LOG)
	{
		initLogging();
	}

	// start movie if recording is enabled
	if(REC_MOVIE==1)
	{
		startMovieRec();
	}
	
	// // logging
	// if(LOG)
	// {
	// 	log_time.open("log_time.txt");
	// 	log_ext_forces.open("log_ext_forces.txt");
	// 	log_norm_forces.open("log_norm_forces.txt");
	// 	log_torques.open("log_torques.txt");	
	// 	log_act.open("log_act.txt");
	// 	log_joint_ang.open("log_joint_ang.txt");
	// 	log_theta.open("log_theta.txt");
	// 	log_body_pos.open("log_body_pos.txt");
	// 	log_body_orient.open("log_body_orient.txt");
	// 	log_speed.open("log_speed.txt");
	// 	log_peaks.open("log_peaks.txt");
	// 	log_periods.open("log_periods.txt");
	// 	log_amplitude.open("log_amplitude.txt");
	// }

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

	const char *filename_phase_osc_model = "cpg_single_chain.cdn";
	const char *filename_matsuoka_model = "matsuoka_network.cdn";
	const char *filename = NULL;
	if(cpg_model_type==0){  
		filename = filename_phase_osc_model;
	}
	else if(cpg_model_type==1){
		filename = filename_matsuoka_model;
	}
	else{

	}
	stringstream ss; 
	ss << "defines { N = " << N_SERVOS << " } include \"" << filename << "\"";
	d_cdn_network = cdn_network_new_from_string(ss.str().c_str(), NULL);

	(RAND_FIXED_SEED==false) ? d_cdn_seed = rand()%100000 : d_cdn_seed = rand_fixed_seed_value;
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
	
	// phase oscillator model
	if(cpg_model_type==0){
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

		// neighbor feedback
		selector = g_strdup_printf("\"osc{1:%d}\".nb_fb", N_SERVOS);
		list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
		free(selector);
		item = list;
		for (i=0; item && i<N_SERVOS; i++)
		{
			d_cdn_nb_fb[i] = CDN_VARIABLE (item->data);
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
	else if(cpg_model_type==1){

		// left neurons
		char *selector = g_strdup_printf("\"osc{1:%d}\".xx_a", N_SERVOS);
		GSList *list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
		free(selector);
		GSList *item = list;
		uint i;
		for (i=0; item && i<N_SERVOS; i++) 
		{
			d_cdn_xx_a[i] = CDN_VARIABLE (item->data);
			item = item->next;
		}
		printf("Found %d axial oscillators a\n", i);
		g_slist_free(list);
		
		// right neurons
		selector = g_strdup_printf("\"osc{1:%d}\".xx_b", N_SERVOS);
		list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
		free(selector);
		item = list;
		for (i=0; item && i<N_SERVOS; i++) 
		{
			d_cdn_xx_b[i] = CDN_VARIABLE (item->data);
			item = item->next;
		}
		printf("Found %d axial oscillators b\n", i);
		g_slist_free(list);

		// exteroceptive sensory feedback   																	  
		selector = g_strdup_printf("\"osc{1:%d}\".force", N_SERVOS);
		list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
		free(selector);
		item = list;
		for (i=0; item && i<N_SERVOS; i++)
		{
			d_cdn_ef[i] = CDN_VARIABLE (item->data);
			item = item->next;
		}
		g_slist_free(list);
		printf("Found %d force fb\n", i);

		// left neurons////////////////////////////////////////////////////////////////////////////////////////////
		// left neurons
		selector = g_strdup_printf("\"osc{1:%d}\".x_a", N_SERVOS);
		list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
		free(selector);
		item = list;
		for (i=0; item && i<N_SERVOS; i++) 
		{
			d_cdn_x_a[i] = CDN_VARIABLE (item->data);
			item = item->next;
		}
		printf("Found %d axial oscillators aa\n", i);
		
		// right neurons
		selector = g_strdup_printf("\"osc{1:%d}\".x_b", N_SERVOS);
		list = cdn_node_find_variables(CDN_NODE (d_cdn_network), selector);
		free(selector);
		item = list;
		for (i=0; item && i<N_SERVOS; i++) 
		{
			d_cdn_x_b[i] = CDN_VARIABLE (item->data);
			item = item->next;
		}
		printf("Found %d axial oscillators bb\n", i);
		g_slist_free(list);

		////////////////////////////////////////////////////////////////////////////////////////////
		// parameters
		d_cdn_tau      = cdn_node_find_variable (CDN_NODE (d_cdn_network), "Tau");	
		d_cdn_beta     = cdn_node_find_variable (CDN_NODE (d_cdn_network), "Beta");
		d_cdn_gamma    = cdn_node_find_variable (CDN_NODE (d_cdn_network), "Gamma");
		d_cdn_eta      = cdn_node_find_variable (CDN_NODE (d_cdn_network), "Eta");
		d_cdn_eta_e    = cdn_node_find_variable (CDN_NODE (d_cdn_network), "Eta_e");
		d_cdn_eta_i    = cdn_node_find_variable (CDN_NODE (d_cdn_network), "Eta_i");
		d_cdn_eta_head = cdn_node_find_variable (CDN_NODE (d_cdn_network), "Eta_head");
		d_cdn_tonic    = cdn_node_find_variable (CDN_NODE (d_cdn_network), "Tonic");
		d_cdn_fb_i     = cdn_node_find_variable (CDN_NODE (d_cdn_network), "FB_I");
		d_cdn_fb_e     = cdn_node_find_variable (CDN_NODE (d_cdn_network), "FB_E");

		cout << "finished init cpg" << endl;
	}
	else{

	}

}

void 
AnguilliformRobot::initSystematicLesion()
{

	// // initialize
	// for(int i=0; i<N_SERVOS; i++)
	// {
	// 	lesion_osc[i] = false;
	// 	lesion_fbk[i] = false;
	// 	lesion_mtn[i] = false;
	// 	if (i<section_end_ind){ 
	// 		lesion_cpl[i] = false;
	// 	}
	// }

	// cout << "lesion comb: ";
	// // represent lesion_comb_id as binary number and get corresponding lesion states through bitshifts
	// for(int i=section_end_ind; i>=section_start_ind; i--)
	// {
	// 	cout << (lesion_comb_id & 1) << "|" ;
	// 	if((lesion_comb_id & 1) == 1){
	// 		lesion_osc[i] = true;
	// 		lesion_fbk[i] = true;
	// 		lesion_mtn[i] = true;
	// 		if(i>section_start_ind){
	// 			lesion_cpl[i-1] = true;
	// 		}
	// 	}
	// 	lesion_comb_id = lesion_comb_id >> 1;
	// }
	// cout << endl;

	// // lesion display
	// cout << "osc/fdbk/mtn lesions: ";
	// for(int i=0; i<N_SERVOS; i++)
	// {
	// 	cout << lesion_osc[i] << " ";
	// }
	// cout << endl;
	// cout << "cpl lesions: ";
	// for(int i=0; i<N_SERVOS-1; i++)
	// {
	// 	cout << lesion_cpl[i] << " ";
	// }
	// cout << endl;

	// // apply lesions
	// for(int i=0; i<N_SERVOS; i++)
	// {
	// 	switch(lesion_mode)
	// 	{
	// 		case 0:
	// 			if(lesion_osc[i]==true)
	// 			{
	// 				cdn_variable_set_value(d_cdn_freq[i],0);
	// 			}
	// 			break;
	// 		case 1:
	// 			if(lesion_cpl[i]==true && i<N_SERVOS-1)
	// 			{
	// 				cdn_variable_set_value(d_cdn_couplings[i][0],0);
	// 				cdn_variable_set_value(d_cdn_couplings[i][1],0);
	// 			}
	// 			break;
	// 		case 2: 
	// 			if(lesion_fbk[i]==true)
	// 			{
	// 				ec_fb_gain[i]=0;
	// 			}
	// 			break;
	// 	}
	// }

	bool preset_lesion_osc[10] = {0,0,0,0,0,1,1,1,1,1};
	bool preset_lesion_cpl[9] =  {1,1,1,1,1,0,0,0,0};

	// bool preset_lesion_osc[10] = {1,1,1,1,1,0,0,0,0,0};
	// bool preset_lesion_cpl[9] =  {0,0,0,0,1,1,1,1,1};

	// bool preset_lesion_osc[10] = {0,0,1,1,1,0,0,0,1,1};
	// bool preset_lesion_cpl[9] =  {1,1,0,0,1,1,1,1,0};

	// bool preset_lesion_osc[10] = {1,1,0,0,0,1,1,1,0,0};
	// bool preset_lesion_cpl[9] =  {0,1,1,1,1,0,0,1,1};

	bool preset_lesion_fdb[10] = {0,0,0,0,0,0,0,0,0,0};

	for(int i=0; i<N_SERVOS; i++)
	{
		if(preset_lesion_osc[i]==true)
		{
			cdn_variable_set_value(d_cdn_freq[i],0);
		}

		if(i<N_SERVOS-1 && preset_lesion_cpl[i]==true)
		{
			cdn_variable_set_value(d_cdn_couplings[i][0],0);
			cdn_variable_set_value(d_cdn_couplings[i][1],0);	
		}

		if(preset_lesion_fdb[i]==true)
		{
			ec_fb_gain[i]=0;
		}
	}

}

void
AnguilliformRobot::initLesion() // lesion in nLocations
{

	// lesion_osc = new bool[N_SERVOS];
	// lesion_cpl = new bool[N_SERVOS-1];
	// lesion_fbk = new bool[N_SERVOS];
	// lesion_mtn = new bool[N_SERVOS];
	int nLocations = n_lesion_locations;

	int idx = 0;

	// int segment_list[N_SERVOS];
	// int coupling_list[N_SERVOS-1];

	// for(int i=0; i<N_SERVOS; i++)
	// {
	// 	lesion_osc[i] = false;
	// 	lesion_fbk[i] = false;
	// 	lesion_mtn[i] = false;

	// 	segment_list[i] = i;
	// 	if (i<N_SERVOS-1){ 
	// 		lesion_cpl[i] = false;

	// 		coupling_list[i] = i;
	// 	}
		
	// }	

	// random_shuffle(segment_list, segment_list + N_SERVOS);
	// random_shuffle(coupling_list, coupling_list + N_SERVOS-1);

	// cout << "shuffled lesion segment list:" << endl;
	// for(int i=0; i<N_SERVOS; i++)
	// {
	// 	cout << segment_list[i] << "\t";
	// }
	// cout << endl;
	// cout << "shuffled lesion coupling list:" << endl;
	// for(int i=0; i<N_SERVOS-1; i++)
	// {
	// 	cout << coupling_list[i] << "\t";
	// }	
	// cout << endl;

	int *segment_list = new int[num_section_joints];
	int *coupling_list = new int[num_section_joints-1];

	for(int i=0; i<N_SERVOS; i++)
	{
		lesion_osc[i] = false;
		lesion_fbk[i] = false;
		lesion_mtn[i] = false;
		if (i<section_end_ind){ 
			lesion_cpl[i] = false;
		}
	}

	for(int i=section_start_ind; i<=section_end_ind; i++)
	{
		
		segment_list[i-section_start_ind] = i;
		cout << segment_list[i-section_start_ind] << endl;
		if (i<section_end_ind){ 
			coupling_list[i-section_start_ind] = i;
		}
	}	

	random_shuffle(segment_list, segment_list + num_section_joints);
	random_shuffle(coupling_list, coupling_list + num_section_joints-1);

	cout << "shuffled lesion segment list:" << endl;
	for(int i=0; i<num_section_joints; i++)
	{
		cout << segment_list[i] << "\t";
	}
	cout << endl;
	cout << "shuffled lesion coupling list:" << endl;
	for(int i=0; i<num_section_joints-1; i++)
	{
		cout << coupling_list[i] << "\t";
	}	
	cout << endl;

	if (lesion_mode==0)
	{
		// oscillation knock-off
		for(int i=0; i<nLocations; i++)
		{
			idx = segment_list[i];
			cdn_variable_set_value(d_cdn_freq[idx],0);

			lesion_osc[idx] = true;
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

				lesion_cpl[idx] = true;
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

			lesion_fbk[idx] = true;
		}
	}
	else if (lesion_mode==3)
	{
		// motoneuron knock-off
		for(int i=0; i<nLocations; i++)
		{
			idx = segment_list[i];
			cdn_variable_set_value(d_cdn_r[idx],0);

			lesion_mtn[idx] = true;
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
							lesion_osc[idx] = true;
							break;
						case 1:
							// coupling
							idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
							cdn_variable_set_value(d_cdn_couplings[idx][0],0);
							cdn_variable_set_value(d_cdn_couplings[idx][1],0);
							cout << "coupling lesion: " << idx << endl;
							lesion_cpl[idx] = true;
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
							lesion_osc[idx] = true;
							break;
						case 1:
							// coupling
							idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
							cdn_variable_set_value(d_cdn_couplings[idx][0],0);
							cdn_variable_set_value(d_cdn_couplings[idx][1],0);
							cout << "coupling lesion: " << idx << endl;
							lesion_cpl[idx] = true;
							break;
						case 2:
							// feedback
							idx = segment_list3[i];
							ec_fb_gain[idx]=0;
							cout << "feedback lesion: " << idx << endl;
							lesion_fbk[idx] = true;
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
							lesion_osc[idx] = true;
							break;
						case 1:
							// feedback
							idx = segment_list3[i];
							ec_fb_gain[idx]=0;
							cout << "feedback lesion: " << idx << endl;
							lesion_fbk[idx] = true;
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
							lesion_cpl[idx] = true;
							break;
						case 1:
							// feedback
							idx = segment_list3[i];
							ec_fb_gain[idx]=0;
							cout << "feedback lesion: " << idx << endl;
							lesion_fbk[idx] = true;
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

			lesion_osc[idx] = true;
			lesion_cpl[(int)round((float)idx/N_SERVOS*(N_SERVOS-1))] = true;
			lesion_fbk[idx] = true;
		}
	}
	else if (lesion_mode==6)
	{
		// combined lesions: all types at the same time but at different random locations
		int segment_list2[N_SERVOS], segment_list3[N_SERVOS];
		for(int i=0; i<N_SERVOS; i++)
		{
			segment_list2[i] = segment_list[i];
			segment_list3[i] = segment_list[i];
		}
		random_shuffle(segment_list2, segment_list2 + N_SERVOS);
		random_shuffle(segment_list3, segment_list3 + N_SERVOS);

		// combined lesion (oscillation, coupling and feedback)
		cout << "combined lesion (all types together but at diff locs):" << endl;
		for(int i=0; i<nLocations; i++)
		{
			switch(control_mode)
			{
				// open loop model
				case 0:
					// oscillations
					idx = segment_list[i];
					cdn_variable_set_value(d_cdn_freq[idx],0);
					cout << "oscillator lesion: " << idx << endl;
					lesion_osc[idx] = true;
					
					// coupling
					idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
					cdn_variable_set_value(d_cdn_couplings[idx][0],0);
					cdn_variable_set_value(d_cdn_couplings[idx][1],0);
					cout << "coupling lesion: " << idx << endl;
					lesion_cpl[idx] = true;
							
					break;

				// combined model 
				case 1:
					
					// oscillations
					idx = segment_list[i];
					cdn_variable_set_value(d_cdn_freq[idx],0);
					cout << "oscillator lesion: " << idx << endl;
					lesion_osc[idx] = true;
					
					// coupling
					idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
					cdn_variable_set_value(d_cdn_couplings[idx][0],0);
					cdn_variable_set_value(d_cdn_couplings[idx][1],0);
					cout << "coupling lesion: " << idx << endl;
					lesion_cpl[idx] = true;
					
					// feedback
					idx = segment_list3[i];
					ec_fb_gain[idx]=0;
					cout << "feedback lesion: " << idx << endl;
					lesion_fbk[idx] = true;
							
					break;

				// decoupled model
				case 2:
					
					// oscillations
					idx = segment_list[i];
					cdn_variable_set_value(d_cdn_freq[idx],0);
					cout << "oscillator lesion: " << idx << endl;
					lesion_osc[idx] = true;
					
					// feedback
					idx = segment_list3[i];
					ec_fb_gain[idx]=0;
					cout << "feedback lesion: " << idx << endl;
					lesion_fbk[idx] = true;
							
					break;

				// sensory-driven model
				case 3:
					
					// coupling
					idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
					cdn_variable_set_value(d_cdn_couplings[idx][0],0);
					cdn_variable_set_value(d_cdn_couplings[idx][1],0);
					cout << "coupling lesion: " << idx << endl;
					lesion_cpl[idx] = true;
					
					// feedback
					idx = segment_list3[i];
					ec_fb_gain[idx]=0;
					cout << "feedback lesion: " << idx << endl;
					lesion_fbk[idx] = true;
							
					break;
			}
		}
	}
	else if (lesion_mode==7)
	{
		// combined lesions: all types at the same time but at different random locations
		// no feedback lesions (only oscillator and coupling)
		int segment_list2[N_SERVOS], segment_list3[N_SERVOS];
		for(int i=0; i<N_SERVOS; i++)
		{
			segment_list2[i] = segment_list[i];
			segment_list3[i] = segment_list[i];
		}
		random_shuffle(segment_list2, segment_list2 + N_SERVOS);
		random_shuffle(segment_list3, segment_list3 + N_SERVOS);

		// combined lesion (oscillation, coupling and feedback)
		cout << "combined lesion (oscillator/coupling types together but at diff locs):" << endl;
		for(int i=0; i<nLocations; i++)
		{
			switch(control_mode)
			{
				// open loop model
				case 0:
					// oscillations
					idx = segment_list[i];
					cdn_variable_set_value(d_cdn_freq[idx],0);
					cout << "oscillator lesion: " << idx << endl;
					lesion_osc[idx] = true;
					
					// coupling
					idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
					cdn_variable_set_value(d_cdn_couplings[idx][0],0);
					cdn_variable_set_value(d_cdn_couplings[idx][1],0);
					cout << "coupling lesion: " << idx << endl;
					lesion_cpl[idx] = true;
							
					break;

				// combined model 
				case 1:
					
					// oscillations
					idx = segment_list[i];
					cdn_variable_set_value(d_cdn_freq[idx],0);
					cout << "oscillator lesion: " << idx << endl;
					lesion_osc[idx] = true;
					
					// coupling
					idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
					cdn_variable_set_value(d_cdn_couplings[idx][0],0);
					cdn_variable_set_value(d_cdn_couplings[idx][1],0);
					cout << "coupling lesion: " << idx << endl;
					lesion_cpl[idx] = true;
							
					break;

				// decoupled model
				case 2:
					
					// oscillations
					idx = segment_list[i];
					cdn_variable_set_value(d_cdn_freq[idx],0);
					cout << "oscillator lesion: " << idx << endl;
					lesion_osc[idx] = true;
	
					break;

				// sensory-driven model
				case 3:
					
					// coupling
					idx = (int) round(( (float) segment_list2[i]/N_SERVOS)*(N_SERVOS-1));
					cdn_variable_set_value(d_cdn_couplings[idx][0],0);
					cdn_variable_set_value(d_cdn_couplings[idx][1],0);
					cout << "coupling lesion: " << idx << endl;
					lesion_cpl[idx] = true;
							
					break;
			}
		}
	}

}

void
AnguilliformRobot :: initMode()
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
		if(cpg_model_type==0){
			for(int i=0; i<N_SERVOS-1; i++)
			{
				cdn_variable_set_value(d_cdn_couplings[i][0],0);
				cdn_variable_set_value(d_cdn_couplings[i][1],0);
			}
		}
		else if(cpg_model_type==1){
			cdn_variable_set_value(d_cdn_eta_i,0);
			cdn_variable_set_value(d_cdn_eta_e,0);
		}
		else{

		}
	}
	// sensory-driven
	else if (control_mode==3)
	{
		if(cpg_model_type==0){
			for(int i=0; i<N_SERVOS; i++)
			{
				cdn_variable_set_value(d_cdn_freq[i],0);
			}
		}
		else if(cpg_model_type==1){
			cdn_variable_set_value(d_cdn_eta,0);
		}
		else{

		}
	}
	// sensory-driven with head oscillator
	else if (control_mode==4)
	{
		for(int i=1; i<N_SERVOS; i++)
		{
			cdn_variable_set_value(d_cdn_freq[i],0);
		}
	}
	else if (control_mode==5)
	{
		for(int i=0; i<N_SERVOS; i++)
		{
			cdn_variable_set_value(d_cdn_freq[i],0);
		}
	}
}


void
AnguilliformRobot :: initLogging()
{

	if(LOG==2)
	{
		// alternative logging
		char suffix[50] = "";
		char filename[100];
		for(uint j=0;j<n_opti_params;j++)
		{
			sprintf(suffix,"%s_%.2f",suffix,opti_params[j]);
		}
		if(opti_flag==1){
			sprintf(filename,"/data/thandiac/test/log_all%s.txt",suffix);
			// sprintf(filename,"log_all%s.txt",suffix);
		}
		else{
			sprintf(filename,"log_all%s.txt",suffix);
		}
		log_all = fopen(filename,"w");

		// write header
		for (uint i = 0; i < n_opti_params; i++)
		{
			// (i<n_opti_params-1) ? fprintf(log_all,"%s\t",opti_params_names[i]): fprintf(log_all,"%s\n",opti_params_names[i]);
			fprintf(log_all,"%s\t",opti_params_names[i]);
		}
		for (uint i = 0; i < N_SERVOS; i++){
			fprintf(log_all, "LO%d\t", i); // oscillation lesions
		}
		for (uint i = 0; i < N_SERVOS-1; i++){
			fprintf(log_all, "LC%d\t", i); // coupling lesions
		}
		for (uint i = 0; i < N_SERVOS; i++){
			fprintf(log_all, "LF%d\t", i); // feedback lesions
		}
		for (uint i = 0; i < N_SERVOS; i++){
			fprintf(log_all, "LM%d\t", i); // motoneuron lesions
		}
		fprintf(log_all,"\n");

		for (uint i = 0; i < n_opti_params; i++)
		{
			// (i<n_opti_params-1) ? fprintf(log_all,"%.3f\t",opti_params[i]): fprintf(log_all,"%.3f\n",opti_params[i]);
			fprintf(log_all,"%.3f\t",opti_params[i]);
		}

		for (uint i = 0; i < N_SERVOS; i++){
			fprintf(log_all, "%d\t", lesion_osc[i]); // oscillation lesions
		}
		for (uint i = 0; i < N_SERVOS-1; i++){
			fprintf(log_all, "%d\t", lesion_cpl[i]); // coupling lesions
		}
		for (uint i = 0; i < N_SERVOS; i++){
			fprintf(log_all, "%d\t", lesion_fbk[i]); // feedback lesions
		}
		for (uint i = 0; i < N_SERVOS; i++){
			fprintf(log_all, "%d\t", lesion_mtn[i]); // motoneuron lesions
		}
		fprintf(log_all,"\n");

		// time
		fprintf(log_all,"time\t");
		// external normal forces
		for (uint i = 0; i < N_SERVOS_TOT+1; i++)
		{
			fprintf(log_all,"F%d\t",i);  	
		}
		// torque patterns
		for (uint i = 0; i < N_SERVOS; i++)
		{
			fprintf(log_all,"T%d\t",i);
		}
		// muscle activation patterns
		for (uint i = 0; i < N_SERVOS; i++)
		{
			fprintf(log_all,"A%d\t",i);  	
		}
		// joint angles 
		for (uint i = 0; i < N_SERVOS_TOT; i++)
		{
			fprintf(log_all,"J%d\t",i);  	
		}
		if(cpg_model_type==0){
			// oscillator: theta
			for (uint i = 0; i < N_SERVOS; i++)
			{
				fprintf(log_all,"Theta%d\t",i);
			}
		}
		else if(cpg_model_type==1){
			for (uint i = 0; i < N_SERVOS; i++)
			{
				fprintf(log_all,"MatsuA%d\t",i);
			}
			for (uint i = 0; i < N_SERVOS; i++)
			{
				fprintf(log_all,"MatsuB%d\t",i);
			}
		}
		// head position
		fprintf(log_all,"posx\tposy\tposz\t");
		// head orientation
		fprintf(log_all,"orientx\torienty\torientz\t");
		// 3D external forces
		for (uint i =0; i < N_SERVOS_TOT+1; i++)
		{
			for (uint j=0; j < 3; j++)
			{
				fprintf(log_all,"Fext%d%d\t",i,j);
			}		
		}
		// next line
		fprintf(log_all,"\n");
	}
	else
	{
		if(LOG==1)
		{
			// open log files
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

			// set precision
			log_time.precision(3);
			log_ext_forces.precision(3);
			log_norm_forces.precision(3);
			log_torques.precision(3);
			log_act.precision(3);
			log_joint_ang.precision(3);
			log_theta.precision(5);
			// log_body_pos.precision(3);
			// log_body_orient.precision(3);
			// log_speed.precision(3);		
		}
	}

}

void
AnguilliformRobot :: readFile()
{
	ifstream movie_list("movie_list.txt");
	string data;
	int k = 0;
	int val_int;
	double val_double;

	if(movie_list.is_open())
	{

		while(getline(movie_list,data))
		{
			istringstream line(data);
			line >> val_double;
			mov_list_freq.push_back(val_double);
			line >> val_int; 
			mov_list_cntrl_mode.push_back(val_int); 
			line >> val_int; 
			mov_list_les_mode.push_back(val_int); 
			line >> val_int; 
			mov_list_n_les_loc.push_back(val_int); 
			line >> val_int; 
			mov_list_rand_seed.push_back(val_int);
			k++;
		}
		mov_list_num_settings = k;
	}
	movie_list.close();

	frequency = mov_list_freq[0];
	control_mode = mov_list_cntrl_mode[0];
	lesion_mode = mov_list_les_mode[0];
	n_lesion_locations = mov_list_n_les_loc[0];
	rand_fixed_seed_value = mov_list_rand_seed[0];
	 
}

void 
AnguilliformRobot :: readForceCouplingMatrixFromFile()
{

	ifstream fmat_file("force_coupling_matrix.txt");
	string data;
	double val_double;
	int k = 0;
	int n_lines=0;

	cout << "force coupling matrix: osc x force_sensors " << endl;

	//vector<double> vec;

	if(fmat_file.is_open())
	{
		while(getline(fmat_file,data))
		{
			istringstream line(data);
			
			for(int i=0; i<N_SERVOS+1; i++)
			{
				line>>val_double;
				//vec.push_back(val_double);
				//cout << val_double << " ";
				fmat[k][i] = val_double;
			}
			//cout << endl;
			//fmat.push_back(vec);
			//vec.clear();
			k++;
		}
	}
	fmat_file.close();
	n_lines = k;

	cout << "number of rows: " << k << endl;

	for(int j=0;j<n_lines;j++)
	{
		for(int i=0;i<N_SERVOS+1;i++)
		{
			cout << fmat[j][i] << " ";
		}
		cout << endl;
	}

}

void
AnguilliformRobot :: startMovieRec()
{

	// create filename from settings
	char filename[200];
	sprintf(filename, "fdb_study_%1.1f_%d_%d_%d_%d.avi",frequency, control_mode, lesion_mode, n_lesion_locations,d_cdn_seed);

	// start movie
	startMovie(filename, 1920, 540, 0, 100);
} 