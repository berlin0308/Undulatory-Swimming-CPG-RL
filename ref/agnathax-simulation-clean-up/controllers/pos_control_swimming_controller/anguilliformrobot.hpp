// File:          	webots_undulatory_swimming.cpp
// Date:          	23-11-2015
// Description:		controller to explore undulatory swimming with CPG / proprioceptive and exterceptive feedback
// Author:        	Robin Thandiackal, Florin Dzeladini
// Modifications: 	-

#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Servo.hpp>
#include <webots/Node.hpp>

#include <cmath>
#include <iostream>
#include <fstream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <numeric> 		// std::accumulate
#include <algorithm>	// std::max_element
#include <codyn/codyn.h>

#include <optimization/webots.hh>
#include "../../MyLib/optimizationhandler.hpp"
#include "../../MyLib/config.hpp"

using namespace std;

enum{X,Y,Z};


// enum {KEYBOARD_END, KEYBOARD_HOME, KEYBOARD_LEFT,
// KEYBOARD_UP, KEYBOARD_RIGHT, KEYBOARD_DOWN,
// KEYBOARD_PAGEUP, KEYBOARD_PAGEDOWN,
// KEYBOARD_NUMPAD_HOME, KEYBOARD_NUMPAD_LEFT,
// KEYBOARD_NUMPAD_UP, KEYBOARD_NUMPAD_RIGHT,
// KEYBOARD_NUMPAD_DOWN, KEYBOARD_NUMPAD_END,
// KEYBOARD_KEY, KEYBOARD_SHIFT, KEYBOARD_CONTROL,
// KEYBOARD_ALT};

class AnguilliformRobot : public webots::Supervisor 
{
	// webots related variables
	webots::Servo *servo[N_SERVOS_MAX];
	webots::GPS *gps1, *gps2;
	webots::Emitter *emitter;
	webots::Receiver *receiver;
	webots::Node *bodies[N_SERVOS_MAX];
	webots::Node *head_body;
	
	// codyn related variables
	CdnNetwork *d_cdn_network;
	CdnVariable *d_cdn_theta[N_SERVOS_MAX], *d_cdn_theta_dot[N_SERVOS_MAX], *d_cdn_r[N_SERVOS_MAX], *d_cdn_x[N_SERVOS_MAX], *d_cdn_sf[N_SERVOS_MAX], *d_cdn_ef[N_SERVOS_MAX], *d_cdn_kappa[N_SERVOS_MAX];
	CdnVariable *d_cdn_init_freq, *d_cdn_freq[N_SERVOS_MAX], *d_cdn_phlag, *d_cdn_wup, *d_cdn_wdown, *d_cdn_wupcut, *d_cdn_wdowncut;
	CdnVariable *d_cdn_couplings[N_SERVOS_MAX][2];
	int d_cdn_seed;

	// modes
	// control mode
	int control_mode_enable, control_mode;
	// lesion mode
	int lesion_mode_enable, lesion_mode, n_lesion_locations;

	// muscle parameters
	double parameter[N_MUSCLE_PARAMS];

	// network parameters
	double frequency;

	// physics-controller communication
	packet_physics_to_controller *receiverPacket;

	// fb gain
	double ec_fb_gain[N_SERVOS_MAX]; 

	// speed metric
	double maxSimTime;
	vector <double> posX;
	vector <double> posZ; 	// webots coordinate system
	vector <double> speed;
	vector <double> meanSpeed, stdSpeed;
	double instSpeed;
	double meanSpeedVal, stdSpeedVal, meanStdSpeed, stdStdSpeed;

	// initial condition
	double initTheta[N_SERVOS_MAX];

	// amplitude estimation	
	vector <double> end_cycle_ang[N_SERVOS_MAX];
	double ampl[N_SERVOS_MAX];

	// energy estimation
	double absEnergy, posEnergy, negEnergy;

	// frequency estimation
	vector <double> phaseDerivative[N_SERVOS_MAX];
	double avgPhaseDerivative[N_SERVOS_MAX];
	double avgFrequency;

	// periodicity
	double zcStd[N_SERVOS_MAX]; // zero crossings standard deviation (as fraction of mean difference between zero crossings)
	double avgZCStd; // average zero crossings standard deviation

	// straightness estimation
	vector <double> headPosX;
	vector <double> headPosZ;
	double straigthnessMeas;

	// phase lag estimation
	vector <double> phaseLag[N_SERVOS_MAX];
	double avgIndividualPhaseLag[N_SERVOS_MAX];
	double avgPhaseLag; 
	double phaseLagAct[N_SERVOS_MAX];	
	double phaseLagKin[N_SERVOS_MAX];
	double avgPhaseLagAct;	
	double avgPhaseLagKin;

	// helper vectors for estimations
	vector <double> jointAngles[N_SERVOS_MAX]; // for freq and kin phase lag
	vector <double> jointActivs[N_SERVOS_MAX]; // for neural phase lag

	// disturbance
	double noise_level; 

	// simulation ID (fake parameter to run same simulation multiple times)
	int simuID;

	// logging
	ofstream 	log_time, log_ext_forces, log_norm_forces, 
				log_torques, log_act, log_joint_ang, log_theta, 
				log_body_pos, log_body_orient, log_speed, 
				log_peaks, log_periods, log_amplitude;

	double t;
	double output[N_SERVOS_MAX], external_force[N_SERVOS_MAX][3], external_normal_force[N_SERVOS_MAX];
	double torques[N_SERVOS_MAX], joint_angle[N_SERVOS_MAX], cpg_theta[N_SERVOS_MAX];

	bool opti_flag;

	// optimization / systematic search related variables
	optihandler *optihand;
	map<string,double> fitness;

	// functions
	public:
	AnguilliformRobot();
	void run();

	private:
	void projection(double [3], double [3], double (&));
	bool hasJoint(uint i);
	string getJointNameLowerCase(uint i);
	string getJointNameUpperCase(uint i);
	string getJointPhysicsNameUpperCase(uint i);
	void updateStructuralParameters();
	void initBodies();
	void initServos();
	void initCPG();
	void initGPS();
	void initOptimizationParameters();
	void instantenousSpeed(double, double);
	void amplitudes();
	bool averageFrequency();
	void straightness();
	void averagePhaseLag();
	bool individualPhaseLags();
	void initialCondition();
	double firstOrderFilter(double, double, double, double);
	void meanStdVector(vector<double>, double &, double &);
	void returnFitness();

	void modeInit();
	void lesionInit(int);


	void logging();
	void closeLogFiles();
	//void stiffnessGradient();

};
