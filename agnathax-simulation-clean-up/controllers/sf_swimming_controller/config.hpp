#ifndef __CONFIG_HPP
#define __CONFIG_HPP

// general
#define TIME_STEP		5//1		// [ms]
#define MAX_TIME 		10//40		// [s]

// robot parameters
#define N_SERVOS_MAX	30
#define N_SERVOS_TAIL	4//4
extern unsigned int N_SERVOS 	;		// number of modules without fin
extern unsigned int N_SERVOS_TOT;	// number of all joint elements (including the fin)
#define	AMP 			0.8 	// [Nm] torque

// network parameters
#define FREQ 		1.0//1.5					// [Hz]
extern float PHI_LAG;	// [rad]		
extern float PHI_LAG_PERCENTAGE;// [%]// [rad]
#define DEFAULT_PHI_LAG_PERCENTAGE  100
#define W_UP		10					// network weight (up - towards head)
#define W_DOWN		10					// network weight (down - towards tail)

// muscle parameters
#define N_MUSCLE_PARAMS	3
#define ALPHA			1.0//1.0//1.0	// activation gain
#define GAMMA			2.0//2.0 //0.1//0.8	// stiffness
#define DELTA			0.1 //.05//0.1	// damping

// feedback parameters
#define PC_FB_GAIN		0//10.0	// proprioceptive feedback
#define EC_FB_GAIN 		100.0//100//20//100.0 	// exteroceptive feedback

// feedback decay
#define DECAY_ENABLE 		0 // 0: decay disabled, 1: decay enabled
#define DECAY_START_TIME 	20 // first DECAY_START_TIME seconds EC_FB_GAIN is set 
#define DECAY_DURATION  	40 // duration for decay to occur, at DECAY_START_TIME+DECAY_DURATION feedback is turned off
#define DECAY_PARAM 		0.5 // exponential decay coefficient EC_FB_GAIN*exp(-DECAY_PARAM*(t-DECAY_START_TIME)

// tail parameters
#define P_STIFFNESS 	2.0//2.0//2.0//2.0 // related to tail 
#define P_DAMPING		0.0//0.0 // related to tail 

// motor actuation mode options
#define CNTRL 				0 		// 0: torque control / 1: position control

// pos control settings
#define POS_AMPLITUDE		30.0	// position control amplitude in [deg]
#define POS_GRADIENT_ENABLE	1		// 0: no gradient / 1: with gradient
#define POS_GRADIENT_START	10 		// head amplitude in [deg] , tail amp is POS_AMPLITUDE

// resonance mode control settings
#define RES_TIME_UNTIL_RELEASE	2//10	// time until activation is set back to zero
#define RES_ACTIVATION_LEVEL	0.5	// activation that determines initial bending 
#define RES_GRADIENT_ENABLE		1 	// 0: no gradient / 1: with gradient
#define RES_GRADIENT_START 		0.0 // head activation 

// twitching mode control settings
#define TWITCH_DURATION  		0.5 		// twitching duration in seconds
#define TWITCH_ACTIVATION_LEVEL 1.0 	// activation level of one joint twitching
#define TWITCH_JOINT 	 		9 		// id of joint to twitch

// CPG model type
#define CPG_MODEL_TYPE		0	// 0: phase oscillators / 1: matsuoka oscillators

// logging
#define LOG 				2 		// 1: logging to text file / 2: logging to single text file with opti parameters as name

// optimization / systematic search
#define OPTI_TYPE 		0 	// 0: lesion sys search / 1: pos cntrl sys search / 2: resonance experiments
#define N_OPT_PARAMS	20	// number of variables
#define N_OPT_SETTINGS	20	// number of settings
static const char * PHASE_NAME_PREFIX = "phiL";

// performance metrics
#define SPEED_TIME_CONSTANT 	0.1		// filtering time constant for speed estimation
#define N_SPEED_MEASURE_CYCLES 	2 		// number of cycles over which speed is estimated
// #define N_ENERGY_MEASURE_CYCLES N_SPEED_MEASURE_CYCLES // number of cycles over which energy is estimated
// #define STEADY_STATE_THRESH		0.01	//0.003	// normalized threshold criterion (mean std speed / mean speed )
// #define WINDOW_SIZE				20//10

#define T_FREQ_EST_DURATION		5 // [s] corresponds to slowest detactable freq of 1/T_FREQ_EST_DURATION
#define T_FREQ_EST_START		MAX_TIME-2*T_FREQ_EST_DURATION // [s]

// initialization
#define RAND_FIXED_SEED			1//1		// 1: fixed rand seed (-> RAND_FIXED_VALUE) , 0: rand value 
#define RAND_FIXED_SEED_VALUE	28244//13437//80421//73354//2078	// fixed rand seed

// disturbance
#define DEFAULT_NOISE_LEVEL 	0	// noise level in percentage for signal dependant noise

// control mode config
#define CONTROL_MODE_ENABLE		1
#define CONTROL_MODE 			2	// 0: open loop / 1: combined / 2: decoupled / 3: sensory-driven 
									// 4: sensory-driven with head osc / 5: resonance test / 6: turning / 7: twitching

// lesion study
#define LESION_MODE_ENABLE	0	// 0: lesions off / 1: lesions on
#define LESION_MODE_TYPE 	0 	// 0: random lesion types / 1: definition with lesion comb id 
#define LESION_MODE 		0	// knock-off 0: oscillations / 1: coupling / 2: feedback / 3: motoneuron / 
								// 4: combined lesions (random types and locations) / 
								// 5: combined lesions (all types in same random locations, nLesTot = nLes x 3) / 
								// 6: combined lesions (all types at different random locations, nLesTot = nLes x 3) / 
								// 7: same as 6 but without feedback lesions 
#define DEFAULT_N_LESIONS	0 	// number of lesions (lesion_mode_type: 0)
#define SECTION_START_IND	0	// lesion start index (starts at 0)
#define SECTION_END_IND		9	// lesion end index (careful for coupling lesions)
#define LESION_COMB_ID		7 	// id of lesion combination within section start and end (careful for coupling lesions)

#define REC_MOVIE	0 // record movie -> ATTENTION uses 'movie_list.txt' to set parameters!

#include <vector>

typedef struct
{
	float actv[N_SERVOS_MAX][2];
	float pmtr[N_MUSCLE_PARAMS];
	bool cntrl; // torque: 0 , position: 1
}packet_controller_to_physics;

typedef struct
{
	float torq[N_SERVOS_MAX];
	float forces[N_SERVOS_MAX+1][3];
	float speed[N_SERVOS_MAX];
}packet_physics_to_controller;


#endif
