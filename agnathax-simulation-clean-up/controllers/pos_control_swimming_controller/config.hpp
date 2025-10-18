#ifndef __CONFIG_HPP
#define __CONFIG_HPP

// general
#define TIME_STEP		5//1		// [ms]
#define MAX_TIME 		10//40		// [s]

// robot parameters
#define N_SERVOS_MAX	30
#define N_SERVOS_TAIL	4
extern unsigned int N_SERVOS 	;		// number of modules without fin
extern unsigned int N_SERVOS_TOT;	// number of all joint elements (including the fin)
#define	AMP 			0.8 	// [Nm] torque
#define POS_CNTRL_AMP 	20		// [degrees]

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
#define EC_FB_GAIN 		0//100//20//100.0 	// exteroceptive feedback

// tail parameters
#define P_STIFFNESS 	2.0//2.0//2.0//2.0 // related to tail 
#define P_DAMPING		0.0//0.0 // related to tail 

// options
#define CNTRL 		1 	// 0: torque control / 1: position control
#define LOG 		0 	// 1: logging to text file 

// optimization / systematic search
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
#define RAND_FIXED_SEED			0		// 1: fixed rand seed (-> RAND_FIXED_VALUE) , 0: rand value 
#define RAND_FIXED_SEED_VALUE	73354//80421//73354//2078	// fixed rand seed

// disturbance
#define DEFAULT_NOISE_LEVEL 	0	// noise level in percentage for signal dependant noise

// control mode config
#define CONTROL_MODE_ENABLE		1
#define CONTROL_MODE 			0	// 0: open loop / 1: combined / 2: decoupled / 3: sensory-driven 

// lesion study
#define LESION_MODE_ENABLE	0
#define LESION_MODE 		4	// knock-off 0: oscillations / 1: coupling / 2: feedback / 3: motoneuron / 4: combined lesions 
#define DEFAULT_N_LESIONS	1 

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