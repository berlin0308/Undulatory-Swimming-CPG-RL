#include "hydro_boxes.h"
#include "../../../MyLib/config.hpp"
#include <sstream>
#include <iostream>

using namespace std;


unsigned int NUM_BOXES=0;
array<Box*,N_SERVOS_MAX+1> boxes;

//Box boxes[NUM_BOXES];
	 /*
	 = {
	{"PHYSICS_ROBOT",
	 {0.0725, 0.045, 0.059},
	 {0, 0.01475, 0.01315},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_1",
	 {0.0725, 0.045, 0.059},
	 {0, 0.01475, 0.01315},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_2",
	 {0.0725, 0.045, 0.059},
	 {0, 0.01475, 0.01315},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_3",
	 {0.0725, 0.045, 0.059},
	 {0, 0.01475, 0.01315},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_4",
	 {0.0725, 0.045, 0.059},
	 {0, 0.01475, 0.01315},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_5",
	 {0.0725, 0.045, 0.059},
	 {0, 0.01475, 0.01315},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_6",
	 {0.0725, 0.045, 0.059},
	 {0, 0.01475, 0.01315},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_7",
	 {0.0725, 0.045, 0.059},
	 {0, 0.01475, 0.01315},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_8",
	 {0.0725, 0.045, 0.059},
	 {0, 0.01475, 0.01315},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_9",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_10",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_11",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_12",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_13",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_14",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_15",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_16",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_17",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}},

	{"PHYSICS_JOINT_SERVO_18",
	 {0.0249, 0.001, 0.059},
	 {0, 0.01475, 0.00055},
	 {-0.5, 0.5, 0.5, 0.5}}};
	*/

void init_boxes(uint n_servos, uint n_servos_tail){

	uint n_servos_head=__NUM_HEAD_ELEMENTS__;
	boxes[0] = new Box();

	double 	body_length = __SEGMENT_LENGTH__,
			body_cylinder_radius = __SEGMENT_RADIUS__,
			body_axis_x_shift = __SEGMENT_RADIUS__+0.021,//+0.00125,//0.021,
			body_width = __SEGMENT_WIDTH__,
			body_height = __SEGMENT_HEIGHT__,
			body_center_y = __SEGMENT_Y_COM__,
			body_center_z = __SEGMENT_Z_COM__;

	double 	tail_length = __TAIL_SEGMENT_LENGTH__,
			tail_width = __TAIL_SEGMENT_WIDTH__,//0.001,
			tail_height = body_height,
			// tail_shift = 0.00055;
			tail_center_y = __TAIL_SEGMENT_Y_COM__,
			tail_center_z = __TAIL_SEGMENT_Z_COM__;
//Anouk: added parameters for head
	double 	head_length = __HEAD_LENGTH__,
			head_cylinder_radius = __HEAD_RADIUS__,
			// head_axis_x_shift = __HEAD_RADIUS__+0.00125,//0.021,
			head_width = __HEAD_WIDTH__,
			head_height = __HEAD_HEIGHT__,
			head_center_y = __HEAD_SEGMENT_Y_COM__,
			head_center_z = __HEAD_SEGMENT_Z_COM__;



	// first element. Anouk:adjusted body to head
	boxes[0]->solid_def = "PHYSICS_ROBOT";
	boxes[0]->size[0] =  head_length;//0.0725;	// 
	boxes[0]->size[1] =  head_width;//0.045; 	// width
	boxes[0]->size[2] =  head_height;//0.059;
	boxes[0]->translation[0] = 0;
	boxes[0]->translation[1] = -head_center_y;
	boxes[0]->translation[2] = -head_center_z+head_length/2;//-head_axis_x_shift+(head_length-head_cylinder_radius)/2+head_cylinder_radius-head_axis_x_shift/2; //0.028//0.01315;  -0.021+0.093/2+0.0025
	boxes[0]->rotation[0] =  -0.5;
	boxes[0]->rotation[1] =  0.5;
	boxes[0]->rotation[2] =  0.5;
	boxes[0]->rotation[3] =  0.5;
	
	// head elements. Anouk: added next section
	for(uint i = 1; i<=n_servos_head; i++){
		stringstream ss;
		ss << "PHYSICS_JOINT_SERVO_" << i;
		
		boxes[i] = new Box();
		boxes[i]->solid_def = string(ss.str());
		boxes[i]->size[0] =  head_length;//0.0725;	// 
		boxes[i]->size[1] =  head_width;//0.045; 	// width
		boxes[i]->size[2] =  head_height;//0.059;

		boxes[i]->translation[0] = 0;
		boxes[i]->translation[1] = -head_center_y;
		boxes[i]->translation[2] = -head_center_z+head_length/2;//-head_axis_x_shift+(head_length-head_cylinder_radius)/2+head_cylinder_radius-head_axis_x_shift; //0.028//0.01315;  -0.021+0.093/2+0.0025


		boxes[i]->rotation[0] =  -0.5;
		boxes[i]->rotation[1] =  0.5;
		boxes[i]->rotation[2] =  0.5;
		boxes[i]->rotation[3] =  0.5;
	}

	// body elements. Anouk: changed first line, plus n_servos_head)
	for(uint i = n_servos_head+1; i<=n_servos; i++){
		stringstream ss;
		ss << "PHYSICS_JOINT_SERVO_" << i;
		
		boxes[i] = new Box();
		boxes[i]->solid_def = string(ss.str());
		boxes[i]->size[0] =  body_length;//0.0725;	// 
		boxes[i]->size[1] =  body_width;//0.045; 	// width
		boxes[i]->size[2] =  body_height;//0.059;

		boxes[i]->translation[0] = 0;
		boxes[i]->translation[1] = -body_center_y;;
		// boxes[i]->translation[2] = -body_axis_x_shift+(body_length-body_cylinder_radius)/2+body_cylinder_radius-body_axis_x_shift; //0.028//0.01315;  -0.021+0.093/2+0.0025
		boxes[i]->translation[2] = -body_center_z+body_length/2;//-body_axis_x_shift+(body_length-body_cylinder_radius)/2+body_cylinder_radius-body_axis_x_shift; 

		boxes[i]->rotation[0] =  -0.5;
		boxes[i]->rotation[1] =  0.5;
		boxes[i]->rotation[2] =  0.5;
		boxes[i]->rotation[3] =  0.5;
	}
	// tail elements. Anouk: changed first line, plus n_servos_head)
	for(uint i = n_servos+1; i<= n_servos+1+n_servos_tail; i++){
		stringstream ss;
		ss << "PHYSICS_JOINT_SERVO_" << i;
		
		boxes[i] = new Box();
		boxes[i]->solid_def = string(ss.str());
		boxes[i]->size[0] =  tail_length;//0.0249;
		boxes[i]->size[1] =  -(body_width-0.001)/(n_servos_tail*tail_length)*(i-n_servos)*tail_length+body_width; //tail_width;//0.001;
		boxes[i]->size[2] =  tail_height;//0.059;

		boxes[i]->translation[0] = 0;
		boxes[i]->translation[1] = -tail_center_y;
		boxes[i]->translation[2] = 0.0;//0.00055;

		boxes[i]->rotation[0] =  -0.5;
		boxes[i]->rotation[1] =  0.5;
		boxes[i]->rotation[2] =  0.5;
		boxes[i]->rotation[3] =  0.5;
	}
	

}

