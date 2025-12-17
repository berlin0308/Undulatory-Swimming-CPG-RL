#ifndef __HYDRO_BOXES_H
#define __HYDRO_BOXES_H

#include "../../../MyLib/config.hpp"
#include <misc/matrix/matrix.h>
#include <string>
#include <array>

class Box
{
public:
	Box( ){

	}

	std::string solid_def;
	Vector3 size;          /* in box hydro frame */
	Vector3 translation;   /* from solid center of mass to box center, in solid frame */
	Quaternion rotation;   /* of box hydro frame, in solid frame */
};


//#define NUM_BOXES 19

extern std::array<Box*,N_SERVOS_MAX+1> boxes;

void init_boxes(uint n_servos, uint n_servos_tail);

#endif

