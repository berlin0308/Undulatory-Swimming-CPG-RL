#ifndef __HYDRO_H
#define __HYDRO_H

#include <misc/matrix/matrix.h>

/* macros to access linear and angular parts of a Vector6 */
#define LINEAR(X)  ((Vector) &(X)[0])
#define ANGULAR(X) ((Vector) &(X)[3])

typedef struct
{
	double rho_fluid;

	double cf;
	double cd;
	double cm;
	double cp;
} HydroModel;

typedef struct
{
	HydroModel *hydro;
	
	double length;
	double width;
	double height;
	double origin_offset;		/* distance between axis and edge */

	Vector6 velocity;

	Matrix3 mf;
	Matrix3 If;
	Matrix6 IIf;				/* [ mf 0; 0 If ] */

	Matrix6 Iadd;
	/* Vector6 Fext; */

	int index;
} HydroSegment;

void hydro_segment_calculate_added_mass (HydroSegment *segment);
void hydro_segment_get_segment_force (HydroSegment *segment, Vector6 segment_force);
void hydro_segment_get_caudal_force (HydroSegment *segment, Vector6 caudal_force);
void hydro_segment_get_head_force (HydroSegment *segment, Vector6 head_force);

#endif
