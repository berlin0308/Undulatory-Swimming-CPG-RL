#include <stdlib.h>
#include <plugins/physics.h>

#include "hydro_webots.h"
#include "hydro.h"
#include "../../../MyLib/config.hpp"
#include "archimede.h"

// #include <misc/log/log.h>
// #include <optimizer/optimizer.hh>
#include <GL/glext.h>

extern unsigned int NUM_BOXES;
/* added mass is not accurate (it should be different for X, Y, Z) but
 * the present approximation, which uses the value for Y in the hydro
 * frame (i.e. the value for lateral displacements) can help a lot with
 * stability when using a thin passive tail.  */
static int added_mass = 1;

/* Matrix for change of basis between Webots solid and hydro frames.
 * Each column is the new basis vector expressed in the old basis.
 * Note that inv(M) = Mt.
 * To transform matrix A from old to new base: A' = Mt A M.
 * To transform vector x from old to new bas: x' = Mt x. */
static Matrix3 hydro_to_webots[N_SERVOS_MAX+1];
static Matrix3 webots_to_hydro[N_SERVOS_MAX+1];

static dBodyID bodies[N_SERVOS_MAX+1];
static dJointID joint;
static HydroSegment segments[N_SERVOS_MAX+1];
static double archimede_force[N_SERVOS_MAX+1];
static Vector6 hydro_wrench[N_SERVOS_MAX+1];
static HydroModel hydro;
static dWorldID world;
static dSpaceID space;
static dJointGroupID contact_joint_group;

static double archimede_factor = DEFAULT_ARCHIMEDE_FACTOR;
static double water_level = DEFAULT_WATER_LEVEL;
static int enable_hydro = 1;

static int head_index;
static int tail_index;

static Vector3 colors[] = {{1, 0, 0},
                           {0, 1, 0},
                           {0, 0, 1},
                           {0, 1, 1},
                           {1, 0, 1},
                           {1, 1, 0},
                           {0, 0, 0},
                           {1, 1, 1},
                           {0, 0.5, 1}};

static int logging = 0;
// static Monitors *monitors;

// ******* static variables to store force values for exteroceptive feedback ****
static double extero_force[N_SERVOS_MAX+1][3];

enum {
	RED,
	GREEN,
	BLUE,
	CYAN,
	MAGENTA,
	YELLOW,
	BLACK,
	WHITE,
	WATER_COLOR
};

/* Normalized box corners. */
static const Vector3 vertices[BOX_VERTICES] = {{0, 0, 0},
                                               {1, 0, 0},
                                               {1, 1, 0},
                                               {0, 1, 0},
                                               {0, 0, 1},
                                               {1, 0, 1},
                                               {1, 1, 1},
                                               {0, 1, 1}};

/* Box faces, with 5th index for easy edge extraction.
 * Gives faces counter-clockwise as seen from outside the box. */
static const Face full_box_faces[BOX_FACES] = {{{3, 2, 1, 0, 3}, 4},
                                               {{0, 1, 5, 4, 0}, 4},
                                               {{4, 5, 6, 7, 4}, 4},
                                               {{7, 6, 2, 3, 7}, 4},
                                               {{1, 2, 6, 5, 1}, 4},
                                               {{3, 0, 4, 7, 3}, 4}};

/* Extra vertices are allocated for intersection points. */
static Vector3 box_vertices[N_SERVOS_MAX+1][MAX_VERTICES] = {{{0, }, }, };

static Face faces[N_SERVOS_MAX+1][MAX_FACES] = {{{{0,}, 0}, }, };

static Vector3 box_centroid[N_SERVOS_MAX+1] = {{0, }};

static void
draw_p1_p2 (Vector p1, Vector p2)
{
	glLineWidth(4);
	glBegin(GL_LINES);
	glVertex3f(p1[0], p1[1], p1[2]);
	glVertex3f(p2[0], p2[1], p2[2]);
	glEnd();
}

static void
draw_p1_v (Vector p1, Vector v)
{
	Vector3 p2;

	draw_p1_p2 (p1, vector3_sum(p1, v, p2));
}

static void
draw_arrow (Vector p1,
            Vector v,
            Vector z)
{
	Vector3 p2;
	
	vector3_sum (p1, v, p2);

	/* "horizontal" vector perpendicular to v */
	Vector3 axis;
	vector3_cross_vector (v, z, axis);

	/* if v and z are colinear, then z doesn't properly define the plane for the arrow head
	 * and we take an arbitrary plane instead*/
	if (vector3_norm (axis) < 0.00001)
	{
		Vector3 arbitrary = {-v[1], v[0], v[2]};
		vector_copy (axis, arbitrary, 3);
	}
	
	/* vector defining plane for arrow head */
	vector3_cross_vector (axis, v, axis);

	Vector3 head;
	vector3_multiply (v, -0.1, head);

	Quaternion rot;
	Vector3 head1, head2;
	quaternion_from_axis_angle (axis, M_PI / 4, rot);
	quaternion_rotation (rot, head, head1);

	quaternion_from_axis_angle (axis, -M_PI / 4, rot);
	quaternion_rotation (rot, head, head2);
	
	draw_p1_p2 (p1, p2);
	draw_p1_v (p2, head1);
	draw_p1_v (p2, head2);
}

static void
get_box_origin (int box_index, double *origin)
{
	dBodyGetRelPointPos (bodies[box_index],
	                     boxes[box_index]->translation[0],
	                     boxes[box_index]->translation[1],
	                     boxes[box_index]->translation[2],
	                     origin);
}

static void
draw_box_axes (int box_index, Vector3 z)
{
	if(!DRAW_SCALE_BOX_AXES)
		return;

	Vector3 origin;

	get_box_origin (box_index, origin);

	int i;
	for (i = 0; i < 3; ++i)
	{
		Vector3 axis = {0, 0, 0};

		axis[i] = boxes[box_index]->size[i] / 2;
		quaternion_rotation (boxes[box_index]->rotation, axis, axis);
		dBodyVectorToWorld (bodies[box_index], axis[0], axis[1], axis[2], axis);
		vector3_multiply (axis, DRAW_SCALE_BOX_AXES, axis);

		glColor3dv (colors[RED + i]);
		draw_arrow (origin, axis, z);
	}
}

static void
draw_face (Face *face, Vector3 v[])
{
	/* make sure the first index is repeated at the end */
	face->i[face->n] = face->i[0];

	int second;

	for (second = 1; second < face->n - 1; ++second)
	{
		glColor4d (colors[CYAN][0], colors[CYAN][1], colors[CYAN][2], 0.3);
		glBegin (GL_TRIANGLES);
		glVertex3dv (v[face->i[0]]);
		glVertex3dv (v[face->i[second]]);
		glVertex3dv (v[face->i[second + 1]]);
		glEnd ();

		glColor3dv (colors[BLACK]);
		draw_p1_p2 (v[face->i[second]], v[face->i[second + 1]]);
	}

	glColor3dv (colors[BLACK]);
	draw_p1_p2 (v[face->i[0]], v[face->i[1]]);
	draw_p1_p2 (v[face->i[face->n - 1]], v[face->i[0]]);
}


static void
draw_archimede_faces (int box_index)
{
	if (!DRAW_ARCHIMEDE_FACES)
	{
		return;
	}

	Vector3 *v = box_vertices[box_index];
	Face *f = faces[box_index];

	int i;

	for (i = 0; i < MAX_FACES; ++i)
	{
		if (f[i].n > 0)
		{
			draw_face (&f[i], v);
		}
	}
}

static void
draw_hydro_forces (int box_index, Vector3 z)
{
	Vector3 origin;
	get_box_origin (box_index, origin);

	Vector3 force;
	vector3_multiply (LINEAR (hydro_wrench[box_index]), DRAW_SCALE_HYDRO_FORCES, force);

	if( DRAW_SCALE_HYDRO_FORCES ){
		glColor3dv (colors[MAGENTA]);
		draw_arrow (origin, force, z);
	}
}

static void
draw_hydro_torques (int box_index, Vector3 z)
{
	Vector3 origin;
	get_box_origin (box_index, origin);

	Vector3 torque;
	vector3_multiply (ANGULAR (hydro_wrench[box_index]), DRAW_SCALE_HYDRO_TORQUES, torque);

	if( DRAW_SCALE_HYDRO_TORQUES ){
		glColor3dv (colors[CYAN]);
		draw_arrow (origin, torque, z);
	}
}

static void
draw_archimede_forces (int box_index, Vector3 z)
{
	Vector3 force = {0, archimede_force[box_index], 0};
	vector3_multiply (force, DRAW_SCALE_ARCHIMEDE_FORCES, force);

	if(DRAW_SCALE_ARCHIMEDE_FORCES){
		glColor3dv (colors[YELLOW]);
		/* glPointSize (5); */
		/* glBegin (GL_POINTS); */
		/* glVertex3dv (box_centroid[box_index]); */
		/* glEnd (); */
		draw_arrow (box_centroid[box_index], force, z);
	}
}

void
hydro_webots_draw ()
{
	if (!enable_hydro)
	{
		return;
	}

	glPushAttrib (GL_ENABLE_BIT | GL_LINE_WIDTH);
		
	glDisable (GL_LIGHTING);
	glDisable (GL_DEPTH_TEST);
	glClear (GL_DEPTH_BUFFER_BIT);

	int box_index;
	for (box_index = 0; box_index < NUM_BOXES; ++box_index)
	{
		if (bodies[box_index] == 0)
		{
			continue;
		}

		Vector3 z;
		quaternion_rotation (boxes[box_index]->rotation, z, z);
		dBodyVectorToWorld (bodies[box_index], z[0], z[1], z[2], z);

		/* glPushMatrix (); */
		/* glTranslated (0.5, 0, 0); */
		glLineWidth (1);
		draw_archimede_faces (box_index);
		/* glPopMatrix (); */

		glLineWidth (2);
		draw_archimede_forces (box_index, z);
		draw_box_axes (box_index, z);
		draw_hydro_forces (box_index, z);
		draw_hydro_torques (box_index, z);
	}

	glPopAttrib ();
}

/* initializes the segment structure, adds reactive term of the hydro
 * model to the body inertia tensor */
static void
box_init (int box_index)
{
	HydroSegment *segment = &segments[box_index];
	Matrix3 I = {{1, 0, 0},
	             {0, 1, 0},
	             {0, 0, 1}};

	/* rotation * e1 is the first column of webots_to_hydro, hence first row of hydro_to_webots */
	int i;
	for (i = 0; i < 3; ++i)
	{
		quaternion_rotation (boxes[box_index]->rotation, I[i], hydro_to_webots[box_index][i]);
	}

	matrix3_transpose (hydro_to_webots[box_index], webots_to_hydro[box_index]);
	
	/* Segment initialization */
	segment->index = box_index;
	segment->hydro = &hydro;
	segment->length = boxes[box_index]->size[0];
	segment->width = boxes[box_index]->size[1];
	segment->height = boxes[box_index]->size[2];
	segment->origin_offset = segment->length / 2.0; /* in hydro's frame of reference, x coordinate of the point for which linear velocity is given
	                                                 * i.e. x coordinate of the box center */
	
	/* Get added mass from hydrodynamical model in segment->Iadd */
	hydro_segment_calculate_added_mass (segment);

	if (added_mass)
	{
		dMass mass0, mass;
		Matrix3 Iadded, M;
		double added_mass;
		dBodyID body = bodies[box_index];

		/* Get current mass from ODE body */
		dBodyGetMass (body, &mass0);

		/* Extract inertia matrix from segment->Iadd into Iadded */
		/* Get actual mass in added_mass */
		matrix6_get_block3 (segment->Iadd, Iadded, 3, 3);
		matrix6_get_block3 (segment->Iadd, M, 0, 0);
		added_mass = segment->Iadd[1][1];

		/* Express Iadded in ODE's coordinate system */
		matrix3_product_matrix3 (webots_to_hydro[box_index], Iadded, Iadded);
		matrix3_product_matrix3 (Iadded, hydro_to_webots[box_index], Iadded);
	
		/* Prepare new mass structure with added mass */
		dMassSetParameters (&mass,
		                    added_mass,
		                    0, 0, 0, /* correct? */
		                    Iadded[0][0], Iadded[1][1], Iadded[2][2],
		                    Iadded[0][1], Iadded[0][2], Iadded[1][2]);

		/* Add it to the current mass */
		dMassAdd (&mass, &mass0);

		/* Set the new body mass */

		dBodySetMass (body, &mass);
	}
}

void
hydro_webots_init (dWorldID       w,
                   dSpaceID       s,
                   dJointGroupID  j,
                   char const    *head,
                   char const    *tail,
                   int            add_mass)
{
	int i;

	world = w;
	space = s;
	contact_joint_group = j;
	added_mass = add_mass;

	hydro.cf = 2;
	hydro.cd = 1.75;
	hydro.cm = 0.5;
	hydro.cp = 1;
	hydro.rho_fluid = WATER_DENSITY;

	head_index = -1;
	tail_index = -1;

	for (i = 0; i < NUM_BOXES; ++i)
	{
		bodies[i] = dWebotsGetBodyFromDEF (boxes[i]->solid_def.c_str());

		if (bodies[i])
		{
			box_init (i);

			/* first head box will get the head forces */
			if (head_index == -1 && head && !strcmp(boxes[i]->solid_def.c_str(), head))
			{
				head_index = i;
			}

			/* last tail box will get the tail forces */
			if (tail && !strcmp(boxes[i]->solid_def.c_str(), tail))
			{
				tail_index = i;
			}
		}
		else
		{
			dWebotsConsolePrintf ("Warning: couldn't find solid with DEF \"%s\"\n", boxes[i]->solid_def.c_str());
		}
	}
  

	if (head_index == -1)
	{
		head_index = 0;

		if (head)
		{
			dWebotsConsolePrintf ("Warning: couldn't find head solid with DEF \"%s\"\n", head);
		}
	}
	
	if (tail_index == -1)
	{
		tail_index = NUM_BOXES - 1;

		if (tail)
		{
			dWebotsConsolePrintf ("Warning: couldn't find tail solid with DEF \"%s\"\n", tail);
		}
	}

	//dWebotsConsolePrintf (enable_hydro ? "Hydrodynamics enabled\n" : "Hydrodynamics disabled\n", 0);
  
	// logging = optimizer_get_setting_bool ("logging", 1);
	logging = 0;

	if (logging)
	{
		// monitors = log_monitors_new ();
		// log_monitor_double_array (monitors, "log_hydro_wrench", 6 * NUM_BOXES, (double *) hydro_wrench);
	}
}

static double
box_add_archimede_force (HydroSegment *segment,
                         dBodyID       body,
                         int           box_index)
{
	double const *solid_position = dBodyGetPosition (body);
	double const *solid_rotation = dBodyGetQuaternion (body);

	Vector3 box_position;
	Quaternion box_rotation;

	quaternion_rotation ((double *) solid_rotation, boxes[box_index]->translation, box_position);
	vector3_sum ((double *) solid_position, box_position, box_position);

	quaternion_product_quaternion ((double *) solid_rotation, boxes[box_index]->rotation, box_rotation);

	/* Replace normalized vertex coordinates by real positions */
	int i;
	for (i = 0; i < BOX_VERTICES; ++i)
	{
		Vector bv = box_vertices[box_index][i];

		bv[X] = (vertices[i][X] - 0.5) * boxes[box_index]->size[X];
		bv[Y] = (vertices[i][Y] - 0.5) * boxes[box_index]->size[Y];
		bv[Z] = (vertices[i][Z] - 0.5) * boxes[box_index]->size[Z];

		quaternion_rotation (box_rotation, bv, bv);
		vector3_sum (bv, box_position, bv);
	}
	
	Vector centroid = box_centroid[box_index];
	vector_copy (centroid, box_position, 3);

	double full_volume = boxes[box_index]->size[X] * boxes[box_index]->size[Y] * boxes[box_index]->size[Z];

	double volume = archimede_volume (box_vertices[box_index],
	                                  full_box_faces,
	                                  faces[box_index],
	                                  centroid,
	                                  full_volume,
	                                  water_level);

	archimede_force[box_index] = GRAVITY * WATER_DENSITY * volume * archimede_factor;

	if (volume > 0)				/* volume is exactly 0 if the box is fully out of water */
	{
		dBodyAddForceAtPos (body, 0, archimede_force[box_index], 0, centroid[X], centroid[Y], centroid[Z]);
		/* dBodyAddForce (body, 0, archimede_force[box_index], 0); */
	}

	return volume / full_volume;
}

static void
box_add_hydro_forces (int box_index, double viscosity_scaling)
{
	dBodyID body = bodies[box_index];
	HydroSegment *segment = &segments[box_index];
	const double *linear_vel = dBodyGetLinearVel (body);
	const double *angular_vel = dBodyGetAngularVel (body);
	Vector6 segment_force, head_force, tail_force;

	segment->hydro->cf = viscosity_scaling*2.0;
	segment->hydro->cd = viscosity_scaling*1.75; 

	double volume_fraction = box_add_archimede_force (segment, body, box_index);
	
	dBodyVectorFromWorld (body, linear_vel[0], linear_vel[1], linear_vel[2], LINEAR (segment->velocity));
	dBodyVectorFromWorld (body, angular_vel[0], angular_vel[1], angular_vel[2], ANGULAR (segment->velocity));

	Vector3 box_vel;
	vector3_cross_vector (ANGULAR (segment->velocity), boxes[box_index]->translation, box_vel);
	vector3_sum (box_vel, LINEAR (segment->velocity), LINEAR (segment->velocity));

	matrix3_product_vector6 (hydro_to_webots[box_index], segment->velocity, segment->velocity);
	hydro_segment_get_segment_force (segment, segment_force);
	matrix3_product_vector6 (webots_to_hydro[box_index], segment_force, segment_force);

	if (box_index == head_index)
	{
		hydro_segment_get_head_force (segment, head_force);
		matrix3_product_vector6 (webots_to_hydro[box_index], head_force, head_force);
		vector6_sum (head_force, segment_force, segment_force);
	}

	if (box_index == tail_index)
	{
		hydro_segment_get_caudal_force (segment, tail_force);
		matrix3_product_vector6 (webots_to_hydro[box_index], tail_force, tail_force);
		vector6_sum (tail_force, segment_force, segment_force);
	}

	/* translate the wrench back to the center of mass */
	Vector3 box_to_com;
	vector3_multiply (boxes[box_index]->translation, -1, box_to_com);

	/* twist transformation matrix */
	Matrix6 A, AT;
	vector3_translation_transformation_matrix (box_to_com, AT);
	matrix6_transpose (AT, A);
	matrix6_product_vector (A, segment_force, segment_force);

	/* multiply by fraction of the box that is immersed, change sign */
	vector_multiply (segment_force, -volume_fraction, segment_force, 6);

	/* Put in global coordinates */
	double *wrench = hydro_wrench[box_index];
	dBodyVectorToWorld (bodies[box_index], segment_force[0], segment_force[1], segment_force[2], LINEAR (wrench));
	dBodyVectorToWorld (bodies[box_index], segment_force[3], segment_force[4], segment_force[5], ANGULAR (wrench));


	// // include also external force applied (e.g. interactively by user)
	// const dReal *val = dBodyGetForce(body);
	// for(int i=0; i<3; i++)
	// {
	// 	wrench[i] = wrench[i]+val[i];
	// }

	// // artificial forces
	// for(int i = 0; i<6; i++)
	// {
	// 	wrench[i] = 0.0;
	// }
	// if(box_index==2)
	// 	wrench[0] = -0.01;


	// add forces to the body
	dBodyAddForce (body, wrench[0], wrench[1], wrench[2]);
	dBodyAddTorque (body, wrench[3], wrench[4], wrench[5]);

	// store exteroceptive feedback in global variable
	extero_force[box_index][0] = wrench[0];
	extero_force[box_index][1] = wrench[1];
	extero_force[box_index][2] = wrench[2];

	if (added_mass)
	{
		/* compensate for the mass added by the hydrodynamics model */
		dBodyAddForce (body, 0, GRAVITY * segment->Iadd[1][1], 0);
	}
}

void
hydro_webots_step (float data[][3])
{
	int i=0,j=0;
	double viscosity_scaling=1;

	if (!enable_hydro)
	{
		return;
	}


	// // position for viscosity
	// double const *head_position =  dBodyGetPosition (bodies[0]);
	// if(head_position[2]<-4 && head_position[2]>-8)
	// {
	// 	viscosity_scaling = 3;
	// }
	// else
	// {
	// 	viscosity_scaling = 1;
	// }


	for (i = 0; i < NUM_BOXES; ++i)
	{
		if (bodies[i])
		{
			box_add_hydro_forces (i,viscosity_scaling);
		}
	}

	if (logging)
	{
		// log_monitors_update (monitors);
	}

	for (i = 0; i < NUM_BOXES; i++)
	{
		for (j = 0; j < 3; j++)
		{
			data[i][j] = extero_force[i][j];		
		}
	}
	
}

void
hydro_webots_cleanup ()
{
	if (logging)
	{
		// log_monitors_destroy (monitors);
	}
}

void
hydro_webots_enable ()
{
	enable_hydro = 1;
}

void
hydro_webots_disable ()
{
	enable_hydro = 0;
}

void
hydro_webots_set_archimede_factor (double factor)
{
	archimede_factor = factor;
}

void
hydro_webots_set_water_level (double level)
{
	water_level = level;
}
