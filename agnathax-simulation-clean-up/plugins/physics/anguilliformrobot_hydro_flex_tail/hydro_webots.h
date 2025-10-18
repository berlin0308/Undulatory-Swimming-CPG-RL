#ifndef __HYDRO_WEBOTS_H
#define __HYDRO_WEBOTS_H

#include <ode/ode.h>


#include "hydro_boxes.h"

#define GRAVITY              9.81
#define WATER_DENSITY        1000.0

#define DEFAULT_WATER_LEVEL           0.0
#define	DEFAULT_ARCHIMEDE_FACTOR	1.6//1.6//1.6     /* default correction for tuning the Archimede force */

/* red arrow (hydro X axis) should point towards the wake,
 * biggest rotations should be around the blue arrow (hydro Z axis) */
#define DRAW_SCALE_BOX_AXES           0
#define DRAW_ACTIVATIONS			  0

#define DRAW_SCALE_HYDRO_FORCES       0
#define DRAW_SCALE_HYDRO_TORQUES      0
#define DRAW_SCALE_ARCHIMEDE_FORCES   0//0.1

#define DRAW_ARCHIMEDE_FACES          0


void hydro_webots_draw ();
void hydro_webots_init (dWorldID w, dSpaceID s, dJointGroupID j, char const *head, char const *tail, int add_mass);
void hydro_webots_step (float f[][3]);
void hydro_webots_cleanup ();
void hydro_webots_enable ();
void hydro_webots_disable ();
void hydro_webots_set_archimede_factor (double factor);
void hydro_webots_set_water_level (double level);


#endif
