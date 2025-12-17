#ifndef __ARCHIMEDE_H
#define __ARCHIMEDE_H

#include <misc/matrix/matrix.h>

#define BOX_VERTICES      8
#define BOX_FACES         6
#define MAX_INTERSECTIONS 6		/* between water plane and box */
#define EDGES_PER_FACE    4		/* for normal box face */
#define MAX_FACE_VERTICES 6		/* per face of intersected box */
#define MAX_FACES         (BOX_FACES + 1) /* there can be zero or one additional face for the intersection */

#define MAX_VERTICES (BOX_VERTICES + MAX_INTERSECTIONS)

#define UNDER_WATER -2
#define ABOVE_WATER -1

typedef struct {
	unsigned int i[3 * (2 * BOX_FACES + 10)];
	int n;
} Trimesh;

typedef struct {
	unsigned int i[MAX_FACE_VERTICES + 1]; /* +1 to repeat the first vertex at the end, for easy edge extraction */
	int n;
} Face;

enum {X, Y, Z};

/* For the intersection polygon, the outside of the box is webots_y > water_level, so to look at the
 * added polygon from outside with a right-hand coordinate system, we define the 2D x-y space as the
 * z-x space in Webots. */
#define X2D Z
#define Y2D X

double archimede_volume (Vector3    vertices[MAX_VERTICES],
                         Face const faces_in[BOX_FACES],
                         Face       faces_out[MAX_FACES],
                         Vector3    centroid,
                         double     full_volume,
                         double     water_level);
void archimede_trimesh_print (Trimesh *tri, Vector3 vertices[]);
void archimede_face_print (Face *face, Vector3 vertices[]);

#endif
