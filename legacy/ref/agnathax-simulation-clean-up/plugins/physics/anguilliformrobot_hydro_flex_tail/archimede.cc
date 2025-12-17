#include <stdio.h>

#include "archimede.h"

void
print_state (Face *face, Vector3 vertices[], double tangent[])
{
	int i;

	printf ("\n");
	for (i = 0; i < face->n; ++i)
	{
		printf ("%d:\t%f\t%f\t%f\t%f\n",
		        face->i[i],
		        tangent[face->i[i]],
		        vertices[face->i[i]][X], vertices[face->i[i]][Y], vertices[face->i[i]][Z]);
	}
	printf ("\n");
	printf ("\n");
}

static void
face_add_vertex (Face *face, int index)
{
	face->i[face->n] = index;
	++face->n;
}

static int
calculate_intersection (Vector3 vertices[],
                        int     ei[BOX_VERTICES][BOX_VERTICES],
                        int     e1,
                        int     e2,
                        Face   *cut_face,
                        double  water_level)
{
	if (ei[e1][e2])				/* already processed */
	{
		return ei[e1][e2];
	}

	double y1 = vertices[e1][Y];
	double y2 = vertices[e2][Y];

	if (y1 <= water_level && y2 <= water_level)
	{
		return ei[e1][e2] = ei[e2][e1] = UNDER_WATER;
	}

	if (y1 > water_level && y2 > water_level)
	{
		return ei[e1][e2] = ei[e2][e1] = ABOVE_WATER;
	}

	double lambda = (water_level - y1) / (y2 - y1);

	int intersection = BOX_VERTICES + cut_face->n;

	vertices[intersection][X] = vertices[e1][X] + lambda * (vertices[e2][X] - vertices[e1][X]);
	vertices[intersection][Y] = water_level;
	vertices[intersection][Z] = vertices[e1][Z] + lambda * (vertices[e2][Z] - vertices[e1][Z]);

	face_add_vertex (cut_face, intersection);

	return ei[e1][e2] = ei[e2][e1] = intersection;
}

static void
trimesh_add_face (Trimesh *tri, Face *face)
{
	int vertex;

	for (vertex = 1; vertex < face->n - 1; ++vertex)
	{
		tri->i[3 * tri->n] = face->i[0];
		tri->i[3 * tri->n + 1] = face->i[vertex];
		tri->i[3 * tri->n + 2] = face->i[vertex + 1];
		++tri->n;
	}
}

void
archimede_trimesh_print (Trimesh *tri, Vector3 vertices[])
{
	int i;

	int max_index = 0;

	for (i = 0; i < tri->n; ++i)
	{
		int j;

		for (j = 0; j < 3; ++j)
		{
			if (tri->i[3 * i + j] > max_index)
			{
				max_index = tri->i[3 * i + j];
			}
		}
	}

	for (i = 0; i <= max_index; ++i)
	{
		printf ("%f\t%f\t%f\n",
		        vertices[tri->i[i]][X],
		        vertices[tri->i[i]][Y],
		        vertices[tri->i[i]][Z]);
	}

	for (i = 0; i < 3; ++i)
	{
		int j;
		for (j = 0; j < tri->n; ++j)
		{
			printf ("\t%d", tri->i[3 * j + i]);
		}
		printf ("\n");
	}
}

void
archimede_face_print (Face *face, Vector3 vertices[])
{
	int i;
	for (i = 0; i < face->n; ++i)
	{
		printf ("%d:\t%f\t%f\t%f\n", face->i[i], vertices[face->i[i]][X], vertices[face->i[i]][Y], vertices[face->i[i]][Z]);
	}
	printf ("\n");
}

static void
insertion_sort (unsigned int indices[],
                int          start,
                int          stop,
                Vector       data,
                int          step)
{
	int i;

	for (i = start + 1; i <= stop; ++i)
	{
		int j = i - 1;
		int vi = indices[i];
		int vj = indices[j];

		while (j >= 1 && data[step * vj] > data[step * vi])
		{
			indices[j + 1] = vj;
			j = j - 1;
			vj = indices[j];
		}
		indices[j + 1] = vi;
	}
}

/* Order a face's vertices counter-clockwise (when looking from "above", i.e. from high Webots y */
static void
face_order_ccw (Face *face, Vector3 vertices[])
{
	/* Construct counter-clockwise face from intersection points */
	/* The idea is to calculate the angle from an origin to each point, and sort the points by this angle. */
	/* By taking the left-most point as origin, we ensure that the angle is in (-pi/2, pi/2), so that we
	 * can sort by tan(angle) = y/x instead of angle, which is much faster (and accurate).
	 * In case of several left-most points, we take the lowest one, so that we can sort points with
	 * identical tangent by increasing x or decreasing y (see below). */

	/* Find (lowest) left-most point */
	int o = 0;
	Vector origin = vertices[face->i[0]]; 

	int i;
	for (i = 1; i < face->n; ++i)
	{
		Vector p = vertices[face->i[i]];

		if (p[X2D] < origin[X2D] ||
		    (p[X2D] == origin[X2D] && p[Y2D] < origin[Y2D]))
		{
			o = i;
			origin = p;
		}
	}

	/* Calculate tangent of angle from origin (gives nan for origin itself but we don't sort it;
	 * gives -Inf or Inf for other points of same x as origin, which will sort correctly) */
	double tangent[MAX_VERTICES];

	for (i = 0; i < face->n; ++i)
	{
		tangent[face->i[i]] = (vertices[face->i[i]][Y2D] - origin[Y2D]) / (vertices[face->i[i]][X2D] - origin[X2D]);
	}

	/* move the origin to the front (swap values) */
	int temp = face->i[o];
	face->i[o] = face->i[0];
	face->i[0] = temp;

	/* Sort by increasing tangent (same as increasing angle), skipping first element  */
	insertion_sort (face->i, 1, face->n - 1, tangent, 1);

	/* We now need to handle vertices that are aligned with the origin, and thus give the same tangent.
	 * Since the polygon is convex, this can only happen directly next to the origin, i.e. for vertices
	 * of minimum or maximum tangent.
	 * In the first case, we can simply sort by increasing x, and in the second case by increasing y. */

	double tan_min = tangent[face->i[1]]; /* skip the first value which is nan */
	double tan_max = tangent[face->i[face->n - 1]]; /* can be Inf */

	int last_min = 1;
	while (last_min < face->n - 1 && tangent[face->i[last_min + 1]] == tan_min)
	{
		++last_min;
	}

	/* Sort minimal tangents by increasing x */
	insertion_sort (face->i, 1, last_min, &vertices[0][X2D], 3);

	int first_max = face->n - 1;
	while (first_max > 0 && tangent[face->i[first_max - 1]] == tan_max)
	{
		--first_max;
	}

	/* Sort maximal tangents by decreasing y */
	insertion_sort (face->i, first_max, face->n - 1, &vertices[0][Y2D], 3);
}

void
box_water_intersection (Vector3     vertices[MAX_VERTICES],
                        Face const  faces_in[BOX_FACES],
                        Face        faces_out[MAX_FACES],
                        Trimesh    *trimesh,
                        double      water_level)
{
	/* There are 2 triangles per face, plus at most 10 due to the intersection */

	/* Matrix of intersections between edges and water plane */
	int ei[BOX_VERTICES][BOX_VERTICES] = {{0,},}; /* 0 = not calculated yet
	                                               * -1 = above water
	                                               * -2 = under water
	                                               * positive value = intersection vertex index */

	Face *cut_face = &faces_out[BOX_FACES];
	cut_face->n = 0;

	trimesh->n = 0;

	int face;
	for (face = 0; face < BOX_FACES; ++face)
	{
		faces_out[face].n = 0;
		
		int edge;
		for (edge = 0; edge < EDGES_PER_FACE; ++edge)
		{
			int v1 = faces_in[face].i[edge];
			int v2 = faces_in[face].i[edge + 1];
			
			int e = calculate_intersection (vertices, ei, v1, v2, cut_face, water_level);

			if (e > 0)
			{
				face_add_vertex (&faces_out[face], e);
			}

			if (e == UNDER_WATER || (e > 0 && vertices[v1][Y] > water_level))
			{
				face_add_vertex (&faces_out[face], v2);
			}
		}

		trimesh_add_face (trimesh, &faces_out[face]);
	}

	if (cut_face->n > 0)
	{
		face_order_ccw (cut_face, vertices);
		trimesh_add_face (trimesh, cut_face);
	}
}

/* Sets second argument to the trimesh centroid, returns the trimesh volume */
static double 
calculate_centroid (Vector3  vertices[],
                    Trimesh *trimesh,
                    Vector3  centroid)
{
	double const mult[4] = {1.0 / 6, 1.0 / 24, 1.0 / 24, 1.0 / 24};
	double intg[4] = {0, 0, 0, 0};
	int t;

	for (t = 0; t < trimesh->n; t++)
	{
		int i0 = trimesh->i[3 * t];
		int i1 = trimesh->i[3 * t + 1];
		int i2 = trimesh->i[3 * t + 2];

		double x0 = vertices[i0][X], y0 = vertices[i0][Y], z0 = vertices[i0][Z];
		double x1 = vertices[i1][X], y1 = vertices[i1][Y], z1 = vertices[i1][Z];
		double x2 = vertices[i2][X], y2 = vertices[i2][Y], z2 = vertices[i2][Z];

		double a1 = x1 - x0;
		double b1 = y1 - y0;
		double c1 = z1 - z0;

		double a2 = x2 - x0;
		double b2 = y2 - y0;
		double c2 = z2 - z0;

		double d0 = b1 * c2 - b2 * c1;
		double d1 = a2 * c1 - a1 * c2;
		double d2 = a1 * b2 - a2 * b1;

		double f1x = x0 + x1 + x2;
		double f2x = x0 * x0 + x1 * (x0 + x1) + x2 * f1x;
		double f2y = y0 * y0 + y1 * (y0 + y1) + y2 * (y0 + y1 + y2);
		double f2z = z0 * z0 + z1 * (z0 + z1) + z2 * (z0 + z1 + z2);

		intg[0] += d0 * f1x;
		intg[1] += d0 * f2x;
		intg[2] += d1 * f2y;
		intg[3] += d2 * f2z;
	}
	
	double volume = intg[0] * mult[0];

	int i;
	for (i = 1; i < 4; i++)
	{
		centroid[i - 1] = intg[i] * mult[i] / volume;
	}

	return volume;
}

/* vertices:    First BOX_VERTICES elements must be the full box vertices.
 *              Extra space must be available for intersection vertices.
 * centroid:    Must be initialized to the full box centroid. Will be set to
 *              the centroid of the immersed part.
 * faces_in:    Vertex indices defining the full box faces, counter-clockise.
 * faces_out:   Will receive the faces of the immersed part.
 * full_volume: Must be the volume of the full box.
 *
 * Returns the volume of the immersed part. */
double
archimede_volume (Vector3     vertices[MAX_VERTICES],
                  Face const  faces_in[BOX_FACES],
                  Face        faces_out[MAX_FACES],
                  Vector3     centroid,
                  double      full_volume,
                  double      water_level)
{
	/* Count number of corners that are under water */
	int i;
	int n_underwater = 0;
	for (i = 0; i < BOX_VERTICES; ++i)
	{
		if (vertices[i][Y] <= 0)
		{
			++n_underwater;
		}
	}

	if (n_underwater == BOX_VERTICES)
	{
		int i;

		for (i = 0; i < BOX_FACES; ++i)
		{
			faces_out[i] = faces_in[i];
		}

		faces_out[BOX_FACES].n = 0; /* no intersection face */

		return full_volume;		/* centroid is already correct in this case */
	}

	if (n_underwater == 0)
	{
		int i;

		for (i = 0; i < MAX_FACES; ++i)
		{
			faces_out[i].n = 0;
		}

		return 0;				/* centroid is undefined in this case */
	}

	Trimesh trimesh = {{0,}, 0};

	box_water_intersection (vertices, faces_in, faces_out, &trimesh, water_level);

	return calculate_centroid (vertices, &trimesh, centroid);
}

/* int main(int argc, char *argv[]) */
/* { */
/* 	/\* Normalized box corners. Extra vertices are allocated for the intersection points. *\/ */
/* 	Vector3 vertices[MAX_VERTICES] = {{0, 0, 0}, */
/* 	                                  {1, 0, 0}, */
/* 	                                  {1, 1, 0}, */
/* 	                                  {0, 1, 0}, */
/* 	                                  {0, 0, 1}, */
/* 	                                  {1, 0, 1}, */
/* 	                                  {1, 1, 1}, */
/* 	                                  {0, 1, 1},}; */

/* 	/\* Box faces, with 5th index for easy edge extraction. */
/* 	 * Gives faces counter-clockwise as seen from outside the box. *\/ */
/* 	static const Face faces[BOX_FACES] = {{{3, 2, 1, 0, 3}, 4}, */
/* 	                                      {{0, 1, 5, 4, 0}, 4}, */
/* 	                                      {{4, 5, 6, 7, 4}, 4}, */
/* 	                                      {{7, 6, 2, 3, 7}, 4}, */
/* 	                                      {{1, 2, 6, 5, 1}, 4}, */
/* 	                                      {{3, 0, 4, 7, 3}, 4}}; */

/* 	Trimesh trimesh = {{0,}, 0}; */
	
/* 	Face cut_face = {{0,}, 0}; */

/* 	box_plane_intersection (vertices, faces, &trimesh, &cut_face, 0.5); */
    
/* 	return 0; */
/* } */
