#include <stdio.h>
#include <math.h>

#include "matrix.h"

/* int main(int argc, char *argv[]) */
/* { */
/* 	Quaternion q; */
/* 	Vector3 axis = {0, 0, 1}; */

/* 	quaternion_from_axis_angle (axis, M_PI / 4, q); */

/* 	puts ("Rotation"); */
/* 	quaternion_print (q); */

/* 	Vector3 x = {1, 0, 0}; */
/* 	Vector3 out; */
/* 	quaternion_rotation (q, x, out); */

/* 	puts ("v1"); */
/* 	vector3_print (x); */

/* 	puts ("v2"); */
/* 	vector3_print (out); */

/*     return 0; */
/* } */

int main(int argc, char *argv[])
{
	Quaternion q;

	Vector3 axis = {
		-0.000975,
		0.009903,
		-0.000083
	};
	puts ("Axis");
	vector3_print (axis);

	double angle = -M_PI / 2;
	printf ("Angle: %f\n", angle);

	quaternion_from_axis_angle (axis, angle, q);
	puts ("Quaternion");
	quaternion_print (q);

	Vector3 v1 = {
		-0.009916,
		-0.000983,
		-0.000845
	};
	puts ("v1");
	vector3_print (v1);

	Vector3 out;
	quaternion_rotation (q, v1, out);
	puts ("v2");
	vector3_print (out);

	Vector3 axis_test;
	double angle_test;
	quaternion_to_axis_angle (q, axis_test, &angle_test);
	
	puts ("Axis test");
	vector3_print (axis_test);

	printf ("Angle test: %f\n", angle_test);

    return 0;
}
