#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include <misc/gauss/gauss.h>

#include "hydro.h"


static Vector3 e1 = { 1, 0, 0 };
/* static Vector3 e2 = { 0, 1, 0 }; */
/* static Vector3 e3 = { 0, 0, 1 }; */

#define SQUARE(X)  ((X) * (X))


static double
signed_square (double x)
{
	return fabs (x) * x;
}

static void
hydro_segment_added_mass_field (HydroSegment *segment)
{
	Vector3 coef_inertia;
	double coef_angular_inertia;

	double a = segment->width / 2.0;
	double b = segment->height / 2.0;

	/* linear inertia coefficents. Was [nothing, C4, C5] */
	coef_inertia[0] = 0;
	coef_inertia[1] = segment->hydro->cm * M_PI * segment->hydro->rho_fluid * SQUARE (b);
	coef_inertia[2] = segment->hydro->cm * M_PI * segment->hydro->rho_fluid * SQUARE (a); /* was 0 */

	/* note: coef_angular_ineratia might need adjustment with another fitted coefficient like cm */
    /* was C6, Jf1 in deliverable. See Newman, Marine hydrodynamics p. 145 */
	coef_angular_inertia = 0.125 * M_PI * segment->hydro->rho_fluid * SQUARE (a * a - b * b);

	matrix3_zeros (segment->mf);
	segment->mf[1][1] = coef_inertia[1];
	segment->mf[2][2] = coef_inertia[2];

	matrix3_zeros (segment->If);
	segment->If[0][0] = coef_angular_inertia;
		                                
	matrix6_by_blocks3 (segment->IIf, segment->mf, NULL, NULL, segment->If);
}

static void
hydro_segment_added_mass_diff (double        x,
                               Matrix6       dIadd,
                               HydroSegment *segment)
{
	Vector3 x_e1;
	Matrix6 A, AT;

	vector3_multiply (e1, x, x_e1);
	vector3_translation_transformation_matrix (x_e1, AT);
	matrix6_transpose (AT, A);
	matrix6_product_matrix6 (A, segment->IIf, dIadd);
	matrix6_product_matrix6 (dIadd, AT, dIadd);
}

static void
hydro_segment_external_force_diff (double        x,
                                   Vector6       dFext,
                                   HydroSegment *segment)
{
	Matrix6 A, AT;
	Vector v = LINEAR (segment->velocity);
	Vector omega = ANGULAR (segment->velocity);
	Vector3 vx, pf, sigmaf, F_drag_pr, C_drag_pr, x_e1;
	Vector3 coef_viscosity;
	double coef_angular_viscosity;
	Vector3 C_linear_drag;
	Vector6 Tho_ma_pr;
	Vector3 omega_cross_v, omega_cross_pf, mf_omega_cross_v, omega_cross_sigmaf, vx_cross_pf;
	Vector3 vx_signed_square, omega_square_pr;
	double reynolds;
	double perimeter = 2 * segment->height + 2 * segment->width;

	vector3_multiply (e1, x, x_e1);

	/* twist transformation matrix from origin to x, or is it from x to origin? */
	vector3_translation_transformation_matrix (x_e1, AT);

	/* wrench transformation matrix */
	matrix6_transpose (AT, A);
	
	/* linear velocity transfer from origin to point x  */
	vector3_cross_vector (omega, x_e1, vx);
	vector3_sum (v, vx, vx);

	reynolds = fabs (vx[0]) < 0.01 ? 8000 : 1000000 * fabs (vx[0]) * 0.855;

	/* linear drag coefficents. Was [C0, C1, C2] */
	coef_viscosity[0] = 0.5 * segment->hydro->cf * 0.455 / (pow (log10 (reynolds), 2.58)) * perimeter * segment->hydro->rho_fluid; /* C1 */
	coef_viscosity[1] = segment->hydro->cd * 0.5 * segment->hydro->rho_fluid * segment->height; /* C2 */
	coef_viscosity[2] = segment->hydro->cd * 0.5 * segment->hydro->rho_fluid * segment->width; /* was C3 = 0 */

	coef_angular_viscosity = 0.1; /* Was C4 = 0. Tune to reduce rolling (no formula for it). */

	matrix3_product_vector (segment->mf, vx, pf);
	matrix3_product_vector (segment->If, omega, sigmaf);

	vector3_map (vx, signed_square, vx_signed_square);


	/* Resistive wrench at X = 0 */
	/* linear velocity drag force at X = x: */
	vector3_times_vector (coef_viscosity, vx_signed_square, F_drag_pr);

	/* moment at X = 0 from drag force at X = x */
	vector3_cross_vector (x_e1, F_drag_pr, C_linear_drag);

	/* moment from angular velocity drag force */
	vector3_multiply (e1, coef_angular_viscosity * signed_square (omega[0]), omega_square_pr);

	/* total drag moment at X = 0: */
	vector3_sum (C_linear_drag, omega_square_pr, C_drag_pr);


	/* Reactive wrench at X = x */
	/* linear part: */
	vector3_cross_vector (omega, v, omega_cross_v);
	matrix3_product_vector (segment->mf, omega_cross_v, mf_omega_cross_v);
	vector3_cross_vector (omega, pf, omega_cross_pf);
	/* vector3_multiply (omega_cross_pf, -1, omega_cross_pf); */
	/* dWebotsConsolePrintf ("Segment %d: v %f %f %f, vx %f %f %f, pf %f %f %f, ocpf %f %f %f\n", */
	/*                       segment->index, */
	/*                       v[0], v[1], v[2], */
	/*                       vx[0], vx[1], vx[2], */
	/*                       pf[0], pf[1], pf[2], */
	/*                       omega_cross_pf[0], omega_cross_pf[1], omega_cross_pf[2]); */
	vector3_difference (omega_cross_pf, mf_omega_cross_v, LINEAR (Tho_ma_pr));

	/* angular part: */
	vector3_cross_vector (omega, sigmaf, omega_cross_sigmaf);
	vector3_cross_vector (vx, pf, vx_cross_pf);
	vector3_sum (omega_cross_sigmaf, vx_cross_pf, ANGULAR (Tho_ma_pr));
	/* vector3_multiply (ANGULAR (Tho_ma_pr), 0, ANGULAR (Tho_ma_pr)); /\* XXX *\/ */

	/* dWebotsConsolePrintf ("Segment %d: ocpf %f %f %f, thox %f %f %f", segment->index, */
	/*                       omega_cross_pf[0], omega_cross_pf[1], omega_cross_pf[2], */
	/*                       Tho_ma_pr[0], Tho_ma_pr[1], Tho_ma_pr[2]); */

	/* Translated to X = 0 */
	matrix6_product_vector (A, Tho_ma_pr, Tho_ma_pr);

	/* dWebotsConsolePrintf (", tho0 %f %f %f\n", Tho_ma_pr[0], Tho_ma_pr[1], Tho_ma_pr[2]); */
	/* vector_multiply (Tho_ma_pr, 0, Tho_ma_pr, 6); /\* XXX *\/ */

	/* Sum reactive and resistive wrenches, write to output dFext: */
	vector_copy (dFext, Tho_ma_pr, 6);
	vector3_sum (LINEAR (dFext), F_drag_pr, LINEAR (dFext));
	vector3_sum (ANGULAR (dFext), C_drag_pr, ANGULAR (dFext));
}

void
hydro_segment_calculate_added_mass (HydroSegment *segment)
{
	Matrix6 diff;

	hydro_segment_added_mass_field (segment);

	gauss_quadrature (-segment->origin_offset,
	                  segment->length - segment->origin_offset,
	                  (GaussCallback *) hydro_segment_added_mass_diff,
	                  segment,
	                  (Vector) diff,
	                  (Vector) segment->Iadd,
	                  36);
}

void
hydro_segment_get_segment_force (HydroSegment *segment,
                                 Vector6       segment_force)
{
	Vector6 diff;
	
	gauss_quadrature (-segment->origin_offset,
	                  segment->length - segment->origin_offset,
	                  (GaussCallback *) hydro_segment_external_force_diff,
	                  segment,
	                  diff,
	                  segment_force,
	                  6);
}

void
hydro_segment_get_caudal_force (HydroSegment *segment,
                                Vector6       caudal_force)
{
	Vector3 v_tip;				/* velocity at tail tip */
	Vector3 omega_cross_x;
	double coef_reac = -segment->hydro->cm * M_PI * segment->hydro->rho_fluid * SQUARE (segment->height) / 4.0;
	Vector3 x = {segment->length - segment->origin_offset, 0, 0}; /* vector from origin to tail tip */

	vector3_cross_vector (ANGULAR (segment->velocity), x, omega_cross_x);
	vector3_sum (LINEAR (segment->velocity), omega_cross_x, v_tip);

	caudal_force[0] = -0.5 * SQUARE (v_tip[1]) * coef_reac;
	caudal_force[1] = v_tip[0] * v_tip[1] * coef_reac;
	caudal_force[2] = 0;

	vector3_cross_vector (x, LINEAR (caudal_force), ANGULAR (caudal_force));
}

void
hydro_segment_get_head_force (HydroSegment *segment, Vector6 head_force)
{
	double coef_res = 0.5 * segment->hydro->rho_fluid * segment->width * segment->height * segment->hydro->cp;

	/* velocity at the front of the head. */
	/* We only need the x component, which is identical at the origin and at the front of the head since both points lie on the x axis */
	double v_front = segment->velocity[0]; 

	vector_zeros (head_force, 6);
	head_force[0] = signed_square (v_front) * coef_res;
}
