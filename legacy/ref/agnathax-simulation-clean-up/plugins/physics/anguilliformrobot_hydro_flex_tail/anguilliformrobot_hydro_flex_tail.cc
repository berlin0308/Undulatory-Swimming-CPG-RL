#include <stdlib.h>
#include <ode/ode.h>
#include <plugins/physics.h>
#include <ode/ode.h>

#include "hydro_webots.h"
#include "hydro_boxes.h"
#include <stdbool.h>
#include <iostream>
#include <string>
#include <sstream>

unsigned int N_SERVOS;
unsigned int N_SERVOS_TOT;

extern unsigned int NUM_BOXES;
static dJointID joints[N_SERVOS_MAX+N_SERVOS_TAIL];
static dBodyID bodies[N_SERVOS_MAX+N_SERVOS_TAIL];
dReal torque;
float left,right;
float actv[N_SERVOS_MAX][2];
float pmtr[N_MUSCLE_PARAMS];
bool cntrl;

float body_pos[N_SERVOS_MAX][3];

static Vector3 colors[] = {{1, 0, 0},
                           {0, 1, 0},
                           {0, 0, 1},
                           {0, 1, 1},
                           {1, 0, 1},
                           {1, 1, 0},
                           {0, 0, 0},
                           {1, 1, 1},
                           {0, 0.5, 1}};

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

void
webots_physics_draw ()
{
	hydro_webots_draw ();

	if(DRAW_ACTIVATIONS)
	{
		// change OpenGL state
		glDisable(GL_LIGHTING);    // not necessary
		glLineWidth(10);           // use a thick line
		glDisable(GL_DEPTH_TEST);  // draw in front of Webots graphics

		glPointSize(10);
	  
	  	int i = 0;
	  	for(i=0;i<N_SERVOS;i++)
	  	{
			glBegin(GL_POINTS);

			(actv[i][0]<0) ? glColor3dv(colors[CYAN]): glColor3dv(colors[WHITE]) ; //(contraction on the left side: cyan)
			glVertex3f(body_pos[i][0],body_pos[i][1],body_pos[i][2]);
			//glVertex3f(0,0,0);
			glEnd();
		}
	}
}

void
webots_physics_init (dWorldID      w,
                     dSpaceID      s,
                     dJointGroupID j)
{
  
	uint idx_joint = 0; // Assumes 0 as the index of the joint attached to the body. 

	const char * ROBOT_NAME = "ROBOT";
	const char * JOINT_NAME_PREFIX = "JOINT_SERVO_";
	joints[0] = dBodyGetJoint(dWebotsGetBodyFromDEF(ROBOT_NAME), idx_joint); // Get the ROBOT
	bodies[0] = dWebotsGetBodyFromDEF(ROBOT_NAME);

	uint i = 1;
	while(true){
		std::stringstream ss;
		ss << std::string(JOINT_NAME_PREFIX) << i;
		if(dWebotsGetBodyFromDEF(ss.str().c_str()))
		{
			joints[i] = (dBodyGetJoint(dWebotsGetBodyFromDEF(ss.str().c_str()), idx_joint));
			bodies[i] = dWebotsGetBodyFromDEF(ss.str().c_str());
			actv[i][0] = 0.0;
			actv[i][1] = 0.0;  
		}
		else{
			//std::cout << ss.str().c_str() << " not found" << std::endl;
			break;
		}
		i++;
	}
	N_SERVOS_TOT = i-1;
	N_SERVOS = N_SERVOS_TOT - N_SERVOS_TAIL;//i-1-10; // THE -10 is to remove the tail joint
	NUM_BOXES = N_SERVOS_TOT+1;

	std::cout << "Number of joint found in Physics = " << N_SERVOS << std::endl;
	for(i=0; i<N_MUSCLE_PARAMS;i++)
	{
		pmtr[i] = 0.0;
	}
	cntrl = 0;

	init_boxes(N_SERVOS,N_SERVOS_TAIL);
	hydro_webots_init (w, s, j, NULL, NULL, 1);
	hydro_webots_enable ();

}

void
webots_physics_step ()
{
	float fdata[N_SERVOS_MAX+1][3];
	hydro_webots_step(fdata);
	// dWebotsConsolePrintf("%f\n",fdata[0][0]);

	int size;
	uint i,j;

	packet_controller_to_physics *Packet;
	Packet = (packet_controller_to_physics*)dWebotsReceive(&size); // have to do an explicit (not implicit) cast from void* to packet* !!!!!!

	// body positions to draw activation
	for(i = 0; i<N_SERVOS; i++)
	{
		const double *p = dBodyGetPosition(bodies[i+1]);
      //dBodyGetMass(all_bodies[i], &m);
      /*if(i == 1)
        dWebotsConsolePrintf("pos %s: %f\n",BODY_DEFS[i],pos[1]);*/
      body_pos[i][0] = p[0];
      body_pos[i][1] = p[1];
      body_pos[i][2] = p[2];
	}


	if(Packet!=NULL)
	{
		packet_physics_to_controller* Packet_back = new packet_physics_to_controller();
		memset(Packet_back, 0, sizeof(*Packet_back));

		// get control mode (0: torque control, 1: position control)
		cntrl = Packet->cntrl;

		for(i=0; i<N_SERVOS; i++)
		{
			actv[i][0] = Packet->actv[i][0];
			actv[i][1] = Packet->actv[i][1];
		}
		for(i=0; i<N_MUSCLE_PARAMS; i++)
		{
			pmtr[i] = Packet->pmtr[i];
		}
	
		for(i=0; i<N_SERVOS_TOT+1; i++) // +1 because NUM_BOXES = N_SERVOS_TOT + 1
		{
			dReal position = dJointGetHingeAngle(joints[i]);
			dReal speed = dJointGetHingeAngleRate(joints[i]);

			// We set the force on N+1 segments if N is the number of servos
			if(i<NUM_BOXES){
				for (j = 0; j < 3; j++)
				{
					Packet_back->forces[i][j] = fdata[i][j];
				}
			}
			
			if(i<N_SERVOS)// body
			{
				left  = actv[i][0];
				right = actv[i][1];
				torque = pmtr[0]*(left-right) - pmtr[1]*position - pmtr[2]*speed;
				// if(i==3){
				// 	dWebotsConsolePrintf("physics pos: %f\n",position);
				// 	dWebotsConsolePrintf("left: %f , right: %f \n",left,right);
				// 	// dWebotsConsolePrintf("physics torque: %f\n",torque);
				// }

				Packet_back->torq[i] = (float)torque;
				Packet_back->speed[i] = (float)speed;
				if(!(cntrl)) // only apply torques when in torque control
				{
					dJointAddHingeTorque(joints[i], torque);
				}	
			}
			else if (i<N_SERVOS_TOT) // tail
			{
				torque = -P_STIFFNESS*position - P_DAMPING*speed;
				dJointAddHingeTorque(joints[i], torque);
			}

		}

		if(sizeof(*Packet_back)>=1024)
		{
			dWebotsConsolePrintf(" PACKET SIZE EXCEEDS BUFFER SIZE: LIMITED TO 1024 !!!!\n ");
		}
		else
		{
			dWebotsSend(1, Packet_back, sizeof(*Packet_back));
		}	  
	}
	else
	{	
		for(i=0; i<N_SERVOS_TOT+1; i++) // +1 because NUM_BOXES = N_SERVOS_TOT + 1
		{
			dReal position = dJointGetHingeAngle(joints[i]);
			dReal speed = dJointGetHingeAngleRate(joints[i]);
			
			if(i<N_SERVOS)// body
			{
				left  = actv[i][0];
				right = actv[i][1];
				torque = pmtr[0]*(left-right) - pmtr[1]*position - pmtr[2]*speed;
				if(!cntrl) // only apply torques when in torque control
				{
					dJointAddHingeTorque(joints[i], torque);
				}	
			}
			else if (i<N_SERVOS_TOT) // tail
			{
				torque = -P_STIFFNESS*position - P_DAMPING*speed;
				dJointAddHingeTorque(joints[i], torque);
			}

		}
	}
}

void
webots_physics_cleanup ()
{
	hydro_webots_cleanup ();
}
