// File:          	webots_undulatory_swimming.cpp
// Date:          	23-11-2015
// Description:		controller to explore undulatory swimming with CPG / proprioceptive and exterceptive feedback
// Author:        	Robin Thandiackal, Florin Dzeladini
// Modifications: 	-

#include "anguilliformrobot.hpp"

void 
AnguilliformRobot::projection(double a[3], double b[3], double (&c))
{
	static double val = 0, na = 0, nb = 0;
	val = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	na = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	nb = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);

	c = (na==0 || nb==0) ? 0 : val/(na*nb) * nb;
}

void 
AnguilliformRobot :: instantenousSpeed(double timestep, double timeConstant)
{
	// local variables
	double prevPos[2]={0.0}, currPos[2]={0.0}, meanVal=0.0, stdVal=0.0;
	double lastElementPosX = 0.0, lastElementPosZ = 0.0;
	double dotProduct = 0;

	// aquire current position measurement of head
	posX.push_back(bodies[0]->getPosition()[0]);
	posZ.push_back(bodies[0]->getPosition()[2]);

	// aquire current position measurement of last element
	lastElementPosX = bodies[N_SERVOS-1]->getPosition()[0];	
	lastElementPosZ = bodies[N_SERVOS-1]->getPosition()[2];

	if(posX.size() >= timestep/(TIME_STEP/1000.0))
	{
		// keep fixed number of measurements in vector
		posX.erase(posX.begin());
		posZ.erase(posZ.begin());
		speed.erase(speed.begin());
		meanSpeed.erase(meanSpeed.begin());
		stdSpeed.erase(stdSpeed.begin());

		// previous position from one period ago
		prevPos[0] = posX[0];
		prevPos[1] = posZ[0];

		// current position
		currPos[0] = posX.back();			
		currPos[1] = posZ.back();

		// speed based on travelled distance within period
		instSpeed = sqrt(	(currPos[0]-prevPos[0])*(currPos[0]-prevPos[0]) + 
							(currPos[1]-prevPos[1])*(currPos[1]-prevPos[1])	)/timestep;

		// sign of speed
		dotProduct = (currPos[0]-prevPos[0]) * (currPos[0]-lastElementPosX) + (currPos[1]-prevPos[1]) * (currPos[1]-lastElementPosZ);
		if (dotProduct < 0)
		{
			instSpeed = -instSpeed;
		}

		speed.push_back(instSpeed);

		// mean and std speed
		meanStdVector(speed,meanVal,stdVal);
		meanSpeed.push_back(meanVal);
		stdSpeed.push_back(stdVal);
	}
	else
	{
		// set speed to zero in initial period
		instSpeed = 0.0;
		speed.push_back(0.0);
		meanSpeed.push_back(0.0);
		stdSpeed.push_back(0.0);
	}
}

void
AnguilliformRobot :: amplitudes()
{
	double mn=0,mx=0;
	for(uint i=0; i<N_SERVOS_TOT; i++)
	{
		mn = *min_element(end_cycle_ang[i].begin(),end_cycle_ang[i].end());
		mx = *max_element(end_cycle_ang[i].begin(),end_cycle_ang[i].end());
		ampl[i] = (mx-mn)*0.5;
	}
}

bool
AnguilliformRobot :: averageFrequency()
{

	double mn=0, st=0;
	double sz = jointAngles[0].size();
	vector <double> zc_ind[N_SERVOS_MAX];// zero crossings index
	vector <double> zc_ind_diff[N_SERVOS_MAX]; // differences in no of samples between zero crossings
	int joi = ( (N_SERVOS>1) ? N_SERVOS-1 : N_SERVOS); // joints of interest (exclude last joint, if there is more than one joint)

	avgFrequency = 0.0;
	avgZCStd = 0.0;
	for(int i=0; i<joi;i++) // exclude last joint before end
	{
		// cout << "i: " << i << endl;
		for(int j=0; j<sz-1; j++)
		{
			// signbit returns non-zero if negative, 0 if non-negative
			if( (signbit(jointAngles[i][j]) - signbit(jointAngles[i][j+1])) > 0 ) // from negative to positive
			{
				zc_ind[i].push_back(j);
				// cout << "j:" << j << "\t";
				if(zc_ind[i].size()>1)
				{
					zc_ind_diff[i].push_back( j-zc_ind[i][zc_ind[i].size()-2] );
					// cout << "diff: " <<j-zc_ind[i][zc_ind[i].size()-2] << "\t";
				}		
			}
			//cout << jointAngles[i][j] << " // " << signbit(jointAngles[i][j])<<  endl;
		}
		// cout << endl;
		zcStd[i]=-1000;
		if(zc_ind_diff[i].size()>=1){
			meanStdVector(zc_ind_diff[i],mn,st);
			avgFrequency += mn; // mean number of samples
			//cout << "i: " << i << " // no samples: " << mn << endl;
			zcStd[i] = st/mn;
			avgZCStd += zcStd[i];
		}
		else
		{
			cout << "non periodic state!" << endl;
			return 0;
		}
	}

	avgFrequency = 	1.0/(avgFrequency/joi*TIME_STEP*0.001);
	avgZCStd /= joi;
	cout << "avgFrequency (new): " << avgFrequency << endl;
	cout << "zero_crossing std:";
	for(int i=0; i<joi;i++) // exclude last joint before end
	{
		cout << zcStd[i] << " // ";
	}
	cout << endl;
	cout << "avg ZC std: " << avgZCStd << endl;

	return 1;
}

void
AnguilliformRobot :: straightness()
{
	double sz = headPosX.size();
	double m = (headPosZ[sz-1]-headPosZ[0])/(headPosX[sz-1]-headPosX[0]);// slope
	double q = headPosZ[0]-m*headPosX[0];// offset

	double xp=0.0, yp=0.0, xl=0.0, yl=0.0;

	straigthnessMeas = 0.0;

	for(int i=0; i<sz; i++)
	{
		xp = headPosX[i]; 
		yp = headPosZ[i];

		xl = 1.0/(m+1.0/m)*(yp+xp/m-q);
		yl = m*xl+q;

		straigthnessMeas += sqrt((xl-xp)*(xl-xp)+(yl-yp)*(yl-yp));
	}
	straigthnessMeas /= sz;

}

void 
AnguilliformRobot :: averagePhaseLag()
{
	double mn=0, st=0;
	avgPhaseLag = 0.0;

	for(uint i=0; i<N_SERVOS-1;i++)
	{
		meanStdVector(phaseLag[i],mn,st);
		avgIndividualPhaseLag[i] = mn;
		avgPhaseLag += mn;
	}

	avgPhaseLag *= 1.0/(2*M_PI)/(N_SERVOS-1)*N_SERVOS*100.0; // as percentage 
}

bool 
AnguilliformRobot :: individualPhaseLags()
{
	int sz_kin = jointAngles[0].size();
	int sz_act = jointActivs[0].size();
	vector <double> zc_ind_kin[N_SERVOS_MAX];// zero crossings index
	vector <double> zc_ind_act[N_SERVOS_MAX];// zero crossings index
	int joi = ( (N_SERVOS>2) ? N_SERVOS : N_SERVOS); // joints of interest (exclude last two joint, if there is more than two joints)

	double diff1 = 0, diff2 =0, diff3 =0;

	// determine zero crossings (kinematic and neural)
	// cout << "zero crossings (kin): " << endl;
	for(int i=0; i<joi;i++) // exclude last joint before end
	{
		for(int j=0; j<sz_kin-1; j++)
		{
			// signbit returns non-zero if negative, 0 if non-negative
			if( (signbit(jointAngles[i][j]) - signbit(jointAngles[i][j+1])) > 0 ) // from negative to positive
			{
				zc_ind_kin[i].push_back(j);
				// cout << j << "\t";
			}
			//cout << jointAngles[i][j] << " // " << signbit(jointAngles[i][j])<<  endl;
			
		}
		// cout << endl;
		if(zc_ind_kin[i].size()<2)
		{
			cout << "non periodic state!" << endl;
			return false;
		}
	}
	// cout << "zero crossings (act): " << endl;
	for(int i=0; i<joi;i++)
	{
		for(int j=0;j<sz_act-1; j++)
		{
			// signbit returns non-zero if negative, 0 if non-negative
			if( (signbit(jointActivs[i][j]) - signbit(jointActivs[i][j+1])) > 0 ) // from negative to positive
			{
				zc_ind_act[i].push_back(j);
				// cout << j << "\t";
			}
		}
		// cout << endl;
		if(zc_ind_act[i].size()<2)
		{
			cout << "non periodic state!" << endl;
			return false;
		}
	}

	// determine phase lags (kinematic and neural)
	avgPhaseLagKin = 0;
	avgPhaseLagAct = 0;
	for(int i=0; i<joi-1;i++) // exclude last joint before end
	{
		diff1 = zc_ind_kin[i+1][0] - zc_ind_kin[i][0];
		diff2 = zc_ind_kin[i+1][1] - zc_ind_kin[i][0];
		diff3 = zc_ind_kin[i+1][0] - zc_ind_kin[i][1];

		if(fabs(diff1)<fabs(diff2)){
			phaseLagKin[i] = ( (fabs(diff1)<fabs(diff3)) ? diff1 : diff3 ); 
		}
		else{
			phaseLagKin[i] = ( (fabs(diff2)<fabs(diff3)) ? diff2 : diff3 );
		}
		// cout << phaseLagKin[i] << " // ";

		diff1 = zc_ind_act[i+1][0] - zc_ind_act[i][0];
		diff2 = zc_ind_act[i+1][1] - zc_ind_act[i][0];
		diff3 = zc_ind_act[i+1][0] - zc_ind_act[i][1];

		if(fabs(diff1)<fabs(diff2)){
			phaseLagAct[i] = ( (fabs(diff1)<fabs(diff3)) ? diff1 : diff3 ); 
		}
		else{
			phaseLagAct[i] = ( (fabs(diff2)<fabs(diff3)) ? diff2 : diff3 );
		}

		phaseLagKin[i] *= TIME_STEP*0.001*avgFrequency;
		phaseLagAct[i] *= TIME_STEP*0.001*avgFrequency;

		avgPhaseLagKin += phaseLagKin[i];
		avgPhaseLagAct += phaseLagAct[i];
	}
	// cout << endl;

	return true;
}

double
AnguilliformRobot :: firstOrderFilter(double input, double prevOutput, double timestep, double timeConstant)
{
	return exp(-timestep/timeConstant) * prevOutput + (1-exp(-timestep/timeConstant)) * input;
}

void
AnguilliformRobot :: meanStdVector(vector<double> vals, double &meanVal, double &stdVal)
{
	// mean
	meanVal = accumulate(vals.begin(),vals.end(),0.0) / vals.size();
	// std
	for(uint i=0; i<vals.size(); i++)
	{
		stdVal += (vals[i]-meanVal)*(vals[i]-meanVal);
	}
	stdVal = sqrt(1.0/vals.size()*stdVal);
}

void 
AnguilliformRobot :: initialCondition()
{
	for(int i=0;i<N_SERVOS-1;i++)
	{
		initTheta[i] = cdn_variable_get_value(d_cdn_theta[i]);
	}
}
