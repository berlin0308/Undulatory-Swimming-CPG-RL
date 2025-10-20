// ========================================================================================================================================================================================

// File:          optimizationhandler.cpp
// Date:          07-01-2014
// Description:   handles functions to communicate between webots and optimizer
// Author:        Robin Thandiackal
// Modifications: -

// ==============================================================================================================================================================================

#include "optimizationhandler.hpp"
using namespace std;

optihandler::optihandler(optimization::Webots opti, int nParameters, int nSettings)
{

	for(int i=0; i<nParameters; i++)
	{	
		std::stringstream s("");
                s << "P";
                s << i;
		params_names.push_back(s.str());
	}	
	for(int i=0; i<nSettings; i++)
	{	
		std::stringstream s("");
                s << "S";
                s << i;
		settings_names.push_back(s.str());
	}

	getOptiParameters(opti);
	getOptiSettings(opti);
}

void
optihandler::getOptiParameters(optimization::Webots opti)
{
	Task::Parameter p;
	for(uint i=0; i<params_names.size(); i++)
	{
		if (opti.Parameter(params_names[i],p))
        	{	
			params.push_back(p.value());
		}
		else
		{
			std::cerr << "optimization parameter ' " << params_names[i] << " ' not found! " << std::endl;
		}	
	}
}

void 
optihandler::getOptiSettings(optimization::Webots opti)
{
	Task::Parameter p;
	std::string param;
	double val;

	for(uint i=0; i<settings_names.size(); i++)
	{	
		if (opti.Setting(settings_names[i], param))
	      	{
        		std::stringstream s(param);
        		s >> val;
			settings.push_back(val);
		}
		else
		{
			std::cerr << "optimization setting ' " << settings_names[i] << " ' not found! " << std::endl;
		}
	}
}

void
optihandler::returnFitness(optimization::Webots opti,std::map<string,double> fitness)
{
	// Send the actual response
	if (opti.Setting("optiextractor"))
	{
		std::cout << "Not sending response, because in optiextractor mode! Enjoy the show!" << std::endl;
	}
	else
	{
		opti.Respond(fitness);
	}
}

optihandler::~optihandler()
{

}