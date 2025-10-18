// ========================================================================================================================================================================================

// File:          optimizationhandler.hpp
// Date:          07-01-2014
// Description:   handles functions to communicate between webots and optimizer
// Author:        Robin Thandiackal
// Modifications: -

// ==============================================================================================================================================================================
#ifndef OPTIMIZATION_HPP
#define OPTIMIZATION_HPP

#include <optimization/webots.hh>
#include <sstream>
#include <string.h>


using namespace optimization::messages::task;

class optihandler
{
	public:
	std::vector<double> params;
	std::vector<double> settings;
	std::vector<std::string> params_names;
	std::vector<std::string> settings_names;
	  
	optihandler(optimization::Webots, int, int);
	void returnFitness(optimization::Webots, std::map<std::string,double>);
	~optihandler();

	private:
	void getOptiParameters(optimization::Webots);
	void getOptiSettings(optimization::Webots);
};

#endif 
