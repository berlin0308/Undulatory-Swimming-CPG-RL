# Anguilliform swimming simulation with local pressure feedback

## Content

- **world generator**: to adapt dimensions of the simulation model (see also MAR repository-->Behzad Bayat)
- **hydrodynamics** plugin
- **controller** to explore local feedback mechanisms (decoupled and purely sensory-driven model)
- job files for **systematic explorations** on the cluster
- **matlab scripts** for evaluation of logs

## Libraries

### Optimization libraries: 

	$ wget -qO- https://ponyo.epfl.ch/packages/sign.key | sudo apt-key add -  
	$ sudo add-apt-repository https://ponyo.epfl.ch/packages
	$ sudo apt-get update
	$ sudo apt-get install liboptimization2-dev

### Codyn 

For Ubuntu Trusty:	
	
	$ sudo wget -P /etc/apt/sources.list.d/ http://packages.codyn.net/ubuntu/codyn-trusty.list

Make sure to install the gpg key of the repository:
	
	$ wget -O - http://packages.codyn.net/sign.key | sudo apt-key add -

Install:
	
	sudo apt-get update
	sudo apt-get install codyn-all

For more details check: www.codyn.net

## Compiling

1. helper library: 

	navigate to misc/ > **make**
2. controller and physics: 

	navigate to controllers/sf_swimming_controller/ > **./compile_cont_phys.sh**

## World generator

navigate to world_generator/ > **./create_new_anguilliformrobot.py**

