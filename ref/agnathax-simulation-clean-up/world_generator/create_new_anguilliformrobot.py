#!/usr/bin/python
import sys, os
from subprocess import call
from _world import World

segment_count = int(raw_input("Enter number of segments: "))

# anguilliform robot: 10 elements, 1 head segment, 4 tail segments
controller = 3
w = World(y_starting_position=-0.015, \
		  segment_number=segment_count,\
  		  segment_y_com = 0.0-0.005, \
		  segment_z_com = 0.029, \
		  segment_height=0.054,\
  		  segment_radius=0.014,\
		  segment_length=0.09-0.014,\
		  segment_width=0.054,\
		  segment_mass = 0.3+0.0556,\
		  head_y_com = -0.00553, \
		  head_z_com = 0.0192, \
		  head_segments = 0,\
		  #head_height = 0.054,\
		  head_height = 0.096,\
  		  head_radius = 0.0345,\
		  head_length = 0.136-0.0345,\
		  head_width = 0.069,\
		  head_mass = 0.45,\
		  tail_segments=4,\
		  tail_segments_per_module=1.575,\
		  tail_mass = 0.20, \
		  max_sim_time=25.0,\
		  archimede_factor=1.6,\
		  eig_mode=0,\
		  torque_control=controller ) # Example_Envirobot 0 , generic_default_controller 1, position_default_controller 2, sf_swimming_control 3

w.write()
w.writeHydroBoxes()
# w.set_controller_config()
#w.set_hydro_config()
# os.chdir('../controllers/sf_swimming_controller/')
# os.system('./compile_cont_phys.sh')
# os.chdir('../../world_generator/')
# w.write()
