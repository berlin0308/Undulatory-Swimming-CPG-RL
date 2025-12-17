import fileinput
import re
from random import *
from math import *
path = "."
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class World:
	_header_file = 'wbt_header.txt'
	_header_file_eig = 'wbt_header_eig.txt'
	_supervisor_file = 'wbt_supervisor.txt'
	_ground_file = 'wbt_ground.txt'
	_ground_file_eig = 'wbt_ground_eig.txt'
	_tail_file = 'wbt_tail_element.txt'
	_segment_file = 'wbt_body_element.txt'
	_segment_file_eig = 'wbt_body_element_eig.txt'
	_head_file = 'wbt_head_element.txt'
	_hydro_boxes_file = 'hydro_boxes.cc'
	_data_folder = 'data'


	def clean_indentation(self,data, ind_start = 0):
		tab = ''
		for i in range(0,ind_start):
			tab += '  ';
		new_data = '';
		for line in data.split('\n'):
			line = line.lstrip();
			new_data += tab + line + '\n';

			if (line.endswith('{') or line.endswith('[')):
				tab += '  ';
			if (line.endswith('}') or line.endswith(']')):
				tab = tab[:-2];

		return new_data

	def set_hydro_config(self):
		fname=path + '/../plugins/physics/anguilliformrobot_hydro_flex_tail/'+'hydro_webots.h'
		f=open(fname).read()
		text_file=re.sub(r'#define([\s]*)DEFAULT_ARCHIMEDE_FACTOR([\s]*)([0-9.]*)',('#define\tDEFAULT_ARCHIMEDE_FACTOR\t'+str(self.archimede_factor)),f)
		with open(fname, "w") as f:
			f.write(text_file)
		# print text_file
		return 

	def set_controller_config(self):
		fname=path + '/../MyLib/'+'config.hpp'
		f=open(fname).read()
		text_file=re.sub(r'#define([\s]*)N_SERVOS_TAIL([\s]*)([0-9.]*)',('#define\tN_SERVOS_TAIL\t'+str(int(self.number_of_segments_tail))),f)
		text_file=re.sub(r'#define([\s]*)MAX_TIME([\s]*)([0-9.]*)',('#define\tMAX_TIME\t'+str(self.max_sim_time)),text_file)
		with open(fname, "w") as f:
			f.write(text_file)
		# print text_file
		return 


	
	def _get_header(self):
		if self.eig_mode==0:
			with open(path + '/'+self._data_folder+'/'+self._header_file) as f:
				return f.read()
		elif self.eig_mode==1:
			with open(path + '/'+self._data_folder+'/'+self._header_file_eig) as f:
				return f.read()
	def _get_hydro_boxes(self):
		with open(path + '/'+self._data_folder+'/'+self._hydro_boxes_file) as f:
			return f.read()\
				.replace('__SEGMENT_LENGTH__',str(self.length_of_segments))\
				.replace('__SEGMENT_HEIGHT__', str(self.height_of_segments))\
				.replace('__SEGMENT_WIDTH__', str(self.width_of_segments))\
				.replace('__SEGMENT_RADIUS__', str(self.radius_of_segments))\
				.replace('__HEAD_SEGMENT_Y_COM__',str(self.com_y_of_head)) \
				.replace('__SEGMENT_Y_COM__',str(self.com_y_of_segments)) \
				.replace('__TAIL_SEGMENT_Y_COM__',str(self.com_y_of_segments)) \
				.replace('__HEAD_SEGMENT_Z_COM__',str(self.com_z_of_head)) \
				.replace('__SEGMENT_Z_COM__',str(self.com_z_of_segments)) \
				.replace('__TAIL_SEGMENT_Z_COM__',str(self.com_z_of_segments)) \
				.replace('__HEAD_LENGTH__',str(self.length_of_head))\
				.replace('__HEAD_HEIGHT__', str(self.height_of_head))\
				.replace('__HEAD_WIDTH__', str(self.width_of_head))\
				.replace('__HEAD_RADIUS__', str(self.radius_of_head))\
				.replace('__NUM_HEAD_ELEMENTS__', str(int(self.number_of_segments_head)))\
				.replace('__TAIL_SEGMENT_LENGTH__',str((self.length_of_segments/self.tail_segments_per_module)-0.0001) ) \
				.replace('__TAIL_SEGMENT_WIDTH__',str(self.width_of_tail) )

	def _get_supervisor(self):
		if self.torque_control==0:
			controller='Example_Envirobot'
		elif self.torque_control==1:
			controller='generic_default_controller'
		elif self.torque_control==2:
			controller='position_default_controller'
		elif self.torque_control==3:
			controller='sf_swimming_controller'
		elif self.torque_control==4:
			controller='position_controller'	
		with open(path + '/'+self._data_folder+'/'+self._supervisor_file) as f:
			return f.read()\
				.replace('__JOINTS__',self._get_body()) \
				.replace('__JOINT_CONTROLLER__',controller) \
				.replace('__Y_STARTING_POSITION__',str(self.y_starting_position)) \
				.replace('__HEAD_SEGMENT_Y_COM__',str(self.com_y_of_head)) \
				.replace('__HEAD_SEGMENT_Z_COM__',str(self.com_z_of_head)) \
				.replace('__HEAD_SUB_SEGMENT_CYL_Z_COM__',str(-self.com_z_of_head)) \
				.replace('__HEAD_SUB_SEGMENT_BOX_Z_COM__',str(self.length_of_head/2-self.com_z_of_head) ) \
				.replace('__HEAD_SUB_SEGMENT_Y_COM__',str(-self.com_y_of_head)) \
				.replace('__HEAD_SUB_SEGMENT_Z_COM__',str(-self.com_z_of_head)) \
				.replace('__SEGMENT_LENGTH__',str(self.length_of_head))\
				.replace('__SEGMENT_HEIGHT__', str(self.height_of_head))\
				.replace('__SEGMENT_WIDTH__', str(self.width_of_head))\
				.replace('__SEGMENT_RADIUS__', str(self.radius_of_head))\
				.replace('__SEGMENT_HEIGHT_BOX_M__', str(self.height_of_head/2.0))\
				.replace('__SEGMENT_HEIGHT_BOX_UD__', str(self.height_of_head/4.0))\
				.replace('__SEGMENT_TRANSLATION_Y_UD__', str(self.height_of_head*3/8.0))\
				.replace('__SEGMENT_TRANSLATION_Z_BOX__', str(self.length_of_head/2) )\
				.replace('__SEGMENT_MASS__', str(self.mass_of_head) )

	def _get_ground(self):
		if self.eig_mode==0:
			with open(path + '/'+self._data_folder+'/'+self._ground_file) as f:
				return f.read()
		elif self.eig_mode==1:
			with open(path + '/'+self._data_folder+'/'+self._ground_file_eig) as f:
				return f.read()
	def _get_tail(self):
		
		with open(path + '/'+self._data_folder+'/'+self._tail_file) as f:
			data = f.read()

		next_tail_element = 'DEF ANCHOR Transform {\ntranslation 0 0 0.0249\n}';
		tab = '     ';
		# for i in range(self.number_of_segments+self.number_of_segments_tail,self.number_of_segments,-1):
		for i in range(self.number_of_segments+self.number_of_segments_tail,self.number_of_segments,-1):
			if i == self.number_of_segments+1:
				# joint_anchor_translation = 0.0955#0.0725;
				joint_servo_translation = (self.length_of_segments) #0.0955#0.0726;
			else:
				# joint_anchor_translation = 0.0249;
				joint_servo_translation = self.length_of_segments/self.tail_segments_per_module;
				
			joint_name = 'joint_servo_' + str(i)
			joint_name_capital = joint_name.upper();
			tail_segment_length = self.length_of_segments/self.tail_segments_per_module

			
			data_tmp = data\
			.replace('__JOINT_SERVO_TRANSLATION__', str(joint_servo_translation))\
			.replace('__SEGMENT_HEIGHT__', str(self.height_of_segments))\
			.replace('__SEGMENT_HEIGHT_BOX_M__', str(self.height_of_segments/2.0))\
			.replace('__SEGMENT_HEIGHT_BOX_UD__', str(self.height_of_segments/4.0))\
			.replace('__SEGMENT_TRANSLATION_Y_UD__', str(self.height_of_segments*3/8.0))\
			.replace('__TAIL_SEGMENT_LENGTH__',str((self.length_of_segments/self.tail_segments_per_module)-0.0001) )\
			.replace('__TAIL_SEGMENT_WIDTH__',str( -(self.width_of_segments-0.001)/(self.number_of_segments_tail*tail_segment_length)*(i-self.number_of_segments)*tail_segment_length+self.width_of_segments ) )\
			.replace('__TAIL_SEGMENT_TRANSLATION_Z_SUBBOX__',str( ( (self.length_of_segments/self.tail_segments_per_module)-0.0001)/2.0 ) )\
			.replace('__TAIL_SEGMENT_TRANSLATION_Z_BOX__',str( ( (self.length_of_segments/self.tail_segments_per_module)-0.0001)/2.0 -0.00055) )\
			.replace('__TAIL_SEGMENT_Y_COM', str( self.com_y_of_segments ) ) \
			.replace('__TAIL_SEGMENT_Z_COM', str( self.length_of_segments/self.tail_segments_per_module/2.0 ) )\
			.replace('__TAIL_SUB_SEGMENT_Y_COM', str( -self.com_y_of_segments ) ) \
			.replace('__TAIL_SUB_SEGMENT_Z_COM', str( -self.length_of_segments/self.tail_segments_per_module/2.0 ) ) \
			.replace('__SEGMENT_MASS__', str(self.mass_of_tail * (self.number_of_segments_tail+1-(i-self.number_of_segments))/(0.5*self.number_of_segments_tail*(self.number_of_segments_tail+1))) )\
			.replace('__JOINT_NAME__', joint_name)\
			.replace('__JOINT_NAME_CAPITAL__', joint_name_capital)\
			.replace('__NEXT_TAIL_ELEMENT__', next_tail_element)

			next_tail_element = data_tmp;

		return '\n'.join( [ line.lstrip() for line in next_tail_element.split('\n')])

	def _get_body(self):
		
		if self.eig_mode==0:
			with open(path + '/'+self._data_folder+'/'+self._segment_file) as f:
				data = f.read()
		elif self.eig_mode==1:
			with open(path + '/'+self._data_folder+'/'+self._segment_file_eig) as f:
				data = f.read()

		next_element = self._get_tail();
		for i in range(self.number_of_segments,self.number_of_segments_head,-1):
			joint_name = 'joint_servo_' + str(i)
			joint_name_capital = joint_name.upper();
			
			if i==(self.number_of_segments_head+1):
				z=(self.length_of_head)+(self.radius_of_segments)
			else:
				z=(self.length_of_segments)+(self.radius_of_segments)

			data_tmp = data\
			.replace('__JOINT_NAME__', joint_name)\
			.replace('__JOINT_NAME_CAPITAL__', joint_name_capital)\
			.replace('__NEXT_ELEMENT__', next_element)\
			.replace('__SEGMENT_Y_COM__',str(self.com_y_of_segments)) \
			.replace('__SEGMENT_Z_COM__',str(self.com_z_of_segments)) \
			.replace('__SUB_SEGMENT_Y_COM__',str(-self.com_y_of_segments)) \
			.replace('__SUB_SEGMENT_CYL_Z_COM__',str(-self.com_z_of_segments)) \
			.replace('__SUB_SEGMENT_BOX_Z_COM__',str(self.length_of_segments/2-self.com_z_of_segments) ) \
			.replace('__SEGMENT_LENGTH__', str(self.length_of_segments) )\
			.replace('__SEGMENT_HEIGHT__', str(self.height_of_segments))\
			.replace('__SEGMENT_WIDTH__', str(self.width_of_segments))\
			.replace('__SEGMENT_WIDTH_M__', str(self.width_of_segments/2.0))\
			.replace('__SEGMENT_RADIUS__', str(self.radius_of_segments))\
			.replace('__SEGMENT_HEIGHT_BOX_M__', str(self.height_of_segments/2.0))\
			.replace('__SEGMENT_HEIGHT_BOX_UD__', str(self.height_of_segments/4.0))\
			.replace('__SEGMENT_TRANSLATION_Y_UD__', str(self.height_of_segments*3/8.0))\
			.replace('__SEGMENT_TRANSLATION_Z__', str(z) )\
			.replace('__SEGMENT_TRANSLATION_Z_BOX__', str( self.length_of_segments/2.0 ) )\
			.replace('__SEGMENT_TRANSLATION_Z_SUBBOX__', str((self.length_of_segments/2.0)+0.0025) )\
			.replace('__SEGMENT_COM_Z__', str(self.com_z_of_segments) ) \
			.replace('__SEGMENT_MASS__', str(self.mass_of_segments) )#maybe include cyliner too for the density

			next_element = data_tmp;
		
		#preparing head segments
		for i in range(self.number_of_segments_head,0,-1):
			joint_name = 'joint_servo_' + str(i)
			joint_name_capital = joint_name.upper();
			
			data_tmp = data\
			.replace('__JOINT_NAME__', joint_name)\
			.replace('__JOINT_NAME_CAPITAL__', joint_name_capital)\
			.replace('__NEXT_ELEMENT__', next_element)\
			.replace('__SEGMENT_Y_COM__',str(self.com_y_of_head)) \
			.replace('__SEGMENT_Z_COM__',str(self.com_z_of_head)) \
			.replace('__SUB_SEGMENT_Y_COM__',str(-self.com_y_of_head)) \
			.replace('__SUB_SEGMENT_Z_COM__',str(-self.com_z_of_head)) \
			.replace('__SUB_SEGMENT_CYL_Z_COM__',str(-self.com_z_of_head)) \
			.replace('__SUB_SEGMENT_BOX_Z_COM__',str(self.length_of_head/2-self.com_z_of_head) ) \
			.replace('__SEGMENT_LENGTH__', str(self.length_of_head))\
			.replace('__SEGMENT_HEIGHT__', str(self.height_of_head))\
			.replace('__SEGMENT_WIDTH__', str(self.width_of_head))\
			.replace('__SEGMENT_RADIUS__', str(self.radius_of_head))\
			.replace('__SEGMENT_HEIGHT_BOX_M__', str(self.height_of_head/2.0))\
			.replace('__SEGMENT_HEIGHT_BOX_UD__', str(self.height_of_head/4.0))\
			.replace('__SEGMENT_TRANSLATION_Y_UD__', str(self.height_of_head*3/8.0))\
			.replace('__SEGMENT_TRANSLATION_Z__', str((self.length_of_head)+(self.radius_of_head)) )\
			.replace('__SEGMENT_TRANSLATION_Z_BOX__', str(self.length_of_head/2) )\
			.replace('__SEGMENT_TRANSLATION_Z_SUBBOX__', str((self.length_of_head/2)+0.0025) )\
			.replace('__SEGMENT_MASS__', str( self.height_of_head*(self.length_of_head+self.radius_of_head)*0.548/(0.1165*0.0945)) )
			
			next_element = data_tmp;	

		return '\n'.join( [ line.lstrip() for line in next_element.split('\n')])	



	def __init__(self,*args,**kwargs):
		self.y_starting_position = kwargs.get('y_starting_position',-0.01)
		self.number_of_segments = kwargs.get('segment_number',8)
		self.com_y_of_segments = kwargs.get('segment_y_com',-0.02)
		self.com_z_of_segments = kwargs.get('segment_z_com',0.01)
		self.length_of_segments = kwargs.get('segment_length',0.093)
		self.height_of_segments = kwargs.get('segment_height',0.0945)
		self.width_of_segments = kwargs.get('segment_width',0.047)
		self.radius_of_segments = kwargs.get('segment_radius',0.01665)
		self.mass_of_segments = kwargs.get('segment_mass',0.3)
		self.number_of_segments_head = kwargs.get('head_segments',1)
		# self.com_z_of_segments = kwargs.get('com_z_segments',0.038)
		self.com_y_of_head = kwargs.get('head_y_com',-0.02)
		self.com_z_of_head = kwargs.get('head_z_com',0.064)
		self.length_of_head = kwargs.get('head_length',0.093)
		self.height_of_head = kwargs.get('head_height',0.0945)
		self.width_of_head = kwargs.get('head_width',0.047)
		self.radius_of_head = kwargs.get('head_radius',0.01665)
		self.mass_of_head = kwargs.get('head_mass',0.45)		
		self.archimede_factor = kwargs.get('archimede_factor',1.5)
		self.max_sim_time = kwargs.get('max_sim_time',30.0)
		self.number_of_segments_tail = kwargs.get('tail_segments',5)
		self.tail_segments_per_module = kwargs.get('tail_segments_per_module',4.0)
		self.mass_of_tail = kwargs.get('tail_mass',0.57)
		self.width_of_tail = kwargs.get('tail_width',0.01)
		self.torque_control = kwargs.get('torque_control',1)
		self.torque_gain = kwargs.get('torque_gain',5)
		self.network_w_up	=kwargs.get('network_w_up',10)
		self.network_w_down	=kwargs.get('network_w_down',10)
		self.alpha		 	=kwargs.get('alpha', 	1.0)
		self.gamma		 	=kwargs.get('gamma', 	2.0)
		self.delta		 	=kwargs.get('delta',	0.1)
		self.pc_fb_gain 	=kwargs.get('pc_fb_gain', 	0)
		self.ec_fb_gain 	=kwargs.get('ec_fb_gain',  0)
		self.tail_p_stiffness =kwargs.get('tail_p_stiffness', 	2.0)
		self.tail_p_damping   =kwargs.get('tail_p_damping', 	0.0)
		self.default_phi_lag_percentage = kwargs.get('default_phi_lag_percentage',100)
		self.freq=kwargs.get('freq',0.5)
		self.gamma_scaling=kwargs.get('gamma_scaling',1.0)
		self.num_passive_elements=kwargs.get('num_passive_elements',0)
		self.eig_mode=kwargs.get('eig_mode',0)
		self.world = self._get_header()+'\n'+self._get_ground() + '\n' + self._get_supervisor();
		self.world = '\n'.join( [ line.lstrip() for line in self.world.split('\n')])
		self.hydro_boxes = self._get_hydro_boxes();

		
	def get(self):
		return self.world
	def write(self):
		if self.eig_mode==0:
			filename='anguilliformrobot'+'_segment'+ str(self.number_of_segments) +'_tail'+ str(self.number_of_segments_tail)+'.wbt';
		elif self.eig_mode==1:
			filename='anguilliformrobot'+'_segment'+ str(self.number_of_segments) +'_tail'+ str(self.number_of_segments_tail)+'_eig.wbt';			
		print bcolors.OKGREEN + filename + bcolors.ENDC
		with open(path+'/../worlds/'+filename, "w") as text_file:
			text_file.write(self.clean_indentation(self.world))
	def writeHydroBoxes(self):
		with open(path+'/../plugins/physics/anguilliformrobot_hydro_flex_tail/hydro_boxes.cc', "w") as text_file:
			text_file.write(self.hydro_boxes)			
			
	def writeWaveData(self,filename):
		with open(filename, "w") as text_file:
			text_file.write(''.join([str(i) + ' ' + str(j) + '\n' for i,j in zip(self.wavy_ground.positions,self.wavy_ground.slopes)]))
