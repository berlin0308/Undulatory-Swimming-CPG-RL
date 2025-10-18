###
### Standard Makefile for Webots controllers
### 
### Platform: Windows, Mac OS X, Linux
### Language: C, C++ and Java
### 
### Authors: Olivier Michel - www.cyberbotics.com
### Revised: Yvan Bourquin - September 30th, 2009.
###
### Uncomment the variables to customize the Makefile

### -----C Sources-----
###
### if your controller uses several C sources files:
# C_SOURCES = my_controller.c my_clever_algo.c my_gui.c

### -----C++ Sources-----
###
### if your controller uses several C++ sources files:
CPP_SOURCES = main.cpp anguilliformrobot_gen.cpp anguilliformrobot_opt.cpp anguilliformrobot_prf.cpp anguilliformrobot_run.cpp ../../MyLib/optimizationhandler.cpp
###     or
# CC_SOURCES = my_controller.cc my_clever_algo.cc my_gui.cc

### -----C/C++ Options-----
###
### if special CFLAGS are necessary, for example to set optimization level or
### to find include files:
# CFLAGS=-O3 -I/my_library_path/include
###
### if your controller needs additional libraries:
# LIBRARIES=-L/path/to/my/library -lmy_library -lmy_other_library
###
### if you want to use the C API in your C++ program add:
# USE_C_API=1
CFLAGS=-O2 `pkg-config --cflags liboptimization-2.0` `pkg-config --cflags codyn-3.0`
LIBRARIES=-lCppController -lController `pkg-config --libs liboptimization-2.0` `pkg-config --libs codyn-3.0`

### -----Java Options-----
###
### if your Java controller needs additional libraries, you should define
### the CLASSPATH environment variable as explained in the Webots User Guide
# CLASSPATH=relative/mylib.jar

### Do not modify: this includes Webots global Makefile.include
WEBOTS_HOME:=/usr/local/webots
CC=/usr/bin/gcc -w
CXX=/usr/bin/g++ -w
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/projects/default/controllers/Makefile.include
