RELATIVE_ROOT = ../../../
include $(RELATIVE_ROOT)/util.mk

###
### Standard Makefile for Webots physics plugins
###
### Supported platforms: Windows, Mac OS X, Linux
### Supported languages: C, C++
### 
### Authors: Olivier Michel - www.cyberbotics.com
### Revised: Yvan Bourquin - September 30th, 2009.
###
### Uncomment the variables to customize the Makefile

### -----C Sources-----
###
### if your plugin uses several C source files:
# C_SOURCES = archimede.c generic_hydro_flex_tail.c hydro_webots.c hydro.c ../../../worlds/hydro_boxes.c

### -----C++ Sources-----
###
### if your plugin uses several C++ source files:
# CPP_SOURCES = my_plugin.cpp my_clever_algo.cpp my_graphics.cpp
###     or
# CC_SOURCES = my_plugin.cc my_clever_algo.cc my_graphics.cc
CC_SOURCES = archimede.cc anguilliformrobot_hydro_flex_tail.cc hydro_webots.cc hydro.cc hydro_boxes.cc

### -----Options-----
###
### if special CFLAGS are necessary, for example to set optimization level or
### to find include files:
CFLAGS= -I"$(ROOT)" -g -O0 -std=c++11

###
### if your plugin needs additional libraries:
LIBRARIES= -L"$(ROOT)"/misc -lmisc

### Do not modify: this includes Webots global Makefile.include
WEBOTS_HOME:=/usr/local/webots
CC=/usr/bin/gcc
CXX=/usr/bin/g++
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

include ../webots-plugin.makefile
