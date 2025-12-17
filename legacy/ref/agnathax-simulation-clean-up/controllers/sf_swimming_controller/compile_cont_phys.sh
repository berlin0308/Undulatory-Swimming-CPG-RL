#!/bin/bash

current_folder=`pwd`

echo "set config-file"
cp $current_folder/config.hpp $current_folder/../../MyLib/. 

echo "compile controller"
make clean
make

echo "compile hydrodynamics with flexible tail"
make clean --directory=$current_folder/../../plugins/physics/anguilliformrobot_hydro_flex_tail/
make --directory=$current_folder/../../plugins/physics/anguilliformrobot_hydro_flex_tail/