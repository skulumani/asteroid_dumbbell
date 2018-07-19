#!/bin/bash

# generate data all the data
DATA_PATH=$1
ASTEROIDS="castalia geographos golevka 52760"

echo "Going to generate data and the plots"
for ast in $ASTEROIDS
do 
    if [ -e "$DATA_PATH/$ast.hdf5" ]; then
        echo "File exists"
    else 
        echo "File does not exist"
        ./bin/explore -o ${DATA_PATH}/$ast.hdf5 -n $ast
    fi 
    echo "Now plotting $ast"
    mkdir -p $1/$ast

    PYTHONPATH=./ python dissertation/explore_plots.py -r ${DATA_PATH}/$ast.hdf5 $DATA_PATH/$ast &
    PS1=$!
    PYTHONPATH=./ python dissertation/explore_plots.py -u ${DATA_PATH}/$ast.hdf5 $DATA_PATH/$ast &
    PS2=$!
    PYTHONPATH=./ python dissertation/explore_plots.py -v ${DATA_PATH}/$ast.hdf5 $DATA_PATH/$ast &
    PS3=$!
    PYTHONPATH=./ python dissertation/explore_plots.py -a ${DATA_PATH}/$ast.hdf5 $DATA_PATH/$ast &
    PS4=$! 
    PYTHONPATH=./ python dissertation/explore_plots.py -mw -a ${DATA_PATH}/$ast.hdf5 $DATA_PATH/$ast &
    PS5=$!
    wait $PS1 $PS2 $PS3 $PS4 $PS5
done

