#!/bin/bash

# generate data all the data
DATA_PATH=$1
ASTEROIDS="castalia geographos golevka 52760"

echo "Going to generate data and the plots"
for ast in $ASTEROIDS
do 
    echo "Saving data to $DATA_PATH/$ast.hdf5"
    # ./bin/explore -o ${DATA_PATH}/$ast.hdf5 -n $ast
    echo "Now plotting $ast"
    mkdir -p $1/$ast
    PYTHONPATH=./ python dissertation/explore_plots.py -r ${DATA_PATH}/$ast.hdf5 $DATA_PATH/$ast
    PYTHONPATH=./ python dissertation/explore_plots.py -u ${DATA_PATH}/$ast.hdf5 $DATA_PATH/$ast
    PYTHONPATH=./ python dissertation/explore_plots.py -v ${DATA_PATH}/$ast.hdf5 $DATA_PATH/$ast
done

