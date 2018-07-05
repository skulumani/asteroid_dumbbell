#!/bin/bash

IMG_PATH=$1
ASTEROIDS="castalia 52760"
HDF5_FILE="20180615_exploration_52760_15000.hdf5 20180615_exploration_castalia_15000.hdf5"

echo "Plot Castalia exploration data"
mkdir -p $IMG_PATH/castalia
PYTHONPATH=./ python exploration_sim.py -r $IMG_PATH/castalia ./data/exploration/explore/20180615_exploration_castalia_15000.hdf5 castalia
PYTHONPATH=./ python exploration_sim.py -u $IMG_PATH/castalia ./data/exploration/explore/20180615_exploration_castalia_15000.hdf5 castalia
PYTHONPATH=./ python exploration_sim.py -v $IMG_PATH/castalia ./data/exploration/explore/20180615_exploration_castalia_15000.hdf5 castalia
mkdir -p $IMG_PATH/castalia/state
PYTHONPATH=./ python exploration_sim.py -st $IMG_PATH/castalia/state ./data/exploration/explore/20180615_exploration_castalia_15000.hdf5 castalia
mkdir -p $IMG_PATH/castalia/refine
PYTHONPATH=./ python exploration_sim.py -rp $IMG_PATH/castalia/refine ./data/exploration/explore/20180615_exploration_castalia_15000.hdf5 castalia
mkdir -p $IMG_PATH/castalia/land
PYTHONPATH=./ python exploration_sim.py -lp $IMG_PATH/castalia/land ./data/exploration/refine/20180619_castalia_refinement.hdf5 castalia

echo "Plot 52760 exploration data"
mkdir -p $IMG_PATH/52760
PYTHONPATH=./ python exploration_sim.py -r $IMG_PATH/52760 ./data/exploration/explore/20180615_exploration_52760_15000.hdf5 52760
PYTHONPATH=./ python exploration_sim.py -u $IMG_PATH/52760 ./data/exploration/explore/20180615_exploration_52760_15000.hdf5 52760
PYTHONPATH=./ python exploration_sim.py -v $IMG_PATH/52760 ./data/exploration/explore/20180615_exploration_52760_15000.hdf5 52760
mkdir -p $IMG_PATH/52760/state
PYTHONPATH=./ python exploration_sim.py -st $IMG_PATH/52760/state ./data/exploration/explore/20180615_exploration_52760_15000.hdf5 52760
mkdir -p $IMG_PATH/52760/refine
PYTHONPATH=./ python exploration_sim.py -rp $IMG_PATH/52760/refine ./data/exploration/explore/20180615_exploration_52760_15000.hdf5 52760

cd $IMG_PATH/castalia
for file in $IMG_PATH/castalia/*.eps
do
    epstopdf $file
    rm -rf $file
done

cd $IMG_PATH/52760
for file in $IMG_PATH/52760/*.eps
do
    epstopdf $file
    rm -rf $file
done



