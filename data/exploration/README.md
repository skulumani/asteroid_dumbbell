
## Running the simulation
    
1. Run exploration

    ~~~
    python exploration_sim.py -c <name of hdf5 to save>
    ~~~

    This is a little over one period around asteroid castalia

    If you want to generate the animation then run the following

    ~~~
    python exploration_sim.py -a <name of hdf5>
    ~~~
    
2. Run the refinement

    ~~~
    python exploration_sim.py -lkr <name of HDF5 file from exploration> <name of asteroid>
    ~~~

    To animate the refinement process

    ~~~
    python exploration_sim.py -lra <name of HDF5 file> <name of asteroid>
    ~~~
    
3. Run the landing

    ~~~
    python exploration_sim.py -l <name of hdf5> <name of asteroid>
    ~~~

    To animate the landing

    ~~~
    python exploration_sim.py -la <name of HDF5> <name of asteroid>
    ~~~

Each stage will add an additional group with the HDF5 file. The structure of the groups is:

~~~
simulation_parameters
reconstruction
refinement
landing
~~~

## Exploration data

Good exploration files are in exploration subdirectory

~~~
20180615_exploration_castalia_15000.hdf5
20180615_exploration_52760_15000.hdf5
~~~

Copy these and then run the refinement, followed by the landing

The appropriate truth models for the asteroids are stored within Git

## Refinement

Copies of exploration data and refinement run using a bumpy version of Castalia

~~~
20180619_castalia_refinement.hdf5
~~~

## Landing

Now run landing based on the refinement

~~~
20180619_castalia_landing.hdf5
~~~

It includes all the data from the exploration and refinement

## Splitting large files

To split:

~~~
split -b 1024MiB <filename> <prefix>
~~~

To combine:

~~~
cat <prefix>* 
~~~

