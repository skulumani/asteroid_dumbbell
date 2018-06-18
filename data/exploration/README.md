* `explore.hdf5` - created using explore_main.cpp. Kinematics only simulation of cost function based 
exploration without any control cost. Plot the results using `dissertation/explore_plots.py`
* `explore_control.hdf5` - created using explore_control_main.cpp. Kinematics only simulation with control
component of the cost function. Plots results using `dissertation/explore_plots.py`
* `20180602_exploration_sim_control_sim_15000.hdf5` - exploration sim around asteroid castalia. Can recompute using 

    ~~~
    python exploration_sim.py -c <name of hdf5 to save>
    ~~~

    This is a little over one period around asteroid castalia

    If you want to generate the animation then run the following

    ~~~
    python exploration_sim.py -a <name of hdf5>
    ~~~

## Exploration data

Good exploration files are in exploration subdirectory

~~~
20180615_exploration_castalia_15000.hdf5
20180615_exploration_52760_15000.hdf5
~~~

Copy these and then run the refinement, followed by the landing

## Splitting large files

To split:

~~~
split -b 1024MiB <filename> <prefix>
~~~

To combine:

~~~
cat <prefix>* 
~~~

