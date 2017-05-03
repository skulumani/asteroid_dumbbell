## Dumbbell about and asteroid

Simulation to test the motion of a dumbbell spacecraft around an asteroid

## Repository setup

Clone and run `chmod +x setup_repo.sh` then `./setup_repo.sh` to automatically create the correct remote repositories. 
This will ensure that pushes are sent to both:

* [Github](https://github.com/skulumani/asteroid_dumbbell)
* [Bitbucket](https://bitbucket.org/shankarkulumani/asteroid_dumbbell)

## MEX Guide

Look in [Mex Guide](./docs/mex_guide.md) to learn how to compile a mex function.
You need to create a mex version of `polyhedron_potential.m`. 

In addition, you have to have both `polyhedron_potential_mex_1024.mex` and `polyhedron_potential_mex_4092.mex`

## Usage guide

You can run the simulation for both the inertial and relative equations of motion. 
There are driver modules for each, which are called:
    * `inertial_driver.py` - Driver functions to simulate the inertial equations of motion
    * `relative_driver.py` - Driver functions to simulate the relative equations of motion
    * `dumbbell_driver.py` - More driver functions which were used during testing/debuggin
    * `eom_comparison.py` - Functions to allow the comparision between the different EOMS

## Using `mayavi`

There's a conda enviornment which has `mayavi` working correctly.
You can use the following to duplicate the enviornment
~~~
$ conda env export > env.yml
$ conda env create -f mayavi_enviornment.yml
~~~

## [Profiling](https://github.com/barbagroup/numba_tutorial_scipy2016/blob/master/notebooks/01.When.where.to.use.Numba.ipynb)

To profile the Python code you can use `cProfile` or `line-profiler`
~~~
pip install cProfile line-profiler snakeviz
~~~

* Use `cProfile` to find which function call is taking the most time out of a bigger script
~~~
import cProfile
cProfile.run('script to execute as a string')
~~~
    * You can also do this from within iPython as
    ~~~
    %prun -D output.prof function()
    ~~~
* Next use snakeviz to visualize it
~~~
%load_ext snakeviz
%snakeviz function()
~~~
* Once you have an idea of the slow function you can find specific lines within the function 
using `line-profiler`
~~~
%load_ext line_profiler
%lprun -T output.txt -f ast.function() script()
~~~

