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

