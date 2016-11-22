## Purpose

This directory holds several large mat files that contain the data used to test 
the energy behavior of the inertial equations of motion of a dumbbell spacecraft
around an asteroid.

The inertial equations of motion are defined in `dynamics/ast_eoms_inertial.`

## Usage

The driver script that generates this data is in 
`dynamics/inertial_eoms_energy_behavior.m`


## Results

* `inertial_energy_behavior.mat` is with a small 2 m dumbbell in a periodic orbit.
The results doen't show much difference as the tolerance level is decreased
    Commit - `daaa56f8e8567a70783768e16b42fd4568b1d758`
* `inertial_energy_behavior_big_dumbbell.mat` - results with a 100 meter dummbell.
The results still do not show much difference 
    Commit - `af8d793fa8454737c3a87384984b6b5ce3ab6607`
