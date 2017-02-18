#!/bin/bash

# set output and error output filenames, %j will be replaced by Slurm with the jobid
#SBATCH -o ast_int_%j.out
#SBATCH -e ast_int_%j.err 

#SBATCH --mail-type=ALL
#SBATCH --mail-user=skulumani@gwu.edu

#SBATCH -N 1
#SBATCH -p defq

# set the correct directory - cloned via git
#SBATCH -D /home/skulumani/asteroid_dumbbell

#SBATCH -J ast_int
#SBATCH --export=NONE

#SBATCH -t 0-01:00:00

module load anaconda/4.2.0

python3 inertial_driver.py