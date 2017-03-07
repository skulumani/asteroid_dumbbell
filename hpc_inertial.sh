#!/bin/bash

# set output and error output filenames, %j will be replaced by Slurm with the jobid
#SBATCH -o dbint_%j.out
#SBATCH -e dbint_%j.err 

#SBATCH --mail-type=ALL
#SBATCH --mail-user=skulumani@gwu.edu

#SBATCH -N 1
#SBATCH -p short

# set the correct directory - cloned via git
#SBATCH -D /home/skulumani/asteroid_dumbbell

#SBATCH -J dbint
#SBATCH --export=NONE

#SBATCH -t 02-00:00:00

module load anaconda/4.2.0

python3 inertial_driver.py castalia 256 1e6 1e6 castalia_256_1e6_energy_behavior.npz -m 1
