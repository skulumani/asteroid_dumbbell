#!/bin/bash

# set output and error output filenames, %j will be replaced by Slurm with the jobid
#SBATCH -o dbrel_%j.out
#SBATCH -e dbrel_%j.err 

#SBATCH --mail-type=ALL
#SBATCH --mail-user=skulumani@gwu.edu

#SBATCH -N 1
#SBATCH -p short

# set the correct directory - cloned via git
#SBATCH -D /home/skulumani/asteroid_dumbbell

#SBATCH -J dbrel
#SBATCH --export=NONE

#SBATCH -t 00-06:00:00

module load anaconda/4.2.0

python3 relative_driver.py castalia 64 1e5 1e5 castalia_64_1e5_relative_energy_behavior.npz -m 0
