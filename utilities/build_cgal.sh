#!/bin/bash

# install cgal using conda
echo "Verify you're using the correct conda environment"

read -p "Enter to install CGAL"

conda install -c conda-forge cgal
