"""Exploration Simulation

Simulation of a spacecraft exploring an asteroid

States are chosen to minimize a cost function tied to the uncertainty of the 
shape. Full SE(3) dynamics and the polyhedron potential model is used

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
from __future__ import absolute_import, division, print_function, unicode_literals
