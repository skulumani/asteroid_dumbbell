# Script to compare equations of motion
import eom_comparison.utilities as eom

# run inertial frame comparision
eom.inertial_frame_comparison(ast_name='castalia', num_faces=64, tf=1e3, num_steps=1e6)
